// ============================================================================
//  pad_calib.cu — pad_calib.h 的实现:
//    [1] 激励计划表 (先 X 后 Y, 每轴逐级, 每级 ± 各一段, 每段后接零偏转停顿) 与
//        段窗口表;
//    [2] 自适应段时长 (行程目标 / 已测屏速, 夹在中段统计下限与流程预算上限之间);
//    [3] 拟合: 停顿静止窗估噪声底 σ → 停顿边沿实测 L → 逐段中段取样求增益
//        (无延迟对齐) → 段/级有效性判定 → 逐级池化 → 幂律外推 (或单级线性回退);
//    [4] 标定状态机 (相位与 hid 标定同形, 状态自成一态), 采样/计算/回写经
//        g_calib_collect/g_calib_request/g_calib_done 三原子与 io/capture.cu 的
//        ai_thread 交接 (与 hid 标定共用这条链路, 但激励表与拟合路径完全独立)。
// ============================================================================

#include "io/pad_calib.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>

#include "core/state.h"

std::atomic<float> g_pad_probe_px_s{0.0f};
std::atomic<float> g_pad_probe_d{0.0f};

// ========================= 计划表与段窗口 =========================

const std::vector<PadPlanSeg>& pad_cal_plan() {
    static const std::vector<PadPlanSeg> plan = [] {
        std::vector<PadPlanSeg> p;
        for (int axis = 0; axis < PAD_CAL_AXES_N; ++axis)
            for (int li = 0; li < PAD_CAL_LEVELS_N; ++li)
                for (int k = 0; k < PAD_CAL_SEGS_PER_LEVEL; ++k) {
                    const int16_t d = pad_level_defl(PAD_CAL_LEVELS[li]);
                    const int16_t ex = (int16_t)(k ? -d : d);         // ± 交替
                    p.push_back({axis, li, PAD_CAL_LEVELS[li], ex,
                                 ms_to_ticks(PAD_CAL_SEG_MS_MAX), false});
                    p.push_back({axis, li, 0.0f, 0,
                                 ms_to_ticks(PAD_CAL_PAUSE_MS), true});   // 段后停顿
                }
        return p;
    }();
    return plan;
}

void PadExcPlan::reset() {
    auto p = pad_cal_plan();
    std::lock_guard<std::mutex> lk(mtx_);
    segs_.clear();
    segs_.reserve(p.size());
    for (auto& s : p) { PadExcSeg e; e.axis = s.axis; e.level = s.level;
                        e.d = s.d; e.defl = s.defl; e.pause = s.pause; segs_.push_back(e); }
}
void PadExcPlan::begin_seg(int i, std::chrono::steady_clock::time_point t) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (i >= 0 && i < (int)segs_.size()) { segs_[i].t0 = t; segs_[i].t1 = t; }
}
void PadExcPlan::end_seg(int i, std::chrono::steady_clock::time_point t) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (i >= 0 && i < (int)segs_.size()) segs_[i].t1 = t;
}
void PadExcPlan::clear() {
    std::lock_guard<std::mutex> lk(mtx_);
    segs_.clear();
}
std::vector<PadExcSeg> PadExcPlan::snapshot() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return segs_;
}
PadExcPlan g_pad_exc_plan;

// ========================= 自适应段时长 =========================

int pad_cal_seg_ticks(float v_meas_px_s, float d_meas, float d_next, int cam_fps) {
    const int lo = pad_cal_seg_ms_min(cam_fps);
    int ms = lo;                                  // 探针级 (尚无实测) = 统计下限
    if (v_meas_px_s > 0.0f && d_meas > 0.0f && d_next > 0.0f) {
        double v_next = (double)v_meas_px_s
                      * std::pow((double)d_next / (double)d_meas, (double)PAD_CAL_CURVE_P_MAX);
        if (v_next > 1e-6) ms = (int)((double)PAD_CAL_TRAVEL_PX / v_next * 1000.0 + 0.5);
    }
    return ms_to_ticks(std::clamp(ms, lo, PAD_CAL_SEG_MS_MAX));
}

// ========================= 拟合 =========================

namespace {

// 停顿的静止参考窗起点 (×P): 保证 L ≤ 100ms (设计保证) 时窗内样本已完全静止
inline std::chrono::steady_clock::time_point pause_tail(
        const std::chrono::steady_clock::time_point& t0) {
    return shift_ms(t0, (double)PAD_CAL_PAUSE_MS * (double)PAD_CAL_PAUSE_TAIL);
}
// 段的命令区间 = [t0, t1 + 一拍]; 中段窗 = [段起+P, 命令末−一帧]
inline std::chrono::steady_clock::time_point seg_end(
        const std::chrono::steady_clock::time_point& t1) {
    return shift_ms(t1, (double)TICK_MS);
}
inline std::chrono::steady_clock::time_point mid_begin(
        const std::chrono::steady_clock::time_point& t0) {
    return shift_ms(t0, (double)PAD_CAL_PAUSE_MS);
}
inline std::chrono::steady_clock::time_point mid_end(
        const std::chrono::steady_clock::time_point& t1, double frame) {
    return shift_ms(t1, (double)TICK_MS - frame);
}

std::vector<const CalibSample*> pick(const std::deque<CalibSample>& hist,
                                     std::chrono::steady_clock::time_point a,
                                     std::chrono::steady_clock::time_point b) {
    std::vector<const CalibSample*> v;
    if (b < a) return v;
    for (const auto& s : hist) if (s.t >= a && s.t <= b) v.push_back(&s);
    return v;
}

float median_of(std::vector<float> v) {
    if (v.empty()) return 0.0f;
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
}
float median_abs(const std::vector<float>& v) {
    if (v.empty()) return 0.0f;
    std::vector<float> a; a.reserve(v.size());
    for (float x : v) a.push_back(std::fabs(x));
    return median_of(a);
}

// 逐段的中段统计结果
struct SegOut {
    bool ok = false;
    const char* why = "";
    double g = 0, se = 0, cc = 0, dc = 0;
    int n = 0;
    float resp = 0, val = 0, lim = 0;
};

// 单级判定与池化: 通过则 dg.valid=true、g = 池化增益 (px per 偏转·ms) 并返回 true
bool level_gain(const std::vector<const SegOut*>& sv, PadCalLevelDiag& dg, double& g) {
    dg.seg_all = (int)sv.size();
    for (const SegOut* o : sv) {                      // 任一段失效 → 整级丢弃
        if (o->ok) continue;
        dg.why = o->why; dg.val = o->val; dg.lim = o->lim;
        return false;
    }
    dg.seg_ok = (int)sv.size();
    dg.resp = sv[0]->resp;
    for (const SegOut* o : sv) dg.resp = std::min(dg.resp, o->resp);
    if (sv.size() >= 2) {                             // 同级两段一致性 (方向不该改响应)
        const SegOut* a = sv[0]; const SegOut* b = sv[1];
        double se = std::sqrt(a->se * a->se + b->se * b->se);
        if (std::fabs(a->g - b->g) > (double)PAD_CAL_SEG_Z * se) {
            dg.why = "同级两段增益不一致 (方向/量程异常)";
            dg.val = (float)std::fabs(a->g - b->g); dg.lim = (float)((double)PAD_CAL_SEG_Z * se);
            dg.seg_ok = 0;
            return false;
        }
    }
    double cc = 0, dc = 0;
    for (const SegOut* o : sv) { cc += o->cc; dc += o->dc; }
    if (cc <= 0) { dg.why = "账本激发量为零"; return false; }
    g = dc / cc;
    if (g <= 0) { dg.why = "池化增益非正"; return false; }
    // 该级实测屏速 px/s = g [px/(偏转·ms)] × 满偏比例 × 满偏量 × 1000ms/s
    dg.px_s = (float)(g * (double)dg.d * (double)PAD_AXIS_MAX * 1000.0);
    dg.valid = true;
    return true;
}

void assemble_axis(int axis, const std::vector<SegOut>& so,
                   const std::vector<PadExcSeg>& plan, PadCalibResult& r) {
    std::vector<double> lx, ly;                       // 对数域拟合点 (log d, log g)
    for (int li = 0; li < PAD_CAL_LEVELS_N; ++li) {
        PadCalLevelDiag& dg = r.lv[axis][li];
        dg.d = PAD_CAL_LEVELS[li];
        std::vector<const SegOut*> sv;
        for (size_t k = 0; k < plan.size(); ++k)
            if (!plan[k].pause && plan[k].axis == axis && plan[k].level == li)
                sv.push_back(&so[k]);
        double g = 0;
        if (level_gain(sv, dg, g)) {
            lx.push_back(std::log((double)dg.d));
            ly.push_back(std::log(g));
        }
    }
    // 级数降级: ≥2 级 → 对数域最小二乘 (级间等权 — 每点的增益已是该级全部有效段的
    //   池化); 恰好 1 级 → 线性回退 (p=1, 指数未测); 0 级 → 失败
    r.nlv[axis] = (int)lx.size();
    if (lx.empty()) return;
    double s_full, p;
    if (lx.size() == 1) {
        s_full = std::exp(ly[0]); p = 1.0; r.linear[axis] = true; r.res[axis] = 0.0f;
    } else {
        const int m = (int)lx.size();
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int i = 0; i < m; ++i) { sx += lx[i]; sy += ly[i];
                                      sxx += lx[i] * lx[i]; sxy += lx[i] * ly[i]; }
        double den = (double)m * sxx - sx * sx;
        double b = den != 0 ? ((double)m * sxy - sx * sy) / den : 0.0;
        double a = (sy - b * sx) / m;
        s_full = std::exp(a); p = 1.0 + b;
        double acc = 0;
        for (int i = 0; i < m; ++i) { double e = ly[i] - (a + b * lx[i]); acc += e * e; }
        r.res[axis] = (float)std::sqrt(acc / m);
    }
    if (s_full > 0 && std::isfinite(s_full)) {
        r.gain[axis] = (float)pad_gain_from_s_rp((float)s_full);
        r.p[axis] = (float)p;
    }
}

} // namespace

PadCalibResult pad_calib_fit(const std::deque<CalibSample>& hist,
                             const std::vector<PadExcSeg>& plan, float shift_max) {
    PadCalibResult r;
    if ((int)hist.size() < CALIB_WINDOW) { r.err = "样本不足"; return r; }
    if (plan.empty()) { r.err = "无激励计划 (状态机未跑完)"; return r; }
    for (const auto& s : plan)
        if (s.t1 < s.t0) { r.err = "段窗口不完整 (激励未播完)"; return r; }

    // 帧周期: 由样本 dt 的中位给出 (拟合不依赖采集参数)
    std::vector<float> dts; dts.reserve(hist.size());
    for (const auto& s : hist) dts.push_back(s.dt_ms);
    const double frame = std::max(1.0, (double)median_of(dts));

    // ---- [1] 噪声底 σ: 停顿的静止参考窗 (尾段, 保证 L ≤ 100ms 时已完全静止) ----
    std::vector<float> nzv[PAD_CAL_AXES_N];
    for (const auto& sg : plan) {
        if (!sg.pause) continue;
        auto v = pick(hist, pause_tail(sg.t0), seg_end(sg.t1));
        for (auto* s : v) nzv[sg.axis].push_back(sg.axis ? s->sy : s->sx);
    }
    for (int a = 0; a < PAD_CAL_AXES_N; ++a) {
        // σ̂ = 1.4826×中位|·| (稳健尺度); 中位估计的标准误差 ≈ 1.25/√n, 要 ≤30% 需 n ≥ 18
        if ((int)nzv[a].size() < PAD_CAL_SIGMA_MIN_N) { r.err = "静止参考样本不足"; return r; }
        r.sigma[a] = 1.4826f * median_abs(nzv[a]);
        // σ ≈ 0 = 停顿里的样本位移恒为零: 画面完全静止 (无游戏/游戏画面冻结),
        //   既无噪声尺度也无响应可判 — 失败收尾, 不硬算
        if (!(r.sigma[a] > 1e-3f)) {
            r.err = "画面完全静止 (停顿样本位移恒为 0): 无游戏或画面未响应";
            return r;
        }
    }

    // ---- [2] 逐段中段统计 (中段样本的账本窗完全落在段内, 增益无需延迟对齐) ----
    const CountsHistory& led = own_motion_ledger();
    std::vector<SegOut> so(plan.size());
    for (size_t k = 0; k < plan.size(); ++k) {
        const PadExcSeg& sg = plan[k];
        if (sg.pause) continue;
        SegOut& o = so[k];
        auto v = pick(hist, mid_begin(sg.t0), mid_end(sg.t1, frame));
        double cc = 0, dc = 0, sdir = 0; float spmax = 0, dpmax = 0;
        std::vector<float> rq;
        for (auto* s : v) {
            auto c0 = led.at(shift_ms(s->t, -(double)s->dt_ms));
            auto c1 = led.at(s->t);
            double cx = c1.first - c0.first, cy = c1.second - c0.second;
            cc += cx * cx + cy * cy;
            dc += (double)s->sx * cx + (double)s->sy * cy;
            sdir += (double)(sg.defl > 0 ? 1.0 : -1.0) * (sg.axis ? s->sy : s->sx);
            rq.push_back(s->resp);
            spmax = std::max(spmax, s->spread);
            dpmax = std::max(dpmax, (float)std::hypot(s->sx, s->sy));
        }
        o.n = (int)v.size();
        o.cc = cc; o.dc = dc;
        if (o.n < PAD_CAL_SEG_MIN_N) { o.why = "中段样本不足"; continue; }
        if (dpmax > shift_max) {
            o.why = "每帧位移超可靠量程 (游戏太快/相关回卷)"; o.val = dpmax; o.lim = shift_max;
            continue;
        }
        if (spmax > shift_max) {
            o.why = "3×3 块间位移离散超可靠量程 (相关不可信)"; o.val = spmax; o.lim = shift_max;
            continue;
        }
        o.resp = median_of(rq);
        if (o.resp < PAD_CAL_RESP_MIN) {
            o.why = "相位相关响应过低 (背景无纹理/画面不动?)";
            o.val = o.resp; o.lim = PAD_CAL_RESP_MIN; continue;
        }
        // 激励充分性 + 符号: 中段位移须显著正于停顿噪声底 (夹紧 → 位移≈0 → 整级丢弃)
        const double snr = sdir / ((double)r.sigma[sg.axis] * std::sqrt((double)o.n));
        o.val = (float)snr; o.lim = PAD_CAL_MIN_SNR;
        if (snr < (double)PAD_CAL_MIN_SNR) {
            o.why = "中段位移≈0 或符号不符 (俯仰夹紧/屏幕不响应注入)";
            continue;
        }
        if (cc <= 0) { o.why = "账本激发量为零"; continue; }
        o.g = dc / cc;
        if (o.g <= 0) { o.why = "段增益非正"; continue; }
        o.se = (double)r.sigma[sg.axis] / std::sqrt(cc);   // g 的标准误 = σ/√Σ|C|²
        o.ok = true;
    }

    // ---- [3] 逐轴组装与幂律外推 ----
    for (int axis = 0; axis < PAD_CAL_AXES_N; ++axis)
        assemble_axis(axis, so, plan, r);

    // ---- [4] L: 逐停顿两条独立读数 (命令边沿 → 画面真正停下 的时延) ----
    //  (a) 边沿: 首个完全静止的样本出现在 pause_start + L + 帧长 → L = t − pause_start − 帧长。
    //      与增益无关, 但量子化到帧格; 帧格相位逐停顿变化 → 单条估计偏高 ≤一帧, 中位收敛。
    //  (b) 位移和: 停顿内位移之和 = 世界在 [t_first−帧长−L, t_last−L] 的位移; 世界自
    //      pause_start 起静止 → 和 = V·(L − (t_first − pause_start) + 帧长) → 亚帧精度
    //      (位移是精确积分), 噪声按整段停顿平均, 需要前段实测增益 V。
    std::vector<float> La, Lb;
    for (size_t k = 0; k < plan.size(); ++k) {
        const PadExcSeg& sg = plan[k];
        if (!sg.pause || k == 0) continue;
        auto v = pick(hist, sg.t0, seg_end(sg.t1));
        if (v.size() < 2) continue;
        const double frame_i = (double)v[0]->dt_ms;
        int last = -1;                                // 最后一个"在动"的样本
        for (int i = (int)v.size() - 1; i >= 0; --i) {
            double nx = v[(size_t)i]->sx / r.sigma[0], ny = v[(size_t)i]->sy / r.sigma[1];
            if (std::hypot(nx, ny) > (double)PAD_CAL_EDGE_SNR) { last = i; break; }
        }
        if (last >= 0 && last + 1 < (int)v.size()) {  // 无可见边沿 = 前段太弱/夹紧 → 跳过
            double l = elapsed_ms(v[(size_t)(last + 1)]->t, sg.t0) - frame_i;
            if (l >= -frame_i && l <= 3.0 * (double)PAD_CAL_PAUSE_MS) La.push_back((float)l);
        }
        const PadExcSeg& pre = plan[k - 1];
        if (pre.pause) continue;
        const PadCalLevelDiag& pd = r.lv[pre.axis][pre.level];
        if (!pd.valid || pd.px_s <= 0) continue;      // 前段未测出屏速 → 无 V
        const double dir = pre.defl > 0 ? 1.0 : -1.0;
        double ssum = 0;
        for (auto* s : v) ssum += dir * (pre.axis ? s->sy : s->sx);
        double l = ssum / (double)pd.px_s * 1000.0 + elapsed_ms(v[0]->t, sg.t0) - frame_i;
        if (l >= 0.0 && l <= 3.0 * (double)PAD_CAL_PAUSE_MS) Lb.push_back((float)l);
    }
    auto median_and_mad = [](const std::vector<float>& v, float& med, float& mad) {
        med = median_of(v);
        std::vector<float> dev; dev.reserve(v.size());
        for (float x : v) dev.push_back(std::fabs(x - med));
        mad = median_of(dev);
    };
    if (!La.empty()) { r.l_edge_n = (int)La.size(); median_and_mad(La, r.l_edge_ms, r.l_edge_mad); }
    float l_primary = 0.0f;
    if (!Lb.empty()) { median_and_mad(Lb, l_primary, r.l_mad); r.l_n = (int)Lb.size(); }
    else if (!La.empty()) { l_primary = r.l_edge_ms; r.l_mad = r.l_edge_mad; r.l_n = (int)La.size(); }
    else { r.err = "无法实测延迟 (停顿内无边沿)"; return r; }
    // 自校验: 实测 L 超过停顿 P → 停顿留不住命令换向的瞬态, 中段取样窗不再保证落在
    //   段内 → 增益会带上前一段的串扰, 不硬算 (两条读数任一越界即判失败)
    //   边沿读数量子化到帧格 (偏高 ≤ 一帧), 故它的越界判据 = P + 帧长
    if (l_primary > (float)PAD_CAL_PAUSE_MS
        || r.l_edge_ms > (float)PAD_CAL_PAUSE_MS + (float)frame) {
        r.err = "实测延迟超过停顿 P (停顿留不住瞬态)";
        return r;
    }
    r.l_est = std::clamp(l_primary, L_MIN, L_MAX);
    r.ok = (r.nlv[0] > 0 && r.nlv[1] > 0 && r.gain[0] > 0 && r.gain[1] > 0);
    return r;
}

// ========================= 自适应段时长的实测探针 =========================

void pad_calib_update_probe(const std::deque<CalibSample>& hist,
                            const std::vector<PadExcSeg>& plan) {
    if (plan.empty()) return;
    int last_done = -1;                                  // 最后一段已完成 (含停顿)
    for (size_t i = 0; i < plan.size(); ++i)
        if (plan[i].t1 >= plan[i].t0) last_done = (int)i;
    if (last_done < 0) return;
    const int axis = plan[(size_t)last_done].axis, level = plan[(size_t)last_done].level;

    std::vector<float> dts; dts.reserve(hist.size());
    for (const auto& s : hist) dts.push_back(s.dt_ms);
    const double frame = std::max(1.0, (double)median_of(dts));

    const CountsHistory& led = own_motion_ledger();
    double cc = 0, dc = 0;
    for (size_t k = 0; k < plan.size(); ++k) {
        const PadExcSeg& sg = plan[k];
        if (sg.pause || sg.axis != axis || sg.level != level) continue;
        if (sg.t1 < sg.t0) continue;
        for (auto* s : pick(hist, mid_begin(sg.t0), mid_end(sg.t1, frame))) {
            auto c0 = led.at(shift_ms(s->t, -(double)s->dt_ms));
            auto c1 = led.at(s->t);
            double cx = c1.first - c0.first, cy = c1.second - c0.second;
            cc += cx * cx + cy * cy;
            dc += (double)s->sx * cx + (double)s->sy * cy;
        }
    }
    if (cc <= 0) return;
    double g = dc / cc;
    if (g <= 0) return;
    g_pad_probe_d.store((float)PAD_CAL_LEVELS[level]);
    g_pad_probe_px_s.store((float)(g * (double)PAD_CAL_LEVELS[level]
                                 * (double)PAD_AXIS_MAX * 1000.0));
}

// ========================= 标定状态机 =========================

namespace {

enum PadCalPhase {
    PC_IDLE = 0,      // 空闲: L3+R3 长按 / 热参请求计数
    PC_START,         // 起始方块 (纯视觉开始信号, 不采样)
    PC_EXCITE,        // 分轴分级激励 (段间停顿; g_calib_collect 开, 采样中)
    PC_SETTLE,        // 激励后静置 (采样窗收尾)
    PC_WAIT,          // 等 ai_thread 拟合 (g_calib_done; 摇杆静置)
    PC_END_OK,        // 收尾: 成功点头
    PC_END_FAIL,      // 收尾: 失败摇头
};

struct PadCalState {
    PadCalPhase phase = PC_IDLE;
    int hold = 0;                        // L3+R3 长按计数 (拍)
    int wt = 0;                          // 等计算超时计数 (拍)
    const CalibSeg* seq = nullptr;
    int slen = 0, si = 0, st = 0;
    int pi = 0, pt = 0;                  // 计划段游标 / 段内拍
    int level_ticks = 0;                 // 本级激励段时长 (自适应; 级内两段同长)
    int seg_ticks = 0;                   // 当前段的播放时长 (激励段 = level_ticks, 停顿 = P)
    int cur_axis = -1, cur_level = -1;
};

template <size_t N>
void enter(PadCalState& s, const CalibSeg (&seq)[N], PadCalPhase ph) {
    s.seq = seq; s.slen = (int)N; s.si = s.st = 0; s.phase = ph;
}

// 本级激励段时长: 每级开头按已测屏速重定 (低挡位先测完 → 已知增益量级)
void enter_level(PadCalState& s, int cam_fps, const PadPlanSeg& sg) {
    float v = 0, dm = 0;
    if (s.cur_axis != sg.axis) {                 // 换轴 = 换探针 (两轴灵敏度不同)
        g_pad_probe_px_s.store(0.0f); g_pad_probe_d.store(0.0f);
    } else {
        v = g_pad_probe_px_s.load(); dm = g_pad_probe_d.load();
    }
    s.level_ticks = pad_cal_seg_ticks(v, dm, sg.d, cam_fps);
    s.cur_axis = sg.axis; s.cur_level = sg.level;
    printf("[标定] 激励 %s 轴 %d%%: 段长 %dms + 停顿 %dms%s (行程目标 %.0fpx)\n",
           sg.axis ? "Y" : "X", (int)(sg.d * 100.0f + 0.5f),
           (int)(s.level_ticks * 1000 / DEFAULT_FREQ), PAD_CAL_PAUSE_MS,
           v > 0 ? " (按已测屏速)" : " (探针级)", (double)PAD_CAL_TRAVEL_PX);
    fflush(stdout);
}

} // namespace

PadCalibStep pad_calib_step(uint16_t btns, int cam_fps) {
    static PadCalState s;
    const bool req_raw = g_padcalib_request.exchange(false);
    bool req = req_raw;
    PadCalibStep out{false, 0, 0};
    auto now = std::chrono::steady_clock::now();

    if (req && s.phase != PC_IDLE) {           // 请求在标定进行中到达: 一次消费即清
        std::cout << "[标定] 忽略请求: 标定进行中\n";
        req = false;
    }

    if (!g_aim_enabled.load()) {
        // 接管关闭 = 纯透传: 激励是程序注入的移动, 与透传互斥 — 进行中的标定
        //   复位, 触发不可达 (与 hid 标定分支同一纪律)
        if (s.phase != PC_IDLE) {
            s.phase = PC_IDLE; g_calib_collect = false;
            s.seq = nullptr; s.slen = s.si = s.st = 0;
            g_pad_exc_plan.clear();
            std::cout << "[标定] 中断: 接管关闭 (纯透传)\n";
        }
        s.hold = 0;
        if (req) std::cout << "[标定] 忽略请求: 接管关闭 (纯透传)\n";
        return out;
    }

    if (s.phase == PC_IDLE) {
        const bool both = (btns & (PADBTN_L3 | PADBTN_R3)) == (PADBTN_L3 | PADBTN_R3);
        if (both) ++s.hold; else s.hold = 0;
        if (req || s.hold >= PAD_CAL_TRIGGER_TICKS) {
            s.hold = 0;
            printf("[标定] 触发: 分轴分级激励 (X/Y 各 %d 级 × %d 段 + 段间停顿 %dms, "
                   "全程 ≤%.1fs)\n", PAD_CAL_LEVELS_N, PAD_CAL_SEGS_PER_LEVEL,
                   PAD_CAL_PAUSE_MS, (PAD_CAL_PLAN_MS + 2500) / 1000.0);
            fflush(stdout);
            s.cur_axis = s.cur_level = -1;
            enter(s, PAD_CAL_START_SEQ, PC_START);
            out.active = true;        // 触发拍不播激励 (激励自下一拍起, 与 hid 同形)
            return out;
        }
    }

    if (s.phase == PC_WAIT) {
        out.active = true;                       // 等计算期摇杆静置 (dx=dy=0)
        int done = g_calib_done.load();
        if (done != 0 || ++s.wt > CALIB_WAIT_TIMEOUT)
            enter(s, done == 1 ? PAD_CAL_END_OK_SEQ : PAD_CAL_END_FAIL_SEQ,
                  done == 1 ? PC_END_OK : PC_END_FAIL);
        return out;
    }

    if (s.phase == PC_EXCITE) {
        const auto& plan = pad_cal_plan();
        if (s.pi >= (int)plan.size()) { enter(s, PAD_CAL_SETTLE_SEQ, PC_SETTLE); }
        else {
            const PadPlanSeg& sg = plan[(size_t)s.pi];
            if (s.pt == 0) {   // 段首: 定本段时长 (激励段取本级段长, 级内两段同长)
                if (sg.pause) s.seg_ticks = ms_to_ticks(PAD_CAL_PAUSE_MS);
                else {
                    if (s.cur_level != sg.level || s.cur_axis != sg.axis)
                        enter_level(s, cam_fps, sg);
                    s.seg_ticks = s.level_ticks;
                }
                g_pad_exc_plan.begin_seg(s.pi, now);
            }
            out.active = true;
            if (!sg.pause) { if (sg.axis == 0) out.dx = sg.defl; else out.dy = sg.defl; }
            if (++s.pt >= s.seg_ticks) {
                g_pad_exc_plan.end_seg(s.pi, now);
                s.pt = 0; ++s.pi;
            }
            return out;
        }
    }

    if (s.phase != PC_IDLE) {
        out.active = true;
        if (s.si < s.slen) {
            out.dx = (int16_t)s.seq[s.si].dx; out.dy = (int16_t)s.seq[s.si].dy;
            if (++s.st >= s.seq[s.si].ticks) { s.st = 0; ++s.si; }
        }
        if (s.si >= s.slen) {
            if (s.phase == PC_START) {
                g_pad_exc_plan.reset();
                s.cur_axis = s.cur_level = -1;
                g_pad_probe_px_s.store(0.0f); g_pad_probe_d.store(0.0f);
                s.pi = s.pt = 0;
                g_calib_collect = true; s.phase = PC_EXCITE;
            } else if (s.phase == PC_SETTLE) {
                g_calib_collect = false;
                g_calib_request = true;
                s.wt = 0; s.phase = PC_WAIT;
            } else {
                g_pad_exc_plan.clear();
                s.phase = PC_IDLE; s.seq = nullptr; s.slen = s.si = s.st = 0;
            }
        }
    }
    return out;
}
