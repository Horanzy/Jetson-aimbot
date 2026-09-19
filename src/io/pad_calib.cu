// ============================================================================
//  pad_calib.cu — pad_calib.h 的实现:
//    [1] 激励计划表 (先 X 后 Y, 每轴逐级, 每级 ± 各一段, 每段后接零偏转停顿) 与
//        段窗口表;
//    [2] 自适应段时长 (等行程: 行程目标 / 已测屏速, 夹在中段统计下限与段长上限
//        之间) 与上一级实测行程的闭环校正;
//    [3] 拟合: 停顿静止窗估噪声底 σ → 停顿边沿实测 L → 逐段中段取样求增益
//        (无延迟对齐) → 段/级有效性判定 → 逐级池化 → 幂律外推 (或单级线性回退);
//    [4] 采样状态 (逐帧): 探针 (宽松判据 + 量级钳制, 给下一级定段长) 与级采样状态
//        (中段样本数最小值 → 重播裁决);
//    [5] 标定状态机 (相位与 hid 标定同形, 状态自成一态): 触发 → 起始方块 → 分轴
//        分级激励 (段长自适应 + 取样不足重播阶梯) → 静置 → 请求计算 → 等回执
//        (成功/失败/整轮重跑) → 点头/摇头; 采样/计算/回写经
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
std::atomic<float> g_pad_probe_travel_px{0.0f};
std::atomic<int>   g_pad_mid_n[PAD_CAL_AXES_N][PAD_CAL_LEVELS_N];

// ========================= 计划表与段窗口 =========================

// 第 (轴,级) 的首个激励段在计划表里的序号 (重播回退的落点)
static int pad_first_seg_index(int axis, int level) {
    return (axis * PAD_CAL_LEVELS_N + level) * PAD_CAL_SEGS_PER_LEVEL * 2;
}

const std::vector<PadPlanSeg>& pad_cal_plan() {
    static const std::vector<PadPlanSeg> plan = [] {
        std::vector<PadPlanSeg> p;
        // 对称段序 (见 io/pad_calib.h): +d, −d, −d, +d — 行程以该级起点为中心 ±A,
        //   每方向 2 段 (方向成对 → 一致性判据有 2 组样本)
        static const int sgn[PAD_CAL_SEGS_PER_LEVEL] = {1, -1, -1, 1};
        for (int axis = 0; axis < PAD_CAL_AXES_N; ++axis)
            for (int li = 0; li < PAD_CAL_LEVELS_N; ++li)
                for (int k = 0; k < PAD_CAL_SEGS_PER_LEVEL; ++k) {
                    const int16_t d = pad_level_defl(PAD_CAL_LEVELS[li]);
                    const int16_t ex = (int16_t)(sgn[k] * d);
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
    if (i >= 0 && i < (int)segs_.size()) {
        segs_[i].begun = true; segs_[i].t0 = t; segs_[i].t1 = t; }
}
void PadExcPlan::end_seg(int i, std::chrono::steady_clock::time_point t) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (i >= 0 && i < (int)segs_.size()) segs_[i].t1 = t;
}
void PadExcPlan::restart_level(int axis, int level) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto& e : segs_)
        if (e.axis == axis && e.level == level) {
            e.begun = false;
            e.t0 = e.t1 = std::chrono::steady_clock::time_point{};
        }
}
void PadExcPlan::clear() {
    std::lock_guard<std::mutex> lk(mtx_);
    segs_.clear();
}
std::vector<PadExcSeg> PadExcPlan::snapshot() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return segs_;
}
int PadExcPlan::last_begun() const {
    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = (int)segs_.size() - 1; i >= 0; --i) if (segs_[(size_t)i].begun) return i;
    return -1;
}
PadExcPlan g_pad_exc_plan;

// ========================= 自适应段时长 (等行程) =========================

int pad_cal_seg_ticks(float v_meas_px_s, float d_meas, float d_next, int floor_ms) {
    int ms = floor_ms;                            // 探针级 (尚无实测) = 统计下限
    if (v_meas_px_s > 0.0f && d_meas > 0.0f && d_next > 0.0f) {
        double v_next = (double)v_meas_px_s
                      * std::pow((double)d_next / (double)d_meas, (double)PAD_CAL_CURVE_P_MAX);
        if (v_next > 1e-6) ms = (int)((double)PAD_CAL_TRAVEL_PX / v_next * 1000.0 + 0.5);
    }
    return ms_to_ticks(std::clamp(ms, floor_ms, PAD_CAL_SEG_MS_MAX));
}

// ========================= 拟合 =========================

namespace {

// 拍 ↔ 墙钟换算 (拟合窗一律按拍算; 见 seg_end/mid_begin 的说明)
inline std::chrono::steady_clock::time_point one_tick(
        const std::chrono::steady_clock::time_point& t) {
    return shift_ms(t, (double)TICK_MS);                       // 一拍 = TICK_MS
}
inline std::chrono::steady_clock::time_point shift_ticks(
        const std::chrono::steady_clock::time_point& t, int ticks) {
    return shift_ms(t, (double)ticks * (double)TICK_MS);
}
// 停顿的静止参考窗起点 (×P): 保证 L ≤ 100ms (设计保证) 时窗内样本已完全静止
inline std::chrono::steady_clock::time_point pause_tail(
        const std::chrono::steady_clock::time_point& t0) {
    return shift_ticks(t0, (int)((double)PAD_CAL_PAUSE_TICKS * (double)PAD_CAL_PAUSE_TAIL));
}
// 段的命令区间 = [t0, t1 + 一拍]; 中段窗 = [段起+P, 命令末−一帧]
//   P 与"一拍"都以**拍数常量**表达 (PAD_CAL_PAUSE_TICKS / TICK_MS), 换算到墙钟后
//   与 ms 等价 (DEFAULT_FREQ = 1000): 激励与停顿的长度本来就是拍数, 用拍表达使
//   窗与激励边界同源; 换拍率时两者同比缩放, 不会错位。
inline std::chrono::steady_clock::time_point seg_end(
        const std::chrono::steady_clock::time_point& t1) {
    return one_tick(t1);
}
inline std::chrono::steady_clock::time_point mid_begin(
        const std::chrono::steady_clock::time_point& t0) {
    return shift_ticks(t0, PAD_CAL_PAUSE_TICKS);
}
inline std::chrono::steady_clock::time_point mid_end(
        const std::chrono::steady_clock::time_point& t1, double frame) {
    return shift_ms(t1, (double)TICK_MS - frame);              // frame 是实测帧长 (ms)
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

// 帧周期 (ms): 样本 dt 的中位 — 拟合与探针共用 (都不依赖采集参数)
double frame_of(const std::deque<CalibSample>& hist) {
    std::vector<float> dts; dts.reserve(hist.size());
    for (const auto& s : hist) dts.push_back(s.dt_ms);
    return std::max(1.0, (double)median_of(dts));
}

// 某轴的噪声底 σ̂: 该轴全部停顿的静止参考窗 (1.4826×中位|位移|, 稳健尺度)。
//   样本数不足 → 0 (调用方据此判"证据不足")。
float sigma_of(const std::deque<CalibSample>& hist, const std::vector<PadExcSeg>& plan,
               int axis) {
    std::vector<float> nz;
    for (const auto& sg : plan) {
        if (!sg.pause || sg.axis != axis || !sg.begun) continue;
        for (auto* s : pick(hist, pause_tail(sg.t0), seg_end(sg.t1)))
            nz.push_back(sg.axis ? s->sy : s->sx);
    }
    if ((int)nz.size() < PAD_CAL_SIGMA_MIN_N) return 0.0f;
    return 1.4826f * median_abs(nz);
}

// 逐段的中段统计结果
struct SegOut {
    bool ok = false;
    const char* why = "";
    bool samples = false;                     // 原因 = 中段样本不足
    bool no_tex = false;                      // 原因 = 相关响应过低 (该方向无纹理)
    int  dir = 1;                             // 激励方向 (+1/−1)
    double g = 0, se = 0, cc = 0, dc = 0;
    int n = 0;
    float resp = 0, val = 0, lim = 0;
};

// 单级判定与池化: 通过则 dg.valid=true、g = 池化增益 (px per 偏转·ms) 并返回 true。
//   方向分组 (每方向 2 段) 后:
//     两方向可用 → 方向组一致性判据 (|g₊−g₋| ≤ PAD_CAL_SEG_Z·√(SE₊²+SE₋²));
//     只有一方向可用, 另一方向**仅因响应门失败** (无纹理) → 采用可用方向并标注
//       (无纹理只污染它自己那个方向); 另一方向因位移门 (夹紧) 或其他判据失败 →
//       整级丢弃 (见文件头: 判据没有放宽, 只是把两种物理含义分开)。
bool level_gain(const std::vector<const SegOut*>& sv, float sigma,
                PadCalLevelDiag& dg, double& g) {
    dg.seg_all = (int)sv.size();
    for (const SegOut* o : sv) {                      // 首个失效段的原因 (日志)
        if (o->ok) continue;
        dg.why = o->why; dg.why_samples = o->samples;
        dg.val = o->val; dg.lim = o->lim;
        break;
    }
    std::vector<const SegOut*> up, dn;
    for (const SegOut* o : sv) (o->dir > 0 ? up : dn).push_back(o);
    auto usable = [](const std::vector<const SegOut*>& v) {
        for (const SegOut* o : v) if (!o->ok) return false;
        return !v.empty();
    };
    auto only_no_tex = [](const std::vector<const SegOut*>& v) {
        bool any = false;
        for (const SegOut* o : v) {
            if (o->ok) continue;
            if (!o->no_tex) return false;             // 夹紧/量程/样本数 → 不可用
            any = true;
        }
        return any;
    };
    const bool up_ok = usable(up), dn_ok = usable(dn);
    if (!up_ok && !dn_ok) return false;
    if (!up_ok && !only_no_tex(up)) return false;
    if (!dn_ok && !only_no_tex(dn)) return false;
    auto pool = [](const std::vector<const SegOut*>& v, double& cc, double& dc) {
        cc = dc = 0;
        for (const SegOut* o : v) { cc += o->cc; dc += o->dc; }
    };
    double cc_up = 0, dc_up = 0, cc_dn = 0, dc_dn = 0;
    if (up_ok) pool(up, cc_up, dc_up);
    if (dn_ok) pool(dn, cc_dn, dc_dn);
    if (up_ok && dn_ok) {                             // 方向组一致性 (方向不该改响应)
        const double se = (double)sigma * std::sqrt(1.0 / cc_up + 1.0 / cc_dn);
        const double ga = dc_up / cc_up, gb = dc_dn / cc_dn;
        if (std::fabs(ga - gb) > (double)PAD_CAL_SEG_Z * se) {
            dg.why = "两方向池化增益不一致 (方向/量程异常)";
            dg.val = (float)std::fabs(ga - gb); dg.lim = (float)((double)PAD_CAL_SEG_Z * se);
            return false;
        }
    } else {
        dg.note = up_ok ? "仅 +向测量 (另一向相关响应过低=无纹理)"
                        : "仅 −向测量 (另一向相关响应过低=无纹理)";
    }
    dg.seg_ok = 0;
    for (const SegOut* o : sv) if (o->ok) ++dg.seg_ok;
    dg.resp = sv[0]->resp;
    for (const SegOut* o : sv) if (o->ok) dg.resp = std::min(dg.resp, o->resp);
    const double cc = cc_up + cc_dn, dc = dc_up + dc_dn;
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
        if (level_gain(sv, r.sigma[axis], dg, g)) {
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
        if (!s.begun) { r.err = "段窗口不完整 (激励未播完)"; return r; }

    const double frame = frame_of(hist);

    // ---- [1] 噪声底 σ: 停顿的静止参考窗 (尾段, 保证 L ≤ 100ms 时已完全静止) ----
    for (int a = 0; a < PAD_CAL_AXES_N; ++a) {
        // σ̂ = 1.4826×中位|·| (稳健尺度); 中位估计的标准误差 ≈ 1.25/√n, 要 ≤30% 需 n ≥ 18
        r.sigma[a] = sigma_of(hist, plan, a);
        if (r.sigma[a] <= 0.0f) { r.err = "静止参考样本不足"; return r; }
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
        o.dir = sg.defl > 0 ? 1 : -1;
        o.cc = cc; o.dc = dc;
        o.val = (float)o.n; o.lim = (float)PAD_CAL_SEG_MIN_N;
        if (o.n < PAD_CAL_SEG_MIN_N) {
            o.why = "中段样本不足"; o.samples = true; continue; }
        o.val = o.lim = 0;
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
            o.why = "相位相关响应过低 (该方向背景无纹理)";
            o.no_tex = true;
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

// ========================= 采样状态 (探针 + 级采样状态) =========================

void pad_calib_update_probe(const std::deque<CalibSample>& hist,
                            const std::vector<PadExcSeg>& plan) {
    if (plan.empty() || hist.empty()) return;
    int last = -1;
    for (size_t i = 0; i < plan.size(); ++i) if (plan[i].begun) last = (int)i;
    if (last < 0) return;
    const int axis = plan[(size_t)last].axis, level = plan[(size_t)last].level;
    const auto now = hist.back().t;
    const double frame = frame_of(hist);
    const float sigma = sigma_of(hist, plan, axis);

    const CountsHistory& led = own_motion_ledger();
    double cc = 0, dc = 0, sdir = 0;
    int n = 0, mid_min = PAD_CAL_MID_N_NONE;
    double travel_sum = 0; int travel_n = 0;
    for (size_t k = 0; k < plan.size(); ++k) {
        const PadExcSeg& sg = plan[k];
        if (sg.pause || !sg.begun || sg.axis != axis || sg.level != level) continue;
        const double dir = sg.defl > 0 ? 1.0 : -1.0;
        // 中段窗: 增益 (池化最小二乘) 与样本数
        int cnt = 0;
        for (auto* s : pick(hist, mid_begin(sg.t0), mid_end(sg.t1, frame))) {
            auto c0 = led.at(shift_ms(s->t, -(double)s->dt_ms));
            auto c1 = led.at(s->t);
            double cx = c1.first - c0.first, cy = c1.second - c0.second;
            cc += cx * cx + cy * cy;
            dc += (double)s->sx * cx + (double)s->sy * cy;
            sdir += dir * (sg.axis ? s->sy : s->sx);
            ++cnt;
        }
        n += cnt;
        // 级采样状态只数**已完成**的中段窗 (窗未走完的段还在累积, 数它会把最小值
        //   钉在段首的少数样本上): 状态机据此判"取样不足 → 重播"
        if (now >= mid_end(sg.t1, frame)) mid_min = std::min(mid_min, cnt);
        // 实测行程 (闭环校正的输入): 整段命令区间 [t0, t1+一拍] 的位移积分 —
        //   该量含观测模型的 L 滞后, 正是"准星真正走过的距离"里被中段窗略去的那部分
        double tr = 0;
        for (auto* s : pick(hist, sg.t0, seg_end(sg.t1))) tr += dir * (sg.axis ? s->sy : s->sx);
        travel_sum += std::fabs(tr); ++travel_n;
    }
    g_pad_mid_n[axis][level].store(mid_min);
    if (travel_n > 0) g_pad_probe_travel_px.store((float)(travel_sum / travel_n));

    // 探针 (宽松判据 + 量级钳制, 见文件头): 只给下一级定段长, 不判级
    if (sigma <= 0.0f || n <= 0 || cc <= 0.0) return;
    const double g = dc / cc;
    if (!(g > 0.0)) return;
    if (sdir / ((double)sigma * std::sqrt((double)n)) < (double)PAD_CAL_PROBE_SNR) return;
    const double px_s = g * (double)PAD_CAL_LEVELS[level] * (double)PAD_AXIS_MAX * 1000.0;
    if (!(px_s > 0.0) || px_s > (double)PAD_GAIN_MAX) return;   // 量级钳制
    g_pad_probe_d.store((float)PAD_CAL_LEVELS[level]);
    g_pad_probe_px_s.store((float)px_s);
}

// ========================= 死区/响应曲线诊断 =========================

PadCalDeadzone pad_cal_deadzone(const PadCalibResult& r, int axis) {
    PadCalDeadzone z;
    int hi = -1;                                  // 锚点 = 最高有效挡位
    for (int li = PAD_CAL_LEVELS_N - 1; li >= 0; --li)
        if (r.lv[axis][li].valid) { hi = li; break; }
    if (hi < 0) return z;
    z.hi_li = hi;
    for (int li = 0; li < hi; ++li) {
        if (!r.lv[axis][li].valid) continue;
        const float v_line = r.lv[axis][hi].px_s
                           * (r.lv[axis][li].d / r.lv[axis][hi].d);   // 过锚点的 p=1 直线
        if (r.lv[axis][li].px_s < PAD_CAL_DEADZONE_FRAC * v_line) {
            z.hit = true; z.lo_li = li; z.v_lo = r.lv[axis][li].px_s; z.v_line = v_line;
            break;
        }
    }
    return z;
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
    int level_ms = 0;                    // 本级本次尝试的段长 (ms; 级内两段同长)
    int seg_ticks = 0;                   // 当前段的播放拍数
    int tries = 1;                       // 本级已播尝试数 (重播阶梯)
    int round = 0;                       // 0 = 首轮, 1 = 整轮重跑
    int cur_axis = -1, cur_level = -1;
    bool  unclamped = false;             // 本级段长未被下限/上限夹住
    float prev_travel = 0;               // 上一级的实测行程 (px; 0 = 无)
    bool  prev_ok = false;               // 上一级取样充足且未被夹住 (闭环校正可用)
    std::chrono::steady_clock::time_point t_exc0{};   // 首轮激励起点 (流程预算基准)
};

template <size_t N>
void enter(PadCalState& s, const CalibSeg (&seq)[N], PadCalPhase ph) {
    s.seq = seq; s.slen = (int)N; s.si = s.st = 0; s.phase = ph;
}

// 一轮激励的起点: 计划表重建, 探针/级采样状态清零 (第一级无实测 → 取下限)
void start_round(PadCalState& s, int round) {
    s.round = round;
    s.pi = s.pt = 0;
    s.cur_axis = s.cur_level = -1;
    s.tries = 1;
    s.prev_travel = 0; s.prev_ok = false;
    g_pad_probe_px_s.store(0.0f); g_pad_probe_d.store(0.0f);
    g_pad_probe_travel_px.store(0.0f);
    for (int a = 0; a < PAD_CAL_AXES_N; ++a)
        for (int l = 0; l < PAD_CAL_LEVELS_N; ++l)
            g_pad_mid_n[a][l].store(PAD_CAL_MID_N_NONE);
    g_pad_exc_plan.reset();
    g_calib_collect = true;
    s.phase = PC_EXCITE;
}

// 本级段长: 等行程 (下一级屏速由探针外推) + 上一级实测行程的闭环校正 + 夹子。
//   校正只在上一级"取样充足且未被夹住"时施加 — 夹住的级其行程由夹子决定, 比值里
//   不含增益信息; 比值本身不需单独钳制, T 被 [下限, 上限] 夹住, 极端比值退化为夹子。
void enter_level(PadCalState& s, int cam_fps, const PadPlanSeg& sg) {
    float v = 0, dm = 0;
    if (s.cur_axis != sg.axis) {                 // 换轴 = 换探针 (两轴灵敏度不同)
        g_pad_probe_px_s.store(0.0f); g_pad_probe_d.store(0.0f);
    } else {
        v = g_pad_probe_px_s.load(); dm = g_pad_probe_d.load();
    }
    const int lo = pad_cal_seg_floor_ms(cam_fps, s.round > 0);
    int ms = pad_cal_seg_ticks(v, dm, sg.d, lo);
    const double ms_nom = ms;                    // 未校正的等行程段长 (日志用)
    double fac = 1.0;
    if (v > 0.0f && s.prev_ok && s.prev_travel > 0.0f) {
        fac = (double)s.prev_travel / (double)PAD_CAL_TRAVEL_PX;
        ms = (int)((double)ms / fac + 0.5);
    }
    s.level_ms = std::clamp(ms, lo, PAD_CAL_SEG_MS_MAX);
    s.unclamped = (s.level_ms > lo && s.level_ms < PAD_CAL_SEG_MS_MAX);
    s.tries = 1; s.cur_axis = sg.axis; s.cur_level = sg.level;
    // 预期行程: 用与定段长同一条外推 (V̂(d) = v·(d/dm)^P_MAX), 故未夹住时 = 目标,
    //   撞下限 → 超目标, 撞上限 → 低于目标 — 三者都在日志里标出 (现场判断依据)
    double trav_pred = 0.0;
    if (v > 0.0f && dm > 0.0f)
        trav_pred = (double)v * std::pow((double)sg.d / (double)dm, (double)PAD_CAL_CURVE_P_MAX)
                  * (double)s.level_ms / 1000.0;
    const char* lim = s.level_ms <= lo ? "下限(样本数) → 行程超目标"
                    : s.level_ms >= PAD_CAL_SEG_MS_MAX ? "上限 → 行程低于目标"
                    : "按行程目标";
    printf("[标定] 激励 %s 轴 %d%% (第%d次): 段长 %dms + 停顿 %dms | 目标 %.0fpx / 预期 %.0fpx"
           " [%s]%s\n",
           sg.axis ? "Y" : "X", (int)(sg.d * 100.0f + 0.5f), s.tries,
           s.level_ms, PAD_CAL_PAUSE_MS, (double)PAD_CAL_TRAVEL_PX, trav_pred, lim,
           v > 0.0f ? "" : " (探针级: 无实测, 取统计下限)");
    if (fac != 1.0)
        printf("[标定]   ↳ 上一级实测行程 %.0fpx / 目标 %.0fpx → 段长 %.0f→%dms (闭环校正)\n",
               (double)s.prev_travel, (double)PAD_CAL_TRAVEL_PX, ms_nom, s.level_ms);
    fflush(stdout);
}

// 级边界 (下一级的首段开播前): 记下本级的实测行程 (供下一级闭环校正) +
//   裁决"取样不足 → 重播"。重播条件: 该级**已完成的激励段**里中段样本数最小值
//   < PAD_CAL_SEG_MIN_N (即拟合侧唯一的丢级原因), 且段长未到上限 (再播同长无收益)、
//   流程预算仍放得下本次尝试。
void level_boundary(PadCalState& s, std::chrono::steady_clock::time_point now) {
    const int axis = s.cur_axis, level = s.cur_level;
    const int mid = g_pad_mid_n[axis][level].load();
    const float trav = g_pad_probe_travel_px.load();
    s.prev_travel = trav;
    s.prev_ok = (mid >= PAD_CAL_SEG_MIN_N) && s.unclamped && trav > 0.0f;
    if (mid >= PAD_CAL_SEG_MIN_N) return;
    if (s.level_ms >= PAD_CAL_SEG_MS_MAX) return;
    int nms = s.level_ms * 2;
    if (nms > PAD_CAL_SEG_MS_MAX) nms = PAD_CAL_SEG_MS_MAX;
    const double nom = 2.0 * (nms + PAD_CAL_PAUSE_MS);
    if (elapsed_ms(now, s.t_exc0) + nom > (double)PAD_CAL_BUDGET_MS) {
        printf("[标定] 不重播 %s 轴 %d%%: 流程预算不足 (已用 %.1fs + 本次 %.1fs > %dms)\n",
               axis ? "Y" : "X", (int)(PAD_CAL_LEVELS[level] * 100.0f + 0.5f),
               elapsed_ms(now, s.t_exc0) / 1000.0, nom / 1000.0, PAD_CAL_BUDGET_MS);
        fflush(stdout);
        return;
    }
    printf("[标定] 重播 %s 轴 %d%%: 中段样本不足 (%d < %d, 第 %d 次尝试) → 段长 %d→%dms\n",
           axis ? "Y" : "X", (int)(PAD_CAL_LEVELS[level] * 100.0f + 0.5f),
           mid, PAD_CAL_SEG_MIN_N, s.tries + 1, s.level_ms, nms);
    fflush(stdout);
    s.pi = pad_first_seg_index(axis, level);
    s.pt = 0;
    s.level_ms = nms;
    ++s.tries;
    g_pad_exc_plan.restart_level(axis, level);        // 该级旧窗口作废 (重播重建)
    s.unclamped = (s.level_ms < PAD_CAL_SEG_MS_MAX);
    g_pad_mid_n[axis][level].store(PAD_CAL_MID_N_NONE);   // 旧尝试的样本数作废
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
                   "取样不足自动加长重播; 全程 ≤%.1fs)\n", PAD_CAL_LEVELS_N,
                   PAD_CAL_SEGS_PER_LEVEL, PAD_CAL_PAUSE_MS,
                   (PAD_CAL_BUDGET_MS + 2500) / 1000.0);
            fflush(stdout);
            s.t_exc0 = now;
            enter(s, PAD_CAL_START_SEQ, PC_START);
            out.active = true;        // 触发拍不播激励 (激励自下一拍起, 与 hid 同形)
            return out;
        }
    }

    if (s.phase == PC_WAIT) {
        out.active = true;                       // 等计算期摇杆静置 (dx=dy=0)
        int done = g_calib_done.load();
        if (done == 3 && s.round == 0) {
            // 整轮重跑 (仅一次): 首轮"有效级 <2"且丢级原因只有中段样本不足 —
            //   阶梯够不到的瞬态 (采样一度中断横跨整轮) 只能靠重跑重新捕获。
            //   下限抬高一档 (pad_cal_seg_floor_ms(rerun)), 预算按本轮名义时长判。
            const int f2 = pad_cal_seg_floor_ms(cam_fps, true);
            const double nom = (double)PAD_CAL_EXCITE_N * (f2 + PAD_CAL_PAUSE_MS);
            if (elapsed_ms(now, s.t_exc0) + nom <= (double)PAD_CAL_BUDGET_MS) {
                printf("[标定] 整轮重跑: 首轮仅因中段样本不足而有效级 <2 → 段长下限 "
                       "%d→%dms (第 2 轮, 仅此一次)\n",
                       pad_cal_seg_ms_min(cam_fps), f2);
                fflush(stdout);
                g_calib_done.store(0);
                start_round(s, 1);
                return out;
            }
            printf("[标定] 不重跑: 流程预算不足 (已用 %.1fs + 本轮 %.1fs > %dms)\n",
                   elapsed_ms(now, s.t_exc0) / 1000.0, nom / 1000.0, PAD_CAL_BUDGET_MS);
            fflush(stdout);
        }
        if (done != 0 || ++s.wt > CALIB_WAIT_TIMEOUT)
            enter(s, done == 1 ? PAD_CAL_END_OK_SEQ : PAD_CAL_END_FAIL_SEQ,
                  done == 1 ? PC_END_OK : PC_END_FAIL);
        return out;
    }

    if (s.phase == PC_EXCITE) {
        const auto& plan = pad_cal_plan();
        if (s.pi >= (int)plan.size()) { enter(s, PAD_CAL_SETTLE_SEQ, PC_SETTLE); }
        else {
            if (s.pt == 0) {   // 段首: 先过级边界 (行程反馈 + 重播裁决), 再定本段段长
                const PadPlanSeg& nx = plan[(size_t)s.pi];
                if (!nx.pause && s.cur_level >= 0
                    && (nx.axis != s.cur_axis || nx.level != s.cur_level))
                    level_boundary(s, now);
                const PadPlanSeg& sg = plan[(size_t)s.pi];   // 重播回退后重新取
                if (sg.pause) s.seg_ticks = ms_to_ticks(PAD_CAL_PAUSE_MS);
                else {
                    if (s.cur_level != sg.level || s.cur_axis != sg.axis)
                        enter_level(s, cam_fps, sg);
                    s.seg_ticks = ms_to_ticks(s.level_ms);
                }
                g_pad_exc_plan.begin_seg(s.pi, now);
            }
            const PadPlanSeg& sg = plan[(size_t)s.pi];
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
                start_round(s, 0);
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
