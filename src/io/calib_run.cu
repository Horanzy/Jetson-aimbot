// ============================================================================
//  calib_run.cu — calib_run.h 的实现:
//    [1] 激励计划 (每轴 2 对, 符号交替; 振幅由屏幕速度目标导出 — 算式见头文件);
//    [2] 段窗口表与采样端在线累计行程 (cal_note_sample);
//    [3] 拟合 cal_fit: 停顿静止窗估 σ → 逐段 (量程/离散/纹理/测量窗门 + 累计曲线的
//        响应检出与目标穿越) → 三读数 (尾迹/停止沿/起始沿) → 中位 + MAD + 一致性门;
//    [4] 诊断行与回写; 状态机 cal_step (该模式的 1kHz 拍驱动, 状态按模式分开)。
// ============================================================================

#include "io/calib_run.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>

#include "core/control.h"     // BOTH_SIDE_KEYS (hid 触发键位)
#include "core/state.h"
#include "io/pad_input.h"     // PADBTN_L3/R3 (pad 触发), PAD_AXIS_MAX (偏转单位)

std::atomic<float> g_cal_live_travel{0.0f};
std::atomic<int>   g_cal_live_slot{-1};

// ========================= [1] 激励计划 =========================

namespace {

// 设计激励屏速 (px/ms) = 行程目标 / 段跨地板 (三条约束的算式见 calib_run.h)
float seg_design_v(int axis) {
    return CAL_TRAVEL_PX[axis] / (float)CAL_SEG_MIN_MS;
}
// hid 注入量 (counts/ms) = v* / 该轴有效灵敏度 (spd 口径)
float hid_amp(int axis) {
    const float s = s_hid_now(ads_down(), axis);
    return seg_design_v(axis) / std::max(1e-6f, s);
}
// pad 注入量 (满偏比例) = v*·1000 / 该轴有效满偏屏速, 夹在 [死区地板, 满偏]
float pad_amp(int axis) {
    const float g = gain_pad_eff(spd_axis(ads_down(), axis));
    const float d = seg_design_v(axis) * 1000.0f / std::max(1.0f, g);
    return std::clamp(d, CAL_PAD_DEFL_MIN, 1.0f);
}

} // namespace

std::vector<CalPlanSeg> cal_plan(CalMode m) {
    std::vector<CalPlanSeg> p;
    const int axes = (m == CAL_MODE_PAD) ? 1 : 2;       // pad 只扫水平轴 (Y 被俯仰/辅助瞄准污染)
    const int nseg = 2 * cal_pairs(m);                  // 每轴段数 (符号交替 [+,−,+,−])
    p.reserve(axes * nseg * 2);
    for (int a = 0; a < axes; ++a) {
        const float amp = (m == CAL_MODE_PAD) ? pad_amp(a) : hid_amp(a);
        for (int k = 0; k < nseg; ++k) {
            CalPlanSeg e; e.axis = a; e.dir = (k % 2 == 0) ? +1 : -1; e.pause = false; e.amp = amp;
            p.push_back(e);
            CalPlanSeg z; z.axis = a; z.dir = 0; z.pause = true; z.amp = 0.0f;
            p.push_back(z);
        }
    }
    return p;
}

int cal_plan_worst_ms(CalMode m) {
    // 全程上限 = 触发长按窗 (5s) + 每轴一次不响应中止 (CAL_ABORT_N 段超时后跳过其余段)
    //   + 动画/静置/回执窗 (起始十字 1460 + 收尾静置 300 + 回执超时 2000 + 点头/摇头 720)
    const int axes = (m == CAL_MODE_PAD) ? 1 : 2;
    const int pauses = 2 * cal_pairs(m);        // 每轴段后停顿数 (不响应时照旧播放 — σ 的来源)
    return 5000 + axes * (CAL_ABORT_N * CAL_SEG_TIMEOUT_MS + pauses * CAL_PAUSE_MS) + 4480;
}

int cal_hist_frames(CalMode m, int cam_fps) {
    return (cal_plan_worst_ms(m) + 1000) * cam_fps / 1000;
}

// ========================= [2] 段窗口表与在线累计 =========================

void CalWinTable::reset(CalMode m) {
    const std::vector<CalPlanSeg> p = cal_plan(m);
    std::lock_guard<std::mutex> lk(mtx_);
    segs_.clear();
    segs_.reserve(p.size());
    for (const auto& s : p) {
        CalSegWin w;
        w.axis = s.axis; w.dir = s.dir; w.pause = s.pause; w.amp = s.amp;
        segs_.push_back(w);
    }
}
void CalWinTable::begin_seg(int i, std::chrono::steady_clock::time_point t) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (i >= 0 && i < (int)segs_.size()) {
        segs_[(size_t)i].begun = true; segs_[(size_t)i].t0 = t; segs_[(size_t)i].t1 = t; }
}
void CalWinTable::end_seg(int i, std::chrono::steady_clock::time_point t,
                          bool timeout, bool moved) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (i >= 0 && i < (int)segs_.size()) {
        segs_[(size_t)i].t1 = t; segs_[(size_t)i].timeout = timeout;
        segs_[(size_t)i].moved = moved; }
}
void CalWinTable::mark_skipped(int i) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (i >= 0 && i < (int)segs_.size()) segs_[(size_t)i].skipped = true;
}
void CalWinTable::clear() {
    std::lock_guard<std::mutex> lk(mtx_);
    segs_.clear();
}
std::vector<CalSegWin> CalWinTable::snapshot() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return segs_;
}
size_t CalWinTable::size() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return segs_.size();
}
CalWinTable g_cal_win;

int cal_note_sample(const CalibSample& s) {
    static int   cur = -1;
    static float acc = 0.0f;
    const std::vector<CalSegWin> w = g_cal_win.snapshot();
    int live = -1;
    for (size_t i = w.size(); i-- > 0; ) if (w[i].begun) { live = (int)i; break; }
    if (live < 0 || w[(size_t)live].pause || w[(size_t)live].skipped) {   // 停顿/未标定: 无激励段
        cur = -1; acc = 0.0f;
        g_cal_live_travel.store(0.0f); g_cal_live_slot.store(-1);
        return -1;
    }
    if (live != cur) { cur = live; acc = 0.0f; }        // 换段: 清零重计
    const CalSegWin& e = w[(size_t)live];
    acc += (float)e.dir * (e.axis ? s.sy : s.sx);       // 沿激励方向投影 (屏幕 px)
    g_cal_live_travel.store(acc);
    g_cal_live_slot.store(live);
    return live;
}

// ========================= [3] 拟合 =========================

namespace {

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
// 中位 + MAD (稳健散度) 与中位的标准误: MAD 描述实际散布, 量化底 dt/√12/√n 兜住
//   "样本少且恰好很齐"的情形 (边沿读数的量化是物理的, 不能当零)
struct Stat { float med = 0, mad = 0, se = 0; int n = 0; };
Stat stat_of(const std::vector<float>& v, double quant_floor) {
    Stat s; s.n = (int)v.size();
    if (v.empty()) return s;
    s.med = median_of(v);
    std::vector<float> dev; dev.reserve(v.size());
    for (float x : v) dev.push_back(std::fabs(x - s.med));
    s.mad = median_of(dev);
    const double se_mad = 1.4826 * (double)s.mad / std::sqrt((double)s.n);
    s.se = (float)std::max(se_mad, quant_floor / std::sqrt((double)s.n));
    return s;
}
// 两族读数的一致性容差: 3 倍合并标准误 (统计部分) + 物理部分 = 一个实测采样间隔
//   (边沿读数各带 ±dt/2 的帧格量化) 与 0.3·L (控制律被证明吸收的失配带, 见 cal_fit)。
double agree_tol(const Stat& a, const Stat& b, double frame) {
    const double se = std::sqrt((double)a.se * a.se + (double)b.se * b.se);
    return (double)CAL_CONSIST_Z * se
         + std::max(frame, (double)CAL_EDGE_TOL_REL * (double)a.med);
}
std::vector<const CalibSample*> pick(const std::deque<CalibSample>& hist,
                                     std::chrono::steady_clock::time_point a,
                                     std::chrono::steady_clock::time_point b) {
    std::vector<const CalibSample*> v;
    if (b < a) return v;
    for (const auto& s : hist) if (!(s.t < a) && !(s.t > b)) v.push_back(&s);
    return v;
}
double frame_of(const std::deque<CalibSample>& hist) {
    std::vector<float> dts; dts.reserve(hist.size());
    for (const auto& s : hist) dts.push_back(s.dt_ms);
    return std::max(1.0, (double)median_of(dts));
}

// 逐段测量: 段内样本上重建累计行程曲线 A(t), 用噪声包络 K·σ·√n 检出响应起点,
//   插值目标穿越点 → 屏速 v (只作尾迹的除数与现场读数), 并给出两个边沿读数。
//   样本按采样侧打的槽位号选取 —— 与状态机"到位即停"读的是同一批样本。
struct SegOut {
    bool  ok = false;
    const char* why = "";
    float val = 0, lim = 0;
    float v = 0;
    float travel = 0, t_ms = 0;
    float resp = 0, disp_med = 0, disp_max = 0, spread = 0, spread_max = 0, raw_med = 0;
    float l_onset = -1;
    float t_last = 0;
};

SegOut measure_seg(const std::deque<CalibSample>& hist, int slot, const CalSegWin& w,
                   float sigma, double frame) {
    SegOut o;
    const float target = CAL_TRAVEL_PX[w.axis];
    const float shift_max = calib_shift_max_px();

    // ---- 段内样本 (采样侧按槽位打标; 与在线累计同一批) ----
    std::vector<const CalibSample*> v;
    for (const auto& s : hist) if (s.slot == slot && s.ok[w.axis]) v.push_back(&s);
    if (v.empty()) { o.why = "段内无样本"; return o; }

    // ---- 量与门 (采样质量是"这一段能不能用"的前提) ----
    std::vector<float> rq, dm, rm, sp;
    for (auto* s : v) {
        dm.push_back(std::hypot(s->sx, s->sy));
        rm.push_back(std::hypot(s->sx_all, s->sy_all));
        sp.push_back(s->spread[w.axis]);
        rq.push_back(s->resp[w.axis]);
    }
    o.disp_med = median_of(dm);  o.disp_max = *std::max_element(dm.begin(), dm.end());
    o.raw_med = median_of(rm);
    o.spread = median_of(sp);    o.spread_max = *std::max_element(sp.begin(), sp.end());
    o.resp = median_of(rq);
    if (o.disp_max > shift_max) { o.why = "每帧位移超相关量程 (游戏太快/相关回卷)";
        o.val = o.disp_max; o.lim = shift_max; return o; }
    if (o.spread > shift_max) { o.why = "块间离散超相关量程 (相关不可信)";
        o.val = o.spread; o.lim = shift_max; return o; }
    if (o.resp < CAL_RESP_MIN) { o.why = "相位相关响应过低 (该方向背景无纹理)";
        o.val = o.resp; o.lim = CAL_RESP_MIN; return o; }

    // ---- 累计曲线上的测速 (中段割线) 与响应检出 ----
    //   响应检出: 累计行程在纯噪声下是随机游走 (标准差 ∝ σ√n), 固定 3σ 门几帧内就被噪声
    //   越过 → 检出点落在响应之前 10–50ms; 门限跟着包络走 (K·σ·√n) 才落在响应开始处。
    //   但检出点自身带噪声 (偶发虚警把起点判早), 而尾迹读数 = Σ/v 对 v 一阶敏感 —
    //   故 v 取**目标 25% 与 75% 两条穿越之间的割线**: 两端都在曲线上、都远离折点与
    //   饱和端, 检出点的噪声进不来, 折点由割线隐含而不必插值 (在折点上插值会早
    //   2–4ms, 高速段就是 10% 偏差)。两端都在同一条观测流上 → 环路延迟在斜率里相消。
    double acc = 0.0, a_prev = 0.0, t_prev = 0.0;
    bool have_on = false, have_off = false, have_25 = false, have_75 = false;
    double a_i0 = 0, t_25 = 0, t_75 = 0;
    int n_idx = 0;
    const double sig = (double)sigma;
    for (auto* s : v) {
        const double proj = (double)w.dir * (double)(w.axis ? s->sy : s->sx);
        const double t_cur = elapsed_ms(s->t, w.t0);
        const double a_cur = acc + proj;
        const int n_cur = ++n_idx;
        const double env = (double)CAL_EDGE_SNR * sig * std::sqrt((double)n_cur);
        if (!have_on && a_cur > env) {                 // 越过噪声包络 = 画面开始响应
            a_i0 = a_cur; have_on = true;
            // 起始沿: 画面开始动的第一个样本落在 (t0+L, t0+L+dt] 内 → 中位括号 ±dt/2
            o.l_onset = (float)(t_cur - 0.5 * (double)s->dt_ms);
        }
        auto cross = [&](double level, double& t_out) {
            if (a_prev < level && a_cur >= level && a_cur > a_prev) {
                t_out = t_prev + (level - a_prev) / (a_cur - a_prev) * (t_cur - t_prev);
                return true; }
            return false;
        };
        if (!have_25 && cross(0.25 * (double)target, t_25)) have_25 = true;
        if (!have_75 && cross(0.75 * (double)target, t_75)) have_75 = true;
        if (!have_off && a_cur >= (double)target) have_off = true;
        acc = a_cur; a_prev = a_cur; t_prev = t_cur;
    }
    o.t_last = (float)acc;
    if (w.timeout && !have_off) { o.why = "不可测 (超时: 累计行程未达目标 — 死区/太慢/夹紧/无游戏)";
        o.val = (float)acc; o.lim = target; return o; }
    if (!have_on) { o.why = "不可测 (累计行程未越过噪声包络: 段内无响应)";
        o.val = (float)acc; o.lim = (float)(CAL_EDGE_SNR * sig * std::sqrt((double)n_idx)); return o; }
    if (!have_off || !have_25 || !have_75) { o.why = "不可测 (累计行程未达目标)";
        o.val = (float)acc; o.lim = target; return o; }
    o.travel = (float)((double)target - a_i0);
    o.t_ms = (float)(t_75 - t_25);
    if (o.t_ms < frame) { o.why = "测量窗不足 (25%→75% 割线 < 一帧)";
        o.val = o.t_ms; o.lim = (float)frame; return o; }
    o.v = (float)(0.5 * (double)target / (double)o.t_ms * 1000.0);
    if (!(o.v > 0.0f)) { o.why = "速度非正"; return o; }
    o.ok = true;
    return o;
}

} // namespace

CalResult cal_fit(CalMode mode, const std::deque<CalibSample>& hist,
                  const std::vector<CalSegWin>& plan) {
    CalResult r;
    if (plan.empty()) { r.err = "无激励计划 (状态机未跑完)"; return r; }
    if (hist.size() < 2) { r.err = "标定期无样本 (采集线程未出样本)"; return r; }
    const double frame = frame_of(hist);
    r.dt_ms = (float)frame;

    // ---- [1] 噪声底 σ: 停顿的静止参考窗 (池化; 只覆盖被激励的轴) ----
    for (int a = 0; a < 2; ++a) {
        bool has = false;
        for (const auto& sg : plan)
            if (sg.pause && sg.axis == a && sg.begun && !sg.skipped) { has = true; break; }
        if (!has) { r.sigma[a] = 0.0f; r.sigma_n[a] = 0; continue; }
        std::vector<float> nz;
        for (const auto& sg : plan) {
            if (!sg.pause || sg.axis != a || !sg.begun || sg.skipped) continue;
            for (auto* s : pick(hist, shift_ms(sg.t0, (double)CAL_STATIC_FROM_MS), sg.t1))
                if (s->ok[a]) nz.push_back(a ? s->sy : s->sx);
        }
        r.sigma_n[a] = (int)nz.size();
        if ((int)nz.size() < CAL_SIGMA_MIN_N) { r.err = "停顿静止参考样本不足"; return r; }
        r.sigma[a] = 1.4826f * median_abs(nz);
        if (!(r.sigma[a] > 1e-3f)) {   // σ=0 = 停顿样本位移恒为零: 画面完全静止
            r.err = "画面完全静止 (停顿样本位移恒为 0): 无游戏或画面未响应";
            return r;
        }
    }

    // ---- [2] 逐段: 屏速 (尾迹的除数) 与三个读数 ----
    std::vector<float> tail[2], onset[2], stop[2];
    const double floor_px = (double)calib_edge_floor_px();
    for (size_t k = 0; k < plan.size() && k < (size_t)CAL_SEGS_MAX; ++k) {
        const CalSegWin& sg = plan[k];
        if (sg.pause || !sg.begun || sg.skipped || sg.dir == 0) continue;
        if (!sg.pause) ++r.n_seg_all;
        SegOut o = measure_seg(hist, (int)k, sg, r.sigma[sg.axis], frame);
        CalSegDiag& d = r.diag[k];
        d.played = true;
        d.axis = sg.axis; d.dir = sg.dir; d.amp = sg.amp;
        d.ok = o.ok; d.why = o.why; d.val = o.val; d.lim = o.lim;
        d.travel = o.travel; d.t_ms = o.t_ms; d.v = o.v;
        d.resp = o.resp; d.disp_med = o.disp_med; d.disp_max = o.disp_max;
        d.spread = o.spread; d.spread_max = o.spread_max; d.raw_med = o.raw_med;
        d.l_onset = o.l_onset;
        d.travel_end = o.t_last;
        if (o.ok) ++r.n_seg_ok;
        if (d.l_onset >= 0) onset[sg.axis].push_back(d.l_onset);

        // ---- [3] 该段后的停顿: 尾迹读数 (需要本段实测屏速 v) 与停止沿读数 ----
        if (k + 1 >= plan.size() || !plan[k + 1].pause || !plan[k + 1].begun) continue;
        const CalSegWin& p = plan[k + 1];
        std::vector<const CalibSample*> pv = pick(hist, sg.t1, p.t1);
        // 停止沿: 画面在观测里最后"还在动"的样本落 (t1+L−dt, t1+L] → 中位括号 −dt/2。
        //   与起始沿用同一条括号规则, 两个读数的量化偏差同向对称 (不引入系统偏移)。
        int last_moving = -1;
        for (size_t i = 0; i < pv.size(); ++i) {
            const double proj = (double)sg.dir
                              * (double)(sg.axis ? pv[i]->sy : pv[i]->sx);
            if (std::fabs(proj) > std::max((double)CAL_EDGE_SNR * (double)r.sigma[sg.axis],
                                           floor_px)) last_moving = (int)i;
        }
        if (last_moving >= 0) {
            const double l = elapsed_ms(pv[(size_t)last_moving]->t, sg.t1)
                           - 0.5 * (double)pv[(size_t)last_moving]->dt_ms;
            if (l >= -frame && l <= (double)CAL_PAUSE_MS) {
                d.l_stop = (float)l; stop[sg.axis].push_back((float)l);
            }
        }
        if (!o.ok || pv.size() < 2) continue;
        // 尾迹和 (主读数): 停顿窗内位移和 = v·(t₁ − t_first + dt_first + L)
        //   → L = Σ/v + (t_first − t₁) − dt_first (亚帧精确; 对掉帧免疫 — 和式在首末
        //   两个世界时刻之间望远镜式相消)
        double ssum = 0.0;
        for (auto* s : pv)
            if (s->ok[sg.axis]) ssum += (double)sg.dir * (double)(sg.axis ? s->sy : s->sx);
        const double l = ssum / (double)o.v * 1000.0
                       + elapsed_ms(pv[0]->t, sg.t1) - (double)pv[0]->dt_ms;
        d.l_tail = (float)l;
        if (l >= -frame && l <= (double)CAL_PAUSE_MS) tail[sg.axis].push_back((float)l);
        else {                                   // 读数越出停顿窗 = 这一段的尾迹没有参考
            d.l_tail = -1;
            if (!o.why[0]) { d.why = "尾迹读数越出停顿窗"; d.val = (float)l;
                             d.lim = (float)CAL_PAUSE_MS; }
        }
    }
    if (r.n_seg_ok == 0) {   // 运动前置: 停顿里的"边沿"只是噪声尖峰, L 无从测起
        r.err = "无任何激励段测到运动 (屏幕未响应) — 延迟无从测起";
        return r;
    }

    // ---- [4] 读数族的聚合: 中位 + MAD + 散度门 ----
    std::vector<float> all[3];
    for (int a = 0; a < 2; ++a) {
        all[0].insert(all[0].end(), tail[a].begin(), tail[a].end());
        all[1].insert(all[1].end(), onset[a].begin(), onset[a].end());
        all[2].insert(all[2].end(), stop[a].begin(), stop[a].end());
    }
    const double quant = frame / std::sqrt(12.0);     // 边沿读数的均匀量化标准误
    Stat st_tail = stat_of(all[0], 0.0), st_stop = stat_of(all[2], quant),
         st_on = stat_of(all[1], quant);
    r.l_tail = st_tail.med; r.l_tail_mad = st_tail.mad; r.l_tail_n = st_tail.n;
    r.l_stop = st_stop.med; r.l_stop_mad = st_stop.mad; r.l_stop_n = st_stop.n;
    r.l_onset = st_on.med;  r.l_onset_mad = st_on.mad;  r.l_onset_n = st_on.n;
    for (int a = 0; a < 2; ++a)
        if (!tail[a].empty()) { r.l_axis[a] = median_of(tail[a]); r.l_axis_n[a] = (int)tail[a].size(); }

    const float disp_lim = (float)(CAL_DISP_FRAMES * frame);
    auto family_usable = [&](const Stat& s) {
        return s.n >= CAL_MIN_READINGS && s.mad <= disp_lim; };
    const bool tail_ok = family_usable(st_tail), stop_ok = family_usable(st_stop),
               onset_ok = family_usable(st_on);
    if (!tail_ok && !stop_ok) {
        if (st_tail.n < CAL_MIN_READINGS && st_stop.n < CAL_MIN_READINGS)
            r.err = "读数不足 (有效段太少 — 每轴 2 对共 12 个读数, 少于 5 个不可聚合)";
        else r.err = "读数离散超一个采样间隔 (读数之间不是同一次物理测量的重复)";
        return r;
    }
    if (tail_ok && st_tail.med > (float)CAL_STATIC_FROM_MS) {
        r.err = "实测延迟超过静止参考窗起点 (停顿留不住尾迹 — 尾迹被截断)";
        return r;
    }
    if (tail_ok && onset_ok) {
        // 两读数一致性 = 3 倍合并标准误 + 容差。交叉检查用**起始沿**而不是停止沿:
        //   起始沿是纯时间差 (与增益、与检测门都无关), 尾迹读的是全部残余运动的等效
        //   滞后, 两者之差只可能来自 v 的误差、通道里的第二个滞后源或状态机的时间戳。
        //   停止沿不做门 — 它的检出是"逐帧位移落到检测门之下"的那一刻, 平滑把命令沿
        //   磨圆之后这个时刻随 (τ·ln(v·dt/门限)) 走, 天然比真值晚几十毫秒 (实测 τ=15ms
        //   时晚 47ms 而起始沿只晚 4ms)。它是现场看"响应形状"的读数, 不是第二个延迟
        //   测量值 — 照进日志, 不参与判定。
        //   容差: 边沿读数自身量化到帧格 (±dt/2 各一) → 一个实测帧长; 超过它的差还要
        //   给相对项 — 控制律被证明吸收的失配带是 ±30% (arena.integrate 的宽延迟带 +
        //   s 失配扫描), 故 0.3·L 之内两个读数给出的是同一个可用值, 再大说明通道里
        //   还有别的滞后源, 需要现场判断而不是硬写一个数。
        const double tol = agree_tol(st_tail, st_on, frame);
        r.consist_diff = st_tail.med - st_on.med;
        r.consist_tol = (float)tol;
        if (std::fabs((double)r.consist_diff) > tol) {
            r.err = "尾迹与起始沿读数不一致 (超容差 — 通道里还有别的滞后源)";
            return r;
        }
    }
    // 主读数 = 尾迹族中位; 尾迹族不可用时退回停止沿族 (增益无关的独立读数, 日志说明)
    const Stat& prim = tail_ok ? st_tail : st_stop;
    r.used_edges = !tail_ok;
    r.l_est = prim.med;

    // 两轴一致度 (hid): 一致 → 合并; 不一致 → 如实并列, 结论写进日志
    r.axes_agree = true;
    if (r.l_axis_n[0] >= CAL_MIN_READINGS && r.l_axis_n[1] >= CAL_MIN_READINGS) {
        const Stat s0 = stat_of(tail[0], 0.0), s1 = stat_of(tail[1], 0.0);
        r.axes_agree = std::fabs((double)s0.med - (double)s1.med) <= agree_tol(s0, s1, frame);
    }
    if (r.l_est < L_MIN || r.l_est > L_MAX) {
        r.err = "实测延迟超出物理带 (测量无效, 不硬钳制)";
        return r;
    }
    r.ok = true;
    return r;
}

// ========================= [4] 判定 / 诊断 / 回写 =========================

int cal_done_code(const CalResult& r) {
    return r.ok ? 1 : 2;
}

namespace {

const char* axis_name(int a) { return a ? "Y" : "X"; }

// 幅度标签: hid = counts/ms, pad = 满偏百分比
void amp_label(CalMode m, float amp, char* out, size_t n) {
    if (m == CAL_MODE_PAD) snprintf(out, n, "%.0f%%", (double)(amp * 100.0f + 0.5f));
    else snprintf(out, n, "%.3g/ms", (double)amp);
}

} // namespace

void cal_print_diag(CalMode mode, const CalResult& r, size_t hist_n) {
    // 逐段原始行 (失败时它就是现场证据): 行程/耗时/屏速/每帧位移/相关峰/离散/三读数
    auto rd = [](float v, char* b, size_t n) {
        if (v >= 0) snprintf(b, n, "%.0f", (double)v); else snprintf(b, n, "-"); };
    for (size_t k = 0; k < (size_t)CAL_SEGS_MAX; ++k) {
        const CalSegDiag& d = r.diag[k];
        if (!d.played) continue;
        char lab[24]; amp_label(mode, d.amp, lab, sizeof(lab));
        char lon[16], lst[16], ltl[16];
        rd(d.l_onset, lon, sizeof(lon)); rd(d.l_stop, lst, sizeof(lst));
        rd(d.l_tail, ltl, sizeof(ltl));
        printf("[标定] 段 %zu %s%s %s: ", k, axis_name(d.axis), d.dir > 0 ? "+" : "-", lab);
        if (d.ok)
            printf("屏速 %.0fpx/s 行程 %.0fpx/%.0fms 每帧 %.1f/%.1fpx 峰 %.2f 离散 %.1f/%.1fpx "
                   "剔除前 %.1fpx 段末 %.0fpx | 尾迹 %s 停止沿 %s 起始沿 %s ms\n",
                   (double)d.v, (double)d.travel, (double)d.t_ms,
                   (double)d.disp_med, (double)d.disp_max, (double)d.resp,
                   (double)d.spread, (double)d.spread_max, (double)d.raw_med,
                   (double)d.travel_end, ltl, lst, lon);
        else
            printf("不可测: %s (实测 %.3g vs 门 %.3g) | 尾迹 %s 停止沿 %s 起始沿 %s ms\n",
                   d.why[0] ? d.why : "未播完", (double)d.val, (double)d.lim, ltl, lst, lon);
    }
    printf("[标定] 噪声底 σx=%.3f σy=%.3f px/帧 (静止窗 %d/%d 样本, 起点 %dms) | 采样间隔 %.2fms "
           "(≈%.0ffps) | 有效激励段 %d/%d\n",
           (double)r.sigma[0], (double)r.sigma[1], r.sigma_n[0], r.sigma_n[1],
           CAL_STATIC_FROM_MS, (double)r.dt_ms, r.dt_ms > 0 ? 1000.0 / (double)r.dt_ms : 0.0,
           r.n_seg_ok, r.n_seg_all);
    printf("[标定] 读数: 尾迹 %d 条 中位 %.1f 离散 %.1fms | 停止沿 %d 条 中位 %.1f 离散 %.1fms "
           "| 起始沿 %d 条 中位 %.1f 离散 %.1fms (单位 ms, 散度 = MAD)\n",
           r.l_tail_n, (double)r.l_tail, (double)r.l_tail_mad,
           r.l_stop_n, (double)r.l_stop, (double)r.l_stop_mad,
           r.l_onset_n, (double)r.l_onset, (double)r.l_onset_mad);
    // 两读数之差无论成败都进日志 (现场最有用的诊断): 尾迹 vs 起始沿 及其容差
    printf("[标定] 尾迹−起始沿 = %+.1fms (容差 ±%.1fms = 3·SE + max(一个采样间隔, 0.3·L))\n",
           (double)r.consist_diff, (double)r.consist_tol);
    if (r.l_axis_n[0] || r.l_axis_n[1])
        printf("[标定] 逐轴 (尾迹族): X %d 条 中位 %.1fms | Y %d 条 中位 %.1fms → %s\n",
               r.l_axis_n[0], (double)r.l_axis[0], r.l_axis_n[1], (double)r.l_axis[1],
               (r.l_axis_n[0] && r.l_axis_n[1])
                   ? (r.axes_agree ? "两轴一致, 合并取全轮中位"
                                   : "两轴不一致 (逐轴平滑策略不同? 延迟仍是一个量), 取全轮中位")
                   : (r.l_axis_n[0] ? "仅 X 轴出读数" : "仅 Y 轴出读数"));
    if (r.err[0]) printf("[标定] 无法测量: %s (样本 %zu)\n", r.err, hist_n);
    else if (r.used_edges)
        printf("[标定] L=%.1f ms (尾迹族不可用, 取停止沿族中位) — 只标延迟; 速度不回写 "
               "(手感走 spdx/spdy)\n", (double)r.l_est);
    else
        printf("[标定] L=%.1f ms (尾迹族中位; 三读数见上) — 只标延迟; 速度不回写 "
               "(手感走 spdx/spdy)\n", (double)r.l_est);
    fflush(stdout);
}

bool cal_writeback(const char* var, const CalResult& r, const std::string& persist_path) {
    if (!r.ok || persist_path.empty()) return false;
    return persist_calibration(persist_path, var, r.l_est);
}

// ========================= 状态机 =========================

namespace {

enum CalPhase {
    CP_IDLE = 0,      // 空闲: 触发长按 / 热参请求
    CP_START,         // 起始十字 (视觉开始信号 + 触发后的准备期, 不采样)
    CP_EXCITE,        // 逐段激励 (段间停顿; g_calib_collect 开, 采样中)
    CP_SETTLE,        // 激励后静置 (采样窗收尾)
    CP_WAIT,          // 等采样侧拟合回执 (坐杆静置)
    CP_END_OK,        // 收尾: 成功点头
    CP_END_FAIL,      // 收尾: 失败摇头
};

struct CalibSeg { int16_t dx, dy; int ticks; };

// 起始十字 (纯视觉开始信号, **兼触发后的准备期** — 操作者需要时间把手拿开; 不采样)
//   与收尾动作 (成功 = 纵向点头 / 失败 = 横向摇头)。以起点为中心, 单侧方块会把准星
//   推离操作者放好的屏幕中心。
inline const CalibSeg CAL_PAD_START_SEQ[] = {
    { 9830,0,ms_to_ticks(240)},{-9830,0,ms_to_ticks(240)},
    {0, 9830,ms_to_ticks(240)},{0,-9830,ms_to_ticks(240)},{0,0,ms_to_ticks(500)}};
inline const CalibSeg CAL_HID_START_SEQ[] = {
    {2,0,ms_to_ticks(240)},{-2,0,ms_to_ticks(240)},
    {0,2,ms_to_ticks(240)},{0,-2,ms_to_ticks(240)},{0,0,ms_to_ticks(500)}};
inline const CalibSeg CAL_SETTLE_SEQ[] = {{0,0,ms_to_ticks(300)}};
inline const CalibSeg CAL_PAD_END_OK_SEQ[] = {          // 成功 = 纵向点头 2 次
    {0, 9830,ms_to_ticks(120)},{0,-9830,ms_to_ticks(120)},
    {0, 9830,ms_to_ticks(120)},{0,-9830,ms_to_ticks(120)},
    {0, 9830,ms_to_ticks(120)},{0,-9830,ms_to_ticks(120)}};
inline const CalibSeg CAL_PAD_END_FAIL_SEQ[] = {        // 失败 = 横向摇头
    { 9830,0,ms_to_ticks(120)},{-9830,0,ms_to_ticks(120)},
    { 9830,0,ms_to_ticks(120)},{-9830,0,ms_to_ticks(120)},
    { 9830,0,ms_to_ticks(120)},{-9830,0,ms_to_ticks(120)}};
inline const CalibSeg CAL_HID_END_OK_SEQ[] = {
    {0,4,ms_to_ticks(60)},{0,-4,ms_to_ticks(60)},{0,4,ms_to_ticks(60)},
    {0,-4,ms_to_ticks(60)},{0,4,ms_to_ticks(60)},{0,-4,ms_to_ticks(60)}};
inline const CalibSeg CAL_HID_END_FAIL_SEQ[] = {
    {4,0,ms_to_ticks(60)},{-4,0,ms_to_ticks(60)},{4,0,ms_to_ticks(60)},
    {-4,0,ms_to_ticks(60)},{4,0,ms_to_ticks(60)},{-4,0,ms_to_ticks(60)}};

struct CalSt {
    CalPhase phase = CP_IDLE;
    int hold = 0, wt = 0;
    std::vector<CalSegWin> plan;       // 本轮计划 (段窗口表快照; 轮开始时冻结: spd 中途变化不改本轮)
    int pi = 0, pt = 0;
    float rem = 0.0f;                  // hid 速率量化的余量 (平均率精确)
    int cur_axis = -1, axis_timeouts = 0;
    const CalibSeg* seq = nullptr;
    int slen = 0, si = 0, st = 0;
};
CalSt g_st[CAL_MODES_N];

template <size_t N>
void enter(CalSt& s, const CalibSeg (&seq)[N], CalPhase ph) {
    s.seq = seq; s.slen = (int)N; s.si = s.st = 0; s.phase = ph;
}

void start_round(CalSt& s, CalMode mode) {
    g_cal_win.reset(mode);
    s.plan = g_cal_win.snapshot();       // 计划冻结: 状态机与采样侧共用同一份
    s.pi = s.pt = 0; s.rem = 0.0f; s.axis_timeouts = 0;
    s.cur_axis = s.plan.empty() ? -1 : s.plan[0].axis;
    g_cal_live_travel.store(0.0f); g_cal_live_slot.store(-1);
    // 上一轮的回执随本轮开跑清零: 回执只由采样侧在拟合完成后写, 不清零则第二轮起的
    //   CP_WAIT 会立刻读到旧值 (点头/摇头报的是上一轮的结论, 等待窗也被跳过)
    g_calib_done.store(0);
    g_calib_collect.store(true);
    s.phase = CP_EXCITE;
}

void reset_round(CalSt& s) {
    s.phase = CP_IDLE; s.pi = s.pt = 0; s.rem = 0.0f;
    s.cur_axis = -1; s.axis_timeouts = 0;
    s.plan.clear(); s.seq = nullptr; s.slen = s.si = s.st = 0;
    g_calib_collect.store(false);
    g_cal_win.clear();
    g_cal_live_travel.store(0.0f); g_cal_live_slot.store(-1);
}

// 一个轴连续 CAL_ABORT_N 段超时 = 该轴不响应: 跳过它剩下的**激励段** (整轮时长因此
//   有界), 但保留段后停顿 — 停顿是静止参考窗的来源 (σ 的样本池), 噪声底与响应无关,
//   屏幕不动时它正是唯一还能测的量。
void skip_axis(CalSt& s, CalMode mode, int axis, int from) {
    printf("[标定] %s 轴连续 %d 段超时 (屏幕未响应) → 跳过该轴其余激励段, 该轴不入读数\n",
           axis ? "Y" : "X", CAL_ABORT_N);
    for (int j = from; j < (int)s.plan.size() && s.plan[j].axis == axis; ++j)
        if (!s.plan[j].pause) {           // 状态机读自己的计划副本, 采样侧读窗口表 — 两处都标
            s.plan[j].skipped = true;
            g_cal_win.mark_skipped(j);
        }
    s.pi = from; s.pt = 0; s.axis_timeouts = 0;
    s.cur_axis = (from < (int)s.plan.size()) ? s.plan[(size_t)from].axis : -1;
    (void)mode;
}

} // namespace

CalStep cal_step(CalMode mode, uint16_t btns, int cam_fps,
                  std::chrono::steady_clock::time_point now) {
    (void)cam_fps;                                   // 段长不再由帧率导出 (到位即停)
    CalSt& s = g_st[mode];
    CalStep out;

    bool req = false;
    if (mode == CAL_MODE_PAD) req = g_padcalib_request.exchange(false);
    if (req && s.phase != CP_IDLE) {                 // 进行中到达的请求: 一次消费即清
        std::cout << "[标定] 忽略请求: 标定进行中\n";
        req = false;
    }
    if (!g_aim_enabled.load()) {                     // 接管关闭 = 纯透传: 激励与之互斥
        if (s.phase != CP_IDLE) {
            reset_round(s);
            std::cout << "[标定] 中断: 接管关闭 (纯透传)\n";
        }
        s.hold = 0;
        if (req) std::cout << "[标定] 忽略请求: 接管关闭 (纯透传)\n";
        return out;
    }

    if (s.phase == CP_IDLE) {
        const uint16_t mask = (mode == CAL_MODE_PAD) ? (uint16_t)(PADBTN_L3 | PADBTN_R3)
                                                     : (uint16_t)BOTH_SIDE_KEYS;
        if ((btns & mask) == mask) ++s.hold; else s.hold = 0;
        if (req || s.hold >= CALIB_TRIGGER_TICKS) {
            s.hold = 0;
            printf("[标定] 触发 (%s): %s, 段间停顿 %dms, 到位即停; 只标延迟; 全程 ≤%.1fs "
                   "— 请双手离开控制器\n",
                   mode == CAL_MODE_PAD ? "手柄 L3+R3 / webui" : "鼠标双侧键",
                   mode == CAL_MODE_PAD ? "水平轴分级激励 (摇杆偏转)"
                                        : "X/Y 两轴速率激励 (鼠标 counts)",
                   CAL_PAUSE_MS, (double)cal_plan_worst_ms(mode) / 1000.0);
            fflush(stdout);
            enter(s, mode == CAL_MODE_PAD ? CAL_PAD_START_SEQ : CAL_HID_START_SEQ, CP_START);
            out.active = true;                       // 触发拍不播激励
            return out;
        }
    }

    if (s.phase == CP_WAIT) {
        out.active = true;                           // 等拟合期摇杆静置
        const int done = g_calib_done.load();
        if (done != 0 || ++s.wt > CALIB_WAIT_TIMEOUT) {
            const bool ok = (done == 1);
            if (mode == CAL_MODE_PAD) enter(s, ok ? CAL_PAD_END_OK_SEQ : CAL_PAD_END_FAIL_SEQ,
                                            ok ? CP_END_OK : CP_END_FAIL);
            else enter(s, ok ? CAL_HID_END_OK_SEQ : CAL_HID_END_FAIL_SEQ,
                       ok ? CP_END_OK : CP_END_FAIL);
        }
        return out;
    }

    if (s.phase == CP_EXCITE) {
        out.active = true;
        if (s.pi >= (int)s.plan.size()) { enter(s, CAL_SETTLE_SEQ, CP_SETTLE); return out; }
        const CalSegWin& sg = s.plan[(size_t)s.pi];
        if (sg.axis != s.cur_axis) { s.cur_axis = sg.axis; s.axis_timeouts = 0; }
        if (sg.skipped) { ++s.pi; s.pt = 0; return out; }   // 该轴已判不响应: 不播该段
        if (s.pt == 0) g_cal_win.begin_seg(s.pi, now);
        if (!sg.pause) {                             // 激励拍: 按模式给出注入量
            if (mode == CAL_MODE_PAD) {
                // 偏转 (满偏比例 → counts); 状态机在 pad 侧整只手柄独占 (见 pad_output)
                const int16_t d = (int16_t)std::lround((double)sg.dir * (double)sg.amp
                                                      * (double)PAD_AXIS_MAX);
                if (sg.axis == 0) out.cx = d; else out.cy = d;
            } else {
                // hid: 每拍命令 = 速率×拍长 的整数量化 (余量累加 → 平均率精确)
                s.rem += (float)sg.dir * sg.amp * TICK_MS;
                const int c = (int)std::trunc(s.rem);
                s.rem -= (float)c;
                if (sg.axis == 0) out.cx = (int16_t)c; else out.cy = (int16_t)c;
            }
        }
        bool stop = false, timeout = false;
        if (sg.pause) stop = (s.pt + 1 >= ms_to_ticks(CAL_PAUSE_MS));
        else {
            // 到位即停: 采样侧在线累计的实测行程 (沿激励方向) 达目标即停段
            const bool reached = g_cal_live_slot.load() == s.pi
                              && g_cal_live_travel.load() >= CAL_TRAVEL_PX[sg.axis];
            if (reached) stop = true;
            else if (s.pt + 1 >= ms_to_ticks(CAL_SEG_TIMEOUT_MS)) { stop = true; timeout = true; }
        }
        if (stop) {
            const bool moved = (g_cal_live_slot.load() == s.pi)
                            && (g_cal_live_travel.load() > 0.0f);
            g_cal_win.end_seg(s.pi, now, timeout, moved);
            if (timeout) {
                char lab[24];
                if (mode == CAL_MODE_PAD)
                    snprintf(lab, sizeof(lab), "%d%%", (int)(sg.amp * 100.0f + 0.5f));
                else snprintf(lab, sizeof(lab), "%.3g/ms", (double)sg.amp);
                printf("[标定] 段 %d %s%s%s 超时: 累计行程 %.0f/%.0fpx (%dms 未达目标) → "
                       "该段屏速 < %.0fpx/s, 按不可测计\n", s.pi, axis_name(sg.axis),
                       sg.dir > 0 ? "+" : "-", lab, (double)g_cal_live_travel.load(),
                       (double)CAL_TRAVEL_PX[sg.axis], CAL_SEG_TIMEOUT_MS,
                       (double)CAL_TRAVEL_PX[sg.axis] / (double)CAL_SEG_TIMEOUT_MS * 1000.0);
                fflush(stdout);
                ++s.axis_timeouts;
            } else if (!sg.pause) s.axis_timeouts = 0;   // 激励段成功才清计数 (停顿与命令无关)
            s.pt = 0; ++s.pi;
            if (s.axis_timeouts >= CAL_ABORT_N
                && s.pi < (int)s.plan.size() && s.plan[(size_t)s.pi].axis == sg.axis)
                skip_axis(s, mode, sg.axis, s.pi);
        } else ++s.pt;
        return out;
    }

    // CP_START / CP_SETTLE / CP_END_*: 播放动画段
    if (s.phase != CP_IDLE) {
        out.active = true;
        if (s.si < s.slen) {
            out.cx = s.seq[s.si].dx; out.cy = s.seq[s.si].dy;
            if (++s.st >= s.seq[s.si].ticks) { s.st = 0; ++s.si; }
        }
        if (s.si >= s.slen) {
            if (s.phase == CP_START) start_round(s, mode);
            else if (s.phase == CP_SETTLE) {
                g_calib_collect.store(false);
                g_calib_request.store(true);
                s.wt = 0; s.phase = CP_WAIT;
            } else reset_round(s);
        }
    }
    return out;
}
