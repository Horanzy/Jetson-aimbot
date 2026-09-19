// ============================================================================
//  pad_test — build/pad_test 单测 (scripts/compile.sh 构建, calib_test 之后执行):
//    [1] 注入换算与合并钳制 (逐轴满偏转屏速: 1.5px/ms@3000px/s = 半偏, 3px/ms = 满偏;
//        人类+注入 ±32767 钳制, 两轴各用各自的 gain 而互不串扰)
//    [1b] 行程形状 = 圆 (径向限幅, 实测驱动) 与账本按最终提交值入账
//    [2] 全透传 1:1: 按键/左摇杆/扳机逐位直通; 零注入时右摇杆=人类通道
//    [3] 摇杆账本 Σ(合并偏转×实际拍时长) (g_counts 不变式 3 的 pad 对应物)
//    [4] 触发判据与 -k 映射: RT≠0=fire / LT≠0=ads, 注入门 = 接管 × 保持窗
//    [5] own_motion_ledger 模式路由 + own_motion_scale 的逐轴比例 (hid 单值 s /
//        pad 双值 gain) — 自身运动补偿换算的来源选择
//    [6] 手柄未在位: 查找空路径, reader 线程不阻塞不停机
//    [7] 发布点: pad_tick 覆盖写最新槽 + seq 单调递增; 内容 = 人类态合并零注入
//    [8] XInput 线格式: 20B 报告的报头/按键位表/扳机直映/摇杆 int16LE 组装与
//        Y 轴口径, 保留字节恒零
//    [9] XInput 设备字节: 身份/配置节/类特定 blob/双端点/无限定符/全速/vendor
//        不钩子/字符串 — 上线外观的逐项断言
//   [10] 激励计划表 (两轴同级集 {10,25,50,70}%, 每级 ± 各一段, 每段后接停顿) 与
//        自适应段时长 (中段统计下限/行程目标/流程预算上限/探针级; 按 p≤2 的最陡
//        曲线假设定长, 行程不超目标)
//   [11] 标定状态机: L3+R3 长按 5s / 热参请求触发, 中途松开复位, 相位推进
//        (起始方块 → 分轴分级激励 + 段间停顿 → 静置 → 请求计算 → 等回执 →
//        点头/摇头 → 空闲), 级间段长随实测屏速自适应, 成功/失败两条回执路径,
//        接管关闭时不可达且复位
//   [12] 激励期独占右摇杆 (人类右摇杆被忽略, 其余字段直通) 与账本入账
//   [13] 量纲换算 (逐轴 px per 偏转·ms ↔ 满偏转屏速 px/s) 与设计带钳制/逐轴判定
//   [14] 拟合 (合成"停顿+瞬态"链路: 账本 + 幂律屏幕位移 + 质量量) → 复原两轴各自的
//        (A,p) 与延迟 L (停顿位移和与边沿两条读数); 两轴曲线不同仍各自复原 (无串扰);
//        **T 与 L 同量级时停顿法仍无偏, 而"段内直接平均"系统性低估** (本次修正的动机);
//        L > P 的自校验路径
//   [15] 级有效性/降级: 越量程级整级丢弃 (70% 被丢仍由其余级复原)、夹紧级丢弃、
//        响应过低丢弃、恰好 1 级走线性回退、全丢按失败 — 丢弃原因可读
//   [16] 回写: persist_calibration 的多 VAR 参数化 (hid/pad 两套互不覆盖),
//        原子替换/权限保留/缺行追加
//  全部断言通过输出 ALL PASS 并返回 0。
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cfloat>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

#include "core/calib.h"
#include "core/control.h"
#include "core/state.h"
#include "io/pad_calib.h"
#include "io/pad_input.h"
#include "io/pad_output.h"
#include "io/pad_xinput.h"

static int g_fail = 0;

#define CHECK(cond, msg) do { \
    if (cond) { std::cout << "  ok  " << msg << "\n"; } \
    else { std::cerr << "  FAIL " << msg << "\n"; ++g_fail; } \
} while (0)

static std::chrono::steady_clock::time_point g_t0;
static std::chrono::steady_clock::time_point at_ms(double ms) { return shift_ms(g_t0, ms); }

static double seg_ms(const CalibSeg& s) { return s.ticks * (double)TICK_MS; }
static bool seg_small(const CalibSeg& s) {          // 单轴且 |偏转| = 25% 视觉信号
    return std::abs(s.dx) + std::abs(s.dy) == PAD_CAL_ANIM_DEFL;
}
static std::string read_file(const std::string& p) {
    std::ifstream in(p); std::ostringstream o; o << in.rdbuf(); return o.str();
}
static bool has_line(const std::string& text, const std::string& line) {
    std::istringstream in(text); std::string l;
    while (std::getline(in, l)) if (l.rfind(line, 0) == 0) return true;
    return false;
}
static bool near(float a, float b, float rel) {
    return std::fabs(a - b) <= rel * std::max(1.0f, std::fabs(b));
}
static float median_copy(std::vector<float> v) {
    if (v.empty()) return 0.0f;
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
}

// ========================= 合成链路 (与真实链路同构) =========================
// 按计划表逐段播放 (激励段 seg_ms, 段后停顿 PAD_CAL_PAUSE_MS), 账本逐毫秒记偏转,
//   样本按 120fps 给出 — 屏幕位移 = 观测模型"t 时刻的位移 = 世界在 [t−L−dt, t−L]
//   的位移" 在幂律速度 A·d^p 下的**精确积分** (逐段积分, 段内速度常数)。于是:
//     中段样本 (窗完全落在段内) 与模型 disp = g·C 逐位自洽 → 停顿法无偏;
//     段首样本的窗跨过停顿 → "段内直接平均"会把这段瞬态算进均值 → 系统性低估;
//     停顿内位移之和 = V·L 的精确积分 → L 的亚帧读数。
// 逐级覆写: scale=0 模拟俯仰夹紧 (屏幕不响应注入), scale>1 模拟位移放大/相关回卷,
//   spread 覆写模拟块间离散, lowresp 模拟相关响应过低。
struct Synth {
    float A[2] = {3000.0f, 1200.0f};
    float p[2] = {1.0f, 1.0f};
    double L_true = 50.0;
    int    seg_ms = 250;
    int    fps = 120;          // 名义采样帧率 (实测链路里处理线程可能慢于 -f 的标称值)
    float  jitter = 0.0f;      // 帧长抖动 (相对标准差; 0 = 等间隔)
    int    drop_every = 0;     // 每 N 帧丢一个样本 (0 = 不丢)
    float  noise = 0.02f;      // 默认给一点采样噪声 (真实链路恒有; σ=0 时噪声底不可测)
    int    seed = 20240919;
    float  scale[2][PAD_CAL_LEVELS_N];
    float  spread[2][PAD_CAL_LEVELS_N];
    bool   lowresp[2][PAD_CAL_LEVELS_N];
    float  dir_scale[2][2];      // [轴][方向 0=+ 1=−] 响应覆写 (0 = 该方向夹紧)
    float  resp_dir[2][2];       // [轴][方向] 相关峰 (压低 = 该方向无纹理)
    Synth() {
        for (int a = 0; a < 2; ++a) {
            for (int l = 0; l < PAD_CAL_LEVELS_N; ++l) {
                scale[a][l] = 1.0f; spread[a][l] = 2.0f; lowresp[a][l] = false; }
            for (int k = 0; k < 2; ++k) { dir_scale[a][k] = 1.0f; resp_dir[a][k] = 0.5f; }
        }
    }
};

static std::vector<PadExcSeg> synth_play(const Synth& sy, double base_ms,
                                         std::deque<CalibSample>& hist) {
    const auto& pl = pad_cal_plan();
    std::vector<double> s0(pl.size()), s1(pl.size());
    double t = base_ms;
    for (size_t i = 0; i < pl.size(); ++i) {
        const double dur = pl[i].pause ? PAD_CAL_PAUSE_MS : sy.seg_ms;
        s0[i] = t; s1[i] = t + dur; t += dur;
    }
    const double total = t - base_ms;
    std::vector<PadExcSeg> win(pl.size());
    for (size_t i = 0; i < pl.size(); ++i) {
        win[i].axis = pl[i].axis; win[i].level = pl[i].level; win[i].d = pl[i].d;
        win[i].defl = pl[i].defl; win[i].pause = pl[i].pause; win[i].begun = true;
        win[i].t0 = at_ms(s0[i]); win[i].t1 = at_ms(s1[i] - 1.0);
    }
    CountsHistory& led = g_pad_ledger;
    for (int k = -200; k < 0; ++k) led.add(at_ms(base_ms + k), 0, 0);   // 前置零段
    for (int k = 0; k < (int)total; ++k) {
        size_t i = 0; while (i + 1 < pl.size() && s0[i + 1] <= base_ms + k) ++i;
        led.add(at_ms(base_ms + k), pl[i].axis ? 0 : pl[i].defl,
                pl[i].axis ? pl[i].defl : 0);
    }
    const double dt_nom = 1000.0 / (double)sy.fps;
    std::mt19937 rng((unsigned)sy.seed);
    std::normal_distribution<double> nd(0.0, 1.0);
    hist.clear();
    int idx = 0;
    double prev_tt = base_ms;
    for (double tt = base_ms; tt < base_ms + total; ) {
        size_t j = 0; while (j + 1 < pl.size() && s0[j + 1] <= tt) ++j;
        // 观测窗 = [t−dt−L, t−L], dt = **实际**帧间隔 (帧抖动如实进模型与账本回溯)
        const double dt = tt > prev_tt ? tt - prev_tt : dt_nom;
        const double w0 = tt - sy.L_true - dt, w1 = tt - sy.L_true;
        double dx = 0, dy = 0;
        for (size_t i = 0; i < pl.size(); ++i) {
            if (pl[i].pause) continue;
            const double ov = std::min(w1, s1[i]) - std::max(w0, s0[i]);
            if (ov <= 0) continue;
            const int ax = pl[i].axis, lv = pl[i].level;
            const double sign = pl[i].defl > 0 ? 1.0 : -1.0;
            const double v = (double)sy.A[ax] * std::pow((double)pl[i].d, (double)sy.p[ax])
                           * (double)sy.scale[ax][lv]
                           * (double)sy.dir_scale[ax][pl[i].defl > 0 ? 0 : 1];
            if (ax == 0) dx += sign * v * ov / 1000.0;
            else         dy += sign * v * ov / 1000.0;
        }
        const int ax = pl[j].axis, lv = pl[j].level;
        const bool dropped = sy.drop_every > 0 && (idx % sy.drop_every) == sy.drop_every - 1;
        if (!dropped) {
            const float nx = sy.noise > 0 ? (float)(nd(rng) * sy.noise) : 0.0f;
            const float ny = sy.noise > 0 ? (float)(nd(rng) * sy.noise) : 0.0f;
            hist.push_back({at_ms(tt), (float)dt, (float)dx + nx, (float)dy + ny,
                            sy.lowresp[ax][lv] ? 0.01f
                                               : sy.resp_dir[ax][pl[j].defl > 0 ? 0 : 1],
                            sy.spread[ax][lv]});
            prev_tt = tt;
        }
        ++idx;
        tt += dt_nom * (sy.jitter > 0 ? (1.0 + (double)sy.jitter * nd(rng)) : 1.0);
    }
    return win;
}

// "段内直接平均"对照 (只去掉段尾一帧, 把段首瞬态算进均值) — 用于钉住动机用例
static double naive_gain(const std::deque<CalibSample>& hist, const PadExcSeg& sg,
                         double frame) {
    const CountsHistory& led = g_pad_ledger;
    const auto wend = shift_ms(sg.t1, (double)TICK_MS - frame);
    double cc = 0, dc = 0;
    for (const auto& s : hist) {
        if (s.t < sg.t0 || s.t > wend) continue;
        auto c0 = led.at(shift_ms(s.t, -(double)s.dt_ms));
        auto c1 = led.at(s.t);
        double cx = c1.first - c0.first, cy = c1.second - c0.second;
        cc += cx * cx + cy * cy;
        dc += (double)s.sx * cx + (double)s.sy * cy;
    }
    return cc > 0 ? dc / cc : 0.0;
}
static double true_g(const Synth& sy, int axis, float d) {       // px per 偏转·ms
    return (double)sy.A[axis] * std::pow((double)d, (double)sy.p[axis] - 1.0)
         / ((double)PAD_AXIS_MAX * 1000.0);                      // px/s → px/ms 口径
}

static const float SYN_SHIFT_MAX = pad_cal_shift_max_px(106);   // cap_w=640 → 块 106

// ============ 闭环合成链路 (实时驱动状态机; 触发 → 激励 → 停顿 → 拟合 → 回写) ============
// 与 ai_thread 同形: 每拍调用 pad_calib_step, 激励拍经 pad_excite 写摇杆账本 (与真机同一条
//   入账路径); 采样按虚拟 120fps 生成 — 屏幕位移 = 观测模型 [t−L−dt, t−L] 上真实屏速
//   (幂律 A·d^p; 逐级响应可覆写) 的积分, 加高斯噪声; 每帧位移超过 max_disp_px 时该帧不
//   出样本 (模拟采样端的块响应门: 画面移动越快相关峰越弱); 收到 g_calib_request 时按
//   ai_thread 的判定跑 pad_calib_fit → pad_cal_done_code → 回写脚本。
// 节拍: 一拍 = TICK_MS 真实墙钟 (与真机同), 采样按 1000/fps 拍一个样本 — 段长/L/帧长
//   的口径与真机逐项相同, 故闭环行为 (探针→段长→拟合) 与真机同构。单场景 ≈ 一个完整
//   激励计划 (40 段 × (段长+150ms)) ≈ 17–25s, 故只跑一个场景; 判据/逐方向/重跑等
//   分支用合成窗与状态机单测覆盖 (那些不需要真实节拍)。
struct LoopCfg {
    float  A[2] = {3000.0f, 3000.0f};        // 满偏转屏速 px/s
    float  p[2] = {1.0f, 1.0f};
    double L_true = 45.0;                    // 环路延迟 ms
    float  noise = 0.015f;                   // 每样本位移噪声 px
    float  fps = 120.0f;                     // 采样帧率 (虚拟)
    float  level_scale[2][PAD_CAL_LEVELS_N]; // 逐级响应覆写 (0 = 死区)
    float  dir_scale[2][2];                  // [轴][方向: 0=+, 1=−] 响应覆写 (0 = 夹紧)
    float  resp_dir[2][2];                   // 该方向的相位相关峰 (压低 = 无纹理)
    float  max_disp_px = 0.0f;               // >0: 每帧位移超它 → 该帧不出样本
    int    gate_rounds = 99;                 // 该门只在前 N 轮生效 (瞬态)
    unsigned seed = 7;
    LoopCfg() {
        for (int a = 0; a < 2; ++a) {
            for (int l = 0; l < PAD_CAL_LEVELS_N; ++l) level_scale[a][l] = 1.0f;
            for (int k = 0; k < 2; ++k) { dir_scale[a][k] = 1.0f; resp_dir[a][k] = 0.5f; }
        }
    }
};
struct LoopOut {
    PadCalibResult res{};
    std::vector<PadExcSeg> plan;
    int    done = 0, fits = 0;
    double seg_ms[PAD_CAL_AXES_N][PAD_CAL_LEVELS_N][PAD_CAL_SEGS_PER_LEVEL];
    int    seg_n[PAD_CAL_AXES_N][PAD_CAL_LEVELS_N];
    double travel[PAD_CAL_AXES_N][PAD_CAL_LEVELS_N];     // 实测行程 (px, 真实口径)
    bool   wrote = false;
    std::string script;
    int    hist_n = 0;
    float  probe_px_s = 0;
};

static LoopOut run_loop(const LoopCfg& cf, const std::string& script_path) {
    LoopOut o{};
    for (int a = 0; a < PAD_CAL_AXES_N; ++a) for (int l = 0; l < PAD_CAL_LEVELS_N; ++l) {
        o.seg_n[a][l] = 0; o.travel[a][l] = 0; }
    own_motion_ledger_set(true);
    g_pad_ledger.clear();                     // 闭环 harness 用真实钟 → 丢弃其它时间基
    const bool  saved_aim  = g_aim_enabled.load();
    const float saved_gx   = g_pad_stick_gain_x.load();
    const float saved_gy   = g_pad_stick_gain_y.load();
    g_aim_enabled.store(true);
    g_calib_collect.store(false); g_calib_request.store(false); g_calib_done.store(0);
    for (int a = 0; a < PAD_CAL_AXES_N; ++a)
        for (int l = 0; l < PAD_CAL_LEVELS_N; ++l) g_pad_mid_n[a][l].store(PAD_CAL_MID_N_NONE);
    g_pad_probe_px_s.store(0); g_pad_probe_d.store(0); g_pad_probe_travel_px.store(0);

    std::deque<CalibSample> hist;
    const int hist_max = pad_cal_hist_frames(120);
    struct Defl { std::chrono::steady_clock::time_point t; int16_t x, y; };
    std::deque<Defl> dq;
    std::mt19937 rng(cf.seed);
    std::normal_distribution<double> nd(0.0, 1.0);
    auto level_of = [](int16_t d) {
        const int ad = std::abs((int)d);
        for (int l = 0; l < PAD_CAL_LEVELS_N; ++l)
            if (std::abs(ad - (int)pad_level_defl(PAD_CAL_LEVELS[l])) <= 1) return l;
        return 0;
    };
    auto speed = [&](int axis, int16_t d) -> double {       // px/ms
        if (!d) return 0.0;
        const int l = level_of(d);
        return (double)cf.A[axis]
             * std::pow(std::fabs((double)d) / (double)PAD_AXIS_MAX, (double)cf.p[axis])
             * (double)cf.level_scale[axis][l] * (double)cf.dir_scale[axis][d > 0 ? 0 : 1]
             / 1000.0 * (d > 0 ? 1.0 : -1.0);
    };
    // 采样质量量: 逐方向相关峰 (压低 = 该方向无纹理 → 只污染该方向)
    auto resp_of = [&](int axis, int16_t d) -> float {
        return cf.resp_dir[axis][d > 0 ? 0 : 1];
    };
    auto disp = [&](int axis, std::chrono::steady_clock::time_point a,
                    std::chrono::steady_clock::time_point b) -> double {
        double acc = 0;
        for (size_t i = 0; i + 1 < dq.size(); ++i) {
            const auto s0 = std::max(a, dq[i].t), s1 = std::min(b, dq[i + 1].t);
            if (s1 <= s0) continue;
            acc += speed(axis, axis ? dq[i].y : dq[i].x) * elapsed_ms(s1, s0);
        }
        return acc;
    };

    const auto t0 = std::chrono::steady_clock::now();
    g_padcalib_request.store(true);           // 热参触发 (等价 L3+R3 长按)
    PadLogical human;
    bool ever = false, was_collect = false;
    double next_sample_ms = 0;
    auto t_sample = t0;
    struct HSeg { int slot; int ticks; double travel; };
    std::vector<HSeg> seg_rec;
    int last_slot = -2; long seg_start_tick = 0;
    const long max_tick = (long)((double)PAD_CAL_BUDGET_MS * 3.0);
    for (long tick = 0; tick < max_tick; ++tick) {
        const double rms = (double)tick * (double)TICK_MS;
        while (elapsed_ms(std::chrono::steady_clock::now(), t0) < rms) { /* 忙等对拍 */ }
        const auto now = std::chrono::steady_clock::now();
        const PadCalibStep st = pad_calib_step(0, 120);
        if (st.active) { human = pad_excite(human, st.dx, st.dy, now); ever = true; }
        else {
            if (ever) break;
            PadLogical z; pad_excite(z, 0, 0, now);      // 标定外也保持账本连续
        }
        dq.push_back({now, st.dx, st.dy});
        while (dq.size() > 8192) dq.pop_front();          // 覆盖最长段 (600 拍) 的实时长
        const bool collecting = g_calib_collect.load();
        if (collecting && !was_collect) hist.clear();
        was_collect = collecting;
        {
            auto snap = g_pad_exc_plan.snapshot();
            int slot = -1;
            for (size_t i = 0; i < snap.size(); ++i) if (snap[i].begun) slot = (int)i;
            if (slot != last_slot) {
                if (last_slot >= 0 && last_slot < (int)snap.size()) {
                    const PadExcSeg& e = snap[(size_t)last_slot];
                    const double tr = e.pause ? 0.0
                        : std::fabs(disp(e.axis, e.t0, shift_ms(e.t1, (double)TICK_MS)));
                    seg_rec.push_back({last_slot, (int)(tick - seg_start_tick), tr});
                }
                last_slot = slot; seg_start_tick = tick;
            }
        }
        if (collecting && elapsed_ms(now, t0) >= next_sample_ms) {
            next_sample_ms += 1000.0 / (double)cf.fps;
            const double dt = std::max(1e-4, elapsed_ms(now, t_sample));
            t_sample = now;
            const float sx = (float)disp(0, shift_ms(now, -(cf.L_true + dt)),
                                         shift_ms(now, -cf.L_true)) + (float)(nd(rng) * cf.noise);
            const float sy = (float)disp(1, shift_ms(now, -(cf.L_true + dt)),
                                         shift_ms(now, -cf.L_true)) + (float)(nd(rng) * cf.noise);
            const bool gated = cf.max_disp_px > 0.0f && o.fits < cf.gate_rounds
                            && std::hypot((double)sx, (double)sy) > (double)cf.max_disp_px;
            if (!gated) {
                hist.push_back({now, (float)dt, sx, sy,
                                resp_of(0, st.dx), 2.0f});
                if ((int)hist.size() > hist_max) hist.pop_front();
                pad_calib_update_probe(hist, g_pad_exc_plan.snapshot());
            }
        }
        if (g_calib_request.exchange(false)) {          // ai_thread 的角色
            o.plan = g_pad_exc_plan.snapshot();          // 收尾序列会清空计划表 → 先抓
            o.hist_n = (int)hist.size();
            o.probe_px_s = g_pad_probe_px_s.load();

            o.res = pad_calib_fit(hist, o.plan, SYN_SHIFT_MAX);
            o.done = pad_cal_done_code(o.res);
            ++o.fits;
            if (o.done == 1) {
                g_pad_stick_gain_x.store(o.res.gain[0]);
                g_pad_stick_gain_y.store(o.res.gain[1]);
                if (!script_path.empty()) {
                    const CalibVar vars[3] = {{PAD_CAL_VAR_GAIN_X, o.res.gain[0], "%.4f"},
                                              {PAD_CAL_VAR_GAIN_Y, o.res.gain[1], "%.4f"},
                                              {PAD_CAL_VAR_L, o.res.l_est, "%.1f"}};
                    o.wrote = persist_calibration(script_path, vars, 3);
                }
            }
            g_calib_done.store(o.done);
        }
    }
    // 逐级段长 (拍 = 虚拟 ms; 取**最后一次尝试**的段) 与实测行程 (px, 合成植物真值)
    {
        const auto& pl = pad_cal_plan();
        for (size_t i = 0; i < seg_rec.size(); ++i) {
            const int slot = seg_rec[i].slot;
            if (slot < 0 || slot >= (int)pl.size() || pl[(size_t)slot].pause) continue;
            const int a = pl[(size_t)slot].axis, l = pl[(size_t)slot].level;
            const int k = (slot - (a * PAD_CAL_LEVELS_N + l) * PAD_CAL_SEGS_PER_LEVEL * 2) / 2;
            if (k >= 0 && k < PAD_CAL_SEGS_PER_LEVEL) {
                o.seg_ms[a][l][k] = seg_rec[i].ticks;                 // 取最后一次尝试
                o.travel[a][l] = seg_rec[i].travel;
                o.seg_n[a][l] = std::max(o.seg_n[a][l], k + 1);
            }
        }
    }
    if (!script_path.empty()) { std::ifstream in(script_path); std::ostringstream ss;
                                ss << in.rdbuf(); o.script = ss.str(); }
    g_aim_enabled.store(saved_aim);
    g_pad_stick_gain_x.store(saved_gx); g_pad_stick_gain_y.store(saved_gy);
    g_calib_collect.store(false); g_calib_request.store(false); g_calib_done.store(0);
    own_motion_ledger_set(false);
    return o;
}

struct SegPlay { int idx; int ticks; int axis; int level; bool pause; int16_t defl; };

static std::vector<SegPlay> drive_segments(PadCalibStep& s, int max_segs, long max_ticks,
                                           const std::function<void(int,int,bool)>& on_tick) {
    std::vector<SegPlay> out;
    long total = 0;
    int cur = -2, ticks = 0, cur_axis = -1, cur_level = 0; bool cur_pause = false;
    int16_t cur_defl = 0;
    while ((int)out.size() < max_segs && total < max_ticks) {
        auto p = g_pad_exc_plan.snapshot();
        int last = -1;
        for (size_t i = 0; i < p.size(); ++i) if (p[i].begun) last = (int)i;
        const int axis = last >= 0 ? p[(size_t)last].axis : -1;
        const int level = last >= 0 ? p[(size_t)last].level : 0;
        const bool pause = last >= 0 && p[(size_t)last].pause;
        const int16_t defl = last >= 0 ? p[(size_t)last].defl : 0;
        on_tick(axis, level, pause);              // AI 线程的角色: 刷新级采样状态
        s = pad_calib_step(0, 120);
        ++total;
        if (last != cur) {
            if (cur >= 0) out.push_back({cur, ticks, cur_axis, cur_level, cur_pause, cur_defl});
            cur = last; cur_axis = axis; cur_level = level; cur_pause = pause;
            cur_defl = defl; ticks = 0;
        }
        ++ticks;
    }
    if (cur >= 0) out.push_back({cur, ticks, cur_axis, cur_level, cur_pause, cur_defl});
    return out;
}

// ============ hid 合成链路 (run_calibration 的输入: counts 账本 + 帧样本) ============
// 指令时间线 = 激励表重复 loops 圈, 每拍 (1ms) 一段 constant counts 速率; 记账进 g_counts
//   (hid 路由), 样本按 fps 给出屏幕位移 = 该窗口内账本增量 (灵敏度恒等 1 时为真值) 加高斯
//   噪声。真 L 非整数 (如 47.3ms) 时 fit 的 lag 网格只能落在它附近 → 残余失配就是 lag
//   量化误差的口径。
struct HidSyn { float s_true = 1.0f; double L_true = 47.3; float noise = 0.1f;
                int fps = 120; unsigned seed = 3; };

static void hid_synth(const std::vector<CalibSeg>& segs, int loops, const HidSyn& hs,
                      double base_ms, std::deque<CalibSample>& hist) {
    std::vector<std::pair<int,int>> cmd;
    for (int l = 0; l < loops; ++l)
        for (const CalibSeg& sg : segs)
            for (int k = 0; k < sg.ticks; ++k) cmd.push_back({sg.dx, sg.dy});
    const int T = (int)cmd.size();
    CountsHistory& led = g_counts;
    const int keep = (int)CALIB_HIST_FRAMES * 25;          // 样本窗 (~2.5s) 覆盖的拍数
    const int t_start = std::max(0, T - keep);
    for (int k = t_start; k < T; ++k) led.add(at_ms(base_ms + k), cmd[k].first, cmd[k].second);
    const double dt = 1000.0 / (double)hs.fps;
    std::mt19937 rng(hs.seed);
    std::normal_distribution<double> nd(0.0, 1.0);
    hist.clear();
    for (double tt = base_ms + t_start; tt < base_ms + T; tt += dt) {
        auto c1 = led.at(at_ms(tt - hs.L_true));
        auto c0 = led.at(at_ms(tt - hs.L_true - dt));
        const float sx = (float)(hs.s_true * (c1.first - c0.first)) + (float)(nd(rng) * hs.noise);
        const float sy = (float)(hs.s_true * (c1.second - c0.second)) + (float)(nd(rng) * hs.noise);
        hist.push_back({at_ms(tt), (float)dt, sx, sy, 0.5f, 2.0f});
        if ((int)hist.size() > CALIB_HIST_FRAMES) hist.pop_front();
    }
}

int main() {
    using namespace std::chrono;
    g_t0 = steady_clock::now();
    auto at = [&](int k) { return at_ms(2.0 * k); };   // 2ms 步进 = 合成拍时长

    std::cout << "[1] 注入换算与合并钳制 (逐轴满偏转屏速)\n";
    {
        PadLogical h; h.rx = 100; h.lx = 100;
        PadLogical o = pad_merge(h, 1.5f, 0.0f, 3000.0f, 3000.0f, at(0));
        CHECK(o.rx == 100 + 16384 && o.lx == 100,
              "1.5px/ms @3000px/s = 半偏 (16384), 人类通道直通");
        o = pad_merge(h, 3.0f, 0.0f, 3000.0f, 3000.0f, at(1));
        CHECK(o.rx == 32767, "3px/ms @3000px/s = 满偏 (上限内)");
        o = pad_merge(h, 9.9f, 0.0f, 3000.0f, 3000.0f, at(2));
        CHECK(o.rx == 32767, "超速注入+人类通道 → +满偏钳制");
        PadLogical hn; hn.rx = -100;
        o = pad_merge(hn, -9.9f, 0.0f, 3000.0f, 3000.0f, at(3));
        CHECK(o.rx == -32767, "负向超速+人类通道 → −满偏钳制");
        o = pad_merge(h, 0.0f, 1.0f, 3000.0f, 1200.0f, at(4));
        CHECK(o.ry == (int16_t)std::lround(1.0f * 1000.0f / 1200.0f * PAD_AXIS_MAX)
              && o.rx == 100,
              "y 轴用各自的满偏转屏速换算 (1200px/s → 27306), x 轴不受扰");
        o = pad_merge(h, 0.0f, 1.0f, 3000.0f, 3000.0f, at(5));
        CHECK(o.ry == 10922, "同一注入在 3000px/s 下 = 1/3 满偏 (10922) — 逐轴 gain 生效");
        o = pad_merge(h, 1.5f, 0.0f, 6000.0f, 3000.0f, at(6));
        CHECK(o.rx == 100 + 8192, "stick_gain 加倍 → 同速度注入减半 (满偏比例反比)");
    }

    std::cout << "[1b] 行程形状 = 圆 (径向限幅, 实测驱动)\n";
    {
        PadLogical h;
        PadLogical o = pad_merge(h, 3.0f, 3.0f, 3000.0f, 3000.0f, at(0));
        CHECK(std::hypot((double)o.rx, (double)o.ry) <= 32767.0 + 1.0
              && o.rx == o.ry && std::abs(o.rx - 23170) <= 1,
              "对角超速 → 径向收到满偏 (两轴各 ≈0.71 满偏, 非逐轴满偏)");
        PadLogical hc; hc.rx = 23170; hc.ry = 23170;
        o = pad_merge(hc, 1.5f, 1.5f, 3000.0f, 3000.0f, at(1));
        CHECK(std::hypot((double)o.rx, (double)o.ry) <= 32767.0 + 1.0
              && std::abs(o.rx - o.ry) <= 1 && o.rx >= 23170,
              "圆上人类 + 同向注入 → 径向压回圆内, 方向保持 (幅度不超满偏)");
        auto l0 = g_pad_ledger.cum();
        o = pad_merge(hc, 1.5f, 1.5f, 3000.0f, 3000.0f, at(2));
        auto l1 = g_pad_ledger.cum();
        CHECK(l1.first - l0.first == (long long)o.rx * 2
              && l1.second - l0.second == (long long)o.ry * 2,
              "账本 = 径向钳制后的最终提交值 × 拍时长");
    }

    std::cout << "[2] 全透传 1:1\n";
    {
        PadLogical h; h.lx = -32767; h.ly = 12345; h.lt = 255; h.rt = 137;
        h.btns = PADBTN_A | PADBTN_START | PADBTN_DPAD_LEFT;
        PadLogical o = pad_merge(h, 0.0f, 0.0f, 3000.0f, 3000.0f, at(10));
        CHECK(o.lx == h.lx && o.ly == h.ly && o.lt == h.lt && o.rt == h.rt
              && o.btns == h.btns, "按键/左摇杆/扳机逐位直通 (扳机模拟量无阈值)");
        CHECK(o.rx == 0 && o.ry == 0, "零注入 → 右摇杆 = 人类通道 (合成数学 v=0 特例)");
    }

    std::cout << "[3] 摇杆账本 Σ(偏转·拍时长)\n";
    {
        PadLogical h; h.rx = 100;
        auto l0 = g_pad_ledger.cum();
        pad_merge(h, 1.5f, 0.0f, 3000.0f, 3000.0f, at(11));    // rx=16484, h=2ms
        pad_merge(h, 3.0f, 0.0f, 3000.0f, 3000.0f, at(12));    // rx=32767, h=2ms
        auto l1 = g_pad_ledger.cum();
        CHECK(l1.first - l0.first == (long long)(16484 + 32767) * 2
              && l1.second - l0.second == 0,
              "账本 = 合并偏转×实际拍时长 (偏转·ms), 未动轴不入账");
    }

    std::cout << "[4] 触发判据与 -k 映射\n";
    {
        bool saved_aim = g_aim_enabled.load();
        int saved_mode = g_aim_mode.load();
        g_aim_enabled.store(true);
        float vx, vy; bool g;
        g_aim_mode.store(0);
        g = control_apply_pad(120, LEFT_KEY, vx, vy);
        CHECK(g, "fire 触发 (-k fire) → 门开");
        g = control_apply_pad(120, 0, vx, vy);
        CHECK(g, "松开后 KEEP_ALIVE 窗内门仍开");
        std::this_thread::sleep_for(milliseconds(KEEP_ALIVE_MS + 80));
        g = control_apply_pad(120, 0, vx, vy);
        CHECK(!g, "保持窗过后门关");
        g = control_apply_pad(120, RIGHT_KEY, vx, vy);
        CHECK(!g, "ads 触发在 -k fire 下门不开");
        g_aim_mode.store(1);
        g = control_apply_pad(120, RIGHT_KEY, vx, vy);
        CHECK(g, "-k ads 下 LT 触发门开");
        std::this_thread::sleep_for(milliseconds(KEEP_ALIVE_MS + 80));
        g = control_apply_pad(120, LEFT_KEY, vx, vy);
        CHECK(!g, "-k ads 下 RT 不开门");
        g_aim_mode.store(2);
        g = control_apply_pad(120, LEFT_KEY, vx, vy);
        CHECK(g, "-k both 下任一触发门开");
        CHECK(vx == 0.0f && vy == 0.0f, "无有效目标时期望速度为 0 (门开≠注入)");
        g_aim_enabled.store(false);
        g = control_apply_pad(120, LEFT_KEY, vx, vy);
        CHECK(!g, "接管关闭 → 门关 (纯透传)");
        g_aim_enabled.store(saved_aim);
        g_aim_mode.store(saved_mode);
    }

    std::cout << "[5] own_motion_ledger 路由与 own_motion_scale\n";
    {
        own_motion_ledger_set(false);
        CHECK(&own_motion_ledger() == &g_counts, "hid 路由 = g_counts");
        LedgerPxScale hsc = own_motion_scale(0.3f);
        CHECK(hsc.x == 0.3f && hsc.y == 0.3f, "hid 比例 = 标定 s 单值 (两轴同值)");
        own_motion_ledger_set(true);
        CHECK(&own_motion_ledger() == &g_pad_ledger, "pad 路由 = 摇杆账本");
        const float sx = g_pad_stick_gain_x.load(), sy = g_pad_stick_gain_y.load();
        g_pad_stick_gain_x.store(3000.0f); g_pad_stick_gain_y.store(1200.0f);
        LedgerPxScale psc = own_motion_scale(0.3f);     // pad 下 hid 的 s 不入换算
        CHECK(psc.x == pad_s_rp_from_gain(3000.0f) && psc.y == pad_s_rp_from_gain(1200.0f)
              && psc.x != psc.y,
              "pad 比例 = 逐轴满偏屏速换算 (与 hid 单值区分开, 否则 Y 轴系统性偏差)");
        g_pad_stick_gain_x.store(sx); g_pad_stick_gain_y.store(sy);
        own_motion_ledger_set(false);
    }

    std::cout << "[6] 手柄未在位\n";
    {
        const std::string absent = "no_such_pad_substr_xyz";
        CHECK(find_pad_device(absent).empty(), "无匹配子串 → 空路径");
        CHECK(find_pad_device("/nonexistent_dev_node").empty(), "不存在的绝对路径 → 空路径");
        PadState pts;
        std::thread r(pad_reader_thread, absent, std::ref(pts));
        std::this_thread::sleep_for(milliseconds(300));
        CHECK(global_running.load(), "reader 未在位时不停机不阻塞");
        global_running = false;
        r.join();
        global_running = true;
    }

    std::cout << "[7] 发布点 (输出后端接入契约)\n";
    {
        PadState in;
        { std::lock_guard<std::mutex> lk(in.mtx);
          in.st.lx = -5000; in.st.ly = 7000; in.st.rx = 900;
          in.st.btns = PADBTN_B | PADBTN_DPAD_UP; }
        uint64_t s0, s1, s2;
        pad_publish_snapshot(&s0);
        pad_tick(120, in, false);
        PadLogical p = pad_publish_snapshot(&s1);
        CHECK(s1 == s0 + 1, "pad_tick → seq 单调 +1");
        CHECK(p.lx == -5000 && p.ly == 7000 && p.rx == 900 && p.btns == (PADBTN_B | PADBTN_DPAD_UP),
              "发布内容 = 合并逻辑态 (人类态 + 零注入, 逐字段)");
        pad_publish_snapshot(&s2);
        CHECK(s2 == s1, "无新拍 → seq 不变 (后端可跳过重发)");
    }

    std::cout << "[8] XInput 线格式 (20B 报告)\n";
    {
        auto i16le = [](const uint8_t* p) { return (int)(int16_t)(p[0] | (p[1] << 8)); };
        uint8_t r[PAD_XINPUT_REPORT_LEN];

        PadLogical n;
        pad_xinput_report(n, r);
        CHECK(r[0] == 0x00 && r[1] == 0x14 && r[2] == 0x00 && r[3] == 0x00,
              "中性报告前 4 字节 = 00 14 00 00 (报头 类型 0 / 长度 20 / 按键位全零)");
        bool zero_tail = true;
        for (int i = 4; i < 14; ++i) zero_tail = zero_tail && r[i] == 0;
        CHECK(zero_tail && i16le(r + 6) == 0 && i16le(r + 10) == 0,
              "中性报告: 扳机 0, 摇杆中心 0");

        PadLogical a; a.btns = PADBTN_A;
        pad_xinput_report(a, r);
        CHECK(r[3] == 0x10, "A 键 → byte3 bit4 = 0x10");
        a.btns = PADBTN_A | PADBTN_B;
        pad_xinput_report(a, r);
        CHECK(r[3] == 0x30, "A+B 组合 → byte3 = 0x30 (位叠加)");

        PadLogical d; d.btns = PADBTN_DPAD_UP | PADBTN_START | PADBTN_L3;
        pad_xinput_report(d, r);
        CHECK(r[2] == 0x51, "十字上+START+L3 → byte2 = 0x51 (bit0/4/6)");
        d.btns = PADBTN_DPAD_DOWN | PADBTN_DPAD_LEFT | PADBTN_DPAD_RIGHT
               | PADBTN_BACK | PADBTN_R3;
        pad_xinput_report(d, r);
        CHECK(r[2] == 0xAE, "十字下/左/右+BACK+R3 → byte2 = 0xAE (bit1/2/3/5/7)");
        d.btns = PADBTN_LB | PADBTN_RB | PADBTN_GUIDE | PADBTN_X | PADBTN_Y;
        pad_xinput_report(d, r);
        CHECK(r[3] == 0x07 + 0x40 + 0x80,
              "LB+RB+Guide+X+Y → byte3 = 0xC7 (保留 bit3 恒 0)");

        PadLogical t; t.lt = 255; t.rt = 137;
        pad_xinput_report(t, r);
        CHECK(r[4] == 255 && r[5] == 137, "扳机模拟量直映 (LT=255 / RT=137 原值)");

        PadLogical ax; ax.lx = 1234; ax.ly = 1234; ax.rx = -1; ax.ry = -32768;
        pad_xinput_report(ax, r);
        CHECK(r[6] == 0xD2 && r[7] == 0x04 && i16le(r + 6) == 1234,
              "LX int16 小端组装 (1234 → D2 04)");
        CHECK(r[8] == 0x2E && r[9] == 0xFB && i16le(r + 8) == -1234,
              "LY 线上上为正: 逻辑态上推 (+1234) → 线上 -1234 (口径取反, 非位反转)");
        CHECK(i16le(r + 10) == -1, "RX 符号直通 (左为负在两套口径下相同)");
        CHECK(i16le(r + 12) == 32767,
              "RY 逻辑满偏上推 → 线上 +32767 (满偏取反饱和, 不越 int16 域)");
        ax.ry = 32767;
        pad_xinput_report(ax, r);
        CHECK(i16le(r + 12) == -32767, "RY 逻辑满偏下推 → 线上 -32767 (满偏取反仍满偏)");

        bool tail0 = true;
        for (int i = 14; i < (int)PAD_XINPUT_REPORT_LEN; ++i) tail0 = tail0 && r[i] == 0;
        CHECK(tail0 && r[1] == (uint8_t)PAD_XINPUT_REPORT_LEN,
              "14–19 保留字节恒零, 长度字节 = 报告长");
    }

    std::cout << "[9] XInput 设备字节\n";
    {
        const UsbRawDeviceDef& d = pad_xinput_usb_def();
        CHECK(d.device.idVendor == 0x045E && d.device.idProduct == 0x028E
              && d.device.bcdDevice == 0x0572 && d.device.bcdUSB == 0x0200,
              "身份 = 微软有线 360 手柄 (0x045E/0x028E, bcdDevice 0x0572, USB 2.0)");
        CHECK(d.device.bDeviceClass == 0xFF && d.device.bDeviceSubClass == 0xFF
              && d.device.bDeviceProtocol == 0xFF && d.device.bMaxPacketSize0 == 64
              && d.device.bNumConfigurations == 1,
              "厂商类三元组 + ep0 64B + 单配置");
        CHECK(d.qualifier == nullptr,
              "无限定符 (全速设备) → GET_DESCRIPTOR(DEVICE_QUALIFIER) STALL");
        CHECK(d.speed == USB_SPEED_FULL, "全速枚举 (端点 bInterval 按 1ms 帧解释)");
        CHECK(d.config_len == 48 && d.config[2] == 48 && d.config[3] == 0
              && d.config[4] == 1 && d.config[5] == 1 && d.config[7] == 0x80,
              "配置节 48B 单接口单配置 / 总线供电 / wTotalLength 自洽");
        CHECK(d.config[8] == 0xFA, "bMaxPower = 0xFA → VBUS 请求 500mA");
        CHECK(d.config[18] == 0x10 && d.config[19] == 0x21 && d.config[24] == 0x81
              && d.config[25] == 0x14,
              "类特定 16B blob: bLength 0x10 / 类型 0x21 / IN 端点 0x81");
        CHECK(d.config[14] == 0xFF && d.config[15] == 0x5D && d.config[16] == 0x01
              && d.config[13] == 2,
              "接口 FF/5D/01 + 双端点 (XInput 绑定键)");
        CHECK(d.ep_in.bEndpointAddress == 0x81 && d.ep_in.wMaxPacketSize == 32
              && d.ep_in.bmAttributes == USB_ENDPOINT_XFER_INT && d.ep_in.bInterval == 4,
              "中断 IN 0x81 / 32B / bInterval=4 (=4ms @全速 → 250Hz)");
        CHECK(d.has_ep_out && d.ep_out.bEndpointAddress == 0x02
              && d.ep_out.wMaxPacketSize == 32 && d.ep_out.bInterval == 8,
              "中断 OUT 0x02 / 32B / bInterval=8 (LED/力反馈命令的落点)");
        CHECK(d.report_desc == nullptr && d.report_desc_len == 0,
              "无 HID 报告描述符 (厂商接口非 HID)");
        CHECK(d.vendor_request == nullptr,
              "不设 vendor 钩子 → xusb 初始化探测与一切 vendor 请求 STALL");
        CHECK(d.report_desc == nullptr && PAD_XINPUT_REPORT_LEN == 20
              && PAD_XINPUT_REPORT_LEN <= d.ep_in.wMaxPacketSize,
              "报告长 20B 在单包 (32B) 内: 一次 EP_WRITE = 一次传输");
        CHECK(d.string_count == 3 && !strcmp(d.strings[0].utf8, "GENERIC")
              && !strcmp(d.strings[1].utf8, "XINPUT CONTROLLER"),
              "字符串 1/2 = GENERIC / XINPUT CONTROLLER");
        {   // 序列号 = 本机身份派生 (12 位大写十六进制), 不是参考固件的公开常量
            const char* s = d.strings[2].utf8;
            bool hex12 = strlen(s) == 12;
            for (const char* p = s; hex12 && *p; ++p)
                hex12 = (*p >= '0' && *p <= '9') || (*p >= 'A' && *p <= 'F');
            CHECK(d.strings[2].index == 3 && hex12 && strcmp(s, "1.0") != 0,
                  "字符串 3 = 12 位大写十六进制 (派生自 machine-id/hostname), 非公开常量");
        }
    }

    std::cout << "[10] 激励计划表与自适应段时长 (等行程 + 闭环校正)\n";
    {
        CHECK(PAD_CAL_TRIGGER_TICKS * (double)TICK_MS == 5000.0,
              "L3+R3 长按触发 = 5000ms 墙钟 (拍数由墙钟导出)");
        const auto& pl = pad_cal_plan();
        CHECK((int)pl.size() == PAD_CAL_SEGS_N && PAD_CAL_SEGS_N == 80
              && PAD_CAL_EXCITE_N == 40,
              "计划 = 80 段 (40 激励 + 40 停顿; 2 轴 × 5 级 × 对称段序 4 段)");
        bool pat_ok = true, alt_ok = true, pause_ok = true;
        for (int ax = 0; ax < 2; ++ax)
            for (int li = 0; li < PAD_CAL_LEVELS_N; ++li)
                for (int k = 0; k < PAD_CAL_SEGS_PER_LEVEL; ++k) {
                    const int b = (ax * PAD_CAL_LEVELS_N + li) * PAD_CAL_SEGS_PER_LEVEL * 2
                                + k * 2;
                    const PadPlanSeg& e = pl[(size_t)b];        // 激励段
                    const PadPlanSeg& p = pl[(size_t)b + 1];    // 段后停顿
                    if (e.axis != ax || e.level != li || e.pause) pat_ok = false;
                    // 对称段序 [+d,−d,−d,+d]: 行程以该级起点为中心 ±A
                    const int sg = (k == 0 || k == PAD_CAL_SEGS_PER_LEVEL - 1) ? 1 : -1;
                    if (e.defl != (int16_t)(sg * pad_level_defl(PAD_CAL_LEVELS[li])))
                        alt_ok = false;
                    if (!p.pause || p.defl != 0 || p.axis != ax || p.level != li
                        || p.ticks != ms_to_ticks(PAD_CAL_PAUSE_MS)) pause_ok = false;
                    if (e.ticks != ms_to_ticks(PAD_CAL_SEG_MS_MAX)) pat_ok = false;
                }
        CHECK(pat_ok && alt_ok,
              "先 X 后 Y 逐级逐段; 级内段序 [+d,−d,−d,+d] (对称, 行程以起点为中心)");
        CHECK(pause_ok, "每段后接零偏转停顿 (150ms = L 上界 100ms + 余量)");
        CHECK((int)(sizeof(PAD_CAL_LEVELS) / sizeof(float)) == 5 && PAD_CAL_LEVELS_N == 5,
              "级集 5 级 (可用区间 {30,40,50,60,70}%, 低端避开死区, 高端外推跨度 1.43)");
        CHECK(PAD_CAL_LEVELS[0] == 0.30f && PAD_CAL_LEVELS[4] == 0.70f,
              "级集端点 = 30% / 70% (实机: 10/25% 落在摇杆死区, 满偏不测)");

        bool small = true;
        for (const CalibSeg& s : PAD_CAL_START_SEQ) if (s.dx || s.dy) small &= seg_small(s);
        for (const CalibSeg& s : PAD_CAL_END_OK_SEQ) small &= seg_small(s);
        for (const CalibSeg& s : PAD_CAL_END_FAIL_SEQ) small &= seg_small(s);
        CHECK(small && PAD_CAL_ANIM_DEFL == pad_level_defl(PAD_CAL_LEVELS[0])
              && PAD_CAL_ANIM_DEFL == 9830,
              "视觉信号 = 级集最低挡 (30%, 9830) — 不另立常量, 非满偏甩动");
        CHECK(seg_ms(PAD_CAL_START_SEQ[0]) == 240.0 && seg_ms(PAD_CAL_START_SEQ[4]) == 500.0
              && seg_ms(PAD_CAL_END_OK_SEQ[0]) == 120.0 && seg_ms(PAD_CAL_SETTLE_SEQ[0]) == 300.0,
              "段时长: 方块 240ms/边 + 500ms 停顿, 收尾 120ms/程, 静置 300ms");

        // 段长下限 = P + 帧长×(1+样本下限+抖动余量): 258ms@120fps / 367ms@60fps
        CHECK(pad_cal_seg_ms_min(120) == 258 && pad_cal_seg_ms_min(60) == 367,
              "中段统计下限 = 150 + 帧长×13 = 258ms@120fps / 367ms@60fps (8 样本 + 4 帧余量)");
        CHECK(PAD_CAL_SEG_MS_MAX == 600 && PAD_CAL_PLAN_MS == 30000
              && PAD_CAL_BUDGET_MS == 60000,
              "段长上限 600ms (覆盖采样周期 4×标称), 名义计划 30s, 流程预算 = 2×计划");
        CHECK(pad_cal_seg_floor_ms(120, false) == 258
              && pad_cal_seg_floor_ms(120, true) == 516
              && pad_cal_seg_floor_ms(60, true) == 600,
              "整轮重跑的下限 = min(2×下限, 上限) (抬高一档)");
        CHECK(pad_cal_seg_ticks(0.0f, 0.0f, 0.30f, 258) == ms_to_ticks(258),
              "探针级 (无实测) → 段长取下限 (行程最小, 最安全)");
        // 等行程: T = 目标/V̂, V̂ = v·(d_next/d_meas)^P_MAX (最陡曲线 = 高估屏速 = 安全方向)
        CHECK(pad_cal_seg_ticks(180.0f, 0.30f, 0.40f, 258) == ms_to_ticks(422),
              "等行程: 180px/s@30% → 40% 预测 320px/s → 段长 422ms (目标行程 135px)");
        CHECK(pad_cal_seg_ticks(2000.0f, 0.30f, 0.40f, 258) == ms_to_ticks(258),
              "快游戏 → 目标行程对应段长短于统计下限 → 取下限 (行程超目标, 量程门兜底)");
        CHECK(pad_cal_seg_ticks(20.0f, 0.30f, 0.40f, 258) == ms_to_ticks(600),
              "迟钝游戏 (20px/s@30% → 40% 预测 36px/s → 3704ms) → 夹到上限 600ms");
        CHECK(pad_cal_seg_ticks(180.0f, 0.30f, 0.70f, 258) == ms_to_ticks(258),
              "同轴跨度更大的一档 (30%→70%): 预测 980px/s → 138ms → 仍夹到下限 258ms");
        // 行程目标: 上界 = 投影保真圆 (2.4%@150px) 与俯仰夹紧, 下界 = 实测散度的倍数
        CHECK(PAD_CAL_TRAVEL_PX == 135.0f, "行程目标 = 屏高/8 = 135px (保真圆 150px 内)");
        bool travel_ok = true; float worst = 0;
        const int lo = pad_cal_seg_ms_min(120);
        for (float pn : {1.0f, 1.5f, 2.0f})
            for (int a = 0; a + 1 < PAD_CAL_LEVELS_N; ++a)
                for (int b = a + 1; b < PAD_CAL_LEVELS_N; ++b) {
                    const float dp = PAD_CAL_LEVELS[a], dn = PAD_CAL_LEVELS[b];
                    for (float A : {300.0f, 3000.0f, 30000.0f}) {
                        const float v_meas = A * std::pow(dp, pn);
                        const float ticks = pad_cal_seg_ticks(v_meas, dp, dn, lo);
                        const float ms = ticks * TICK_MS;
                        if (ms <= (float)lo + 0.5f) continue;      // 已夹到下限
                        const float travel = A * std::pow(dn, pn) * ms / 1000.0f;
                        worst = std::max(worst, travel);
                        if (travel > PAD_CAL_TRAVEL_PX + 1.0f) travel_ok = false;
                    }
                }
        CHECK(travel_ok, "按最陡曲线 (p≤2) 假设定的段长 → 单段行程不超目标 135px");
        std::cout << "      (最坏行程 " << std::fixed << std::setprecision(1) << worst
                  << "px / 目标 " << PAD_CAL_TRAVEL_PX << "px)\n";
        std::cout << std::defaultfloat << std::setprecision(6);   // 恢复默认格式 (后续诊断行)
    }

    std::cout << "[11] 标定触发与相位推进 (分轴分级 + 停顿 + 自适应段长)\n";
    {
        bool saved_aim = g_aim_enabled.load();
        g_aim_enabled.store(true);
        g_padcalib_request.store(false);
        const uint16_t both = PADBTN_L3 | PADBTN_R3;
        PadCalibStep s{false, 0, 0};
        auto run = [&](int n) { for (int i = 0; i < n; ++i) s = pad_calib_step(0, 120); };

        run(5);
        for (int i = 0; i < PAD_CAL_TRIGGER_TICKS - 1; ++i) s = pad_calib_step(both, 120);
        CHECK(!s.active, "L3+R3 长按差一拍 (4999 拍) → 不触发");
        s = pad_calib_step(0, 120);
        CHECK(!s.active, "中途松开 → 仍空闲");
        for (int i = 0; i < PAD_CAL_TRIGGER_TICKS - 1; ++i) s = pad_calib_step(both, 120);
        CHECK(!s.active, "松开后重新计时 (再差一拍仍不触发)");
        s = pad_calib_step(both, 120);
        CHECK(s.active && s.dx == 0 && s.dy == 0,
              "满 5000 拍 → 触发 (触发拍不播激励, 与 hid 同形)");

        const int edge = ms_to_ticks(240);
        const int lo = ms_to_ticks(pad_cal_seg_ms_min(120));
        const int16_t anim = PAD_CAL_ANIM_DEFL;
        run(1);
        CHECK(s.dx == anim && s.dy == 0, "起始十字 1/4: +30%");
        run(edge - 1); run(edge);
        CHECK(s.dx == -anim && s.dy == 0, "2/4: −30% (换到起点另一侧 → 行程居中)");
        run(edge); CHECK(s.dx == 0 && s.dy == anim, "3/4: +30% 纵向");
        run(edge); CHECK(s.dx == 0 && s.dy == -anim, "4/4: −30% 纵向");
        run(ms_to_ticks(500) - 1);
        CHECK(!g_calib_collect.load(), "起始方块期不采样 (纯视觉开始信号)");
        run(1);
        CHECK(g_calib_collect.load(), "起始方块结束 → 采样开 (激励自下一拍起)");
        CHECK(g_pad_exc_plan.snapshot().size() == (size_t)PAD_CAL_SEGS_N,
              "段窗口表已建 (80 段, 时间戳随播放写入)");

        // 30% 级: 逐段驱动 (记录实际播放时长) — 对称段序 4 段 + 每段后 150ms 停顿
        auto noop = [](int, int, bool) {};
        auto p0 = drive_segments(s, 6, 300000, noop);      // 槽位 0..5 (3 段 + 3 停顿)
        bool seq_ok = (int)p0.size() >= 6;
        for (int k = 0; seq_ok && k < 3; ++k) {
            const int16_t d = pad_level_defl(PAD_CAL_LEVELS[0]);
            const int16_t want = (int16_t)(k == 0 ? d : -d);
            seq_ok = seq_ok && p0[(size_t)(k * 2)].idx == 2 * k
                  && !p0[(size_t)(k * 2)].pause && p0[(size_t)(k * 2)].defl == want
                  && p0[(size_t)(k * 2 + 1)].idx == 2 * k + 1
                  && p0[(size_t)(k * 2 + 1)].pause
                  && std::abs(p0[(size_t)(k * 2 + 1)].ticks - ms_to_ticks(PAD_CAL_PAUSE_MS)) <= 2;
        }
        std::cout << "      (逐段: 槽位@拍数 —";
        for (size_t i = 0; i < p0.size(); ++i)
            std::cout << " " << p0[i].idx << "@" << p0[i].ticks;
        std::cout << ")\n";
        CHECK(seq_ok, "30% 级 = 对称段序 [+d,−d,−d,+d] 4 段, 每段后接 150ms 停顿");
        CHECK(p0[0].ticks >= lo - 2 && p0[0].ticks <= lo + 2,
              "探针级 (尚无实测) 段长 = 中段统计下限 258 拍");
        {
            auto snp = g_pad_exc_plan.snapshot();
            CHECK(snp[0].begun && snp[6].begun && !snp[8].begun,
                  "已播段带 begun 标志, 未触碰的槽位不带 (旧判据 t1>=t0 无法区分 — 见 [17])");
        }

        // 趁 30% 级最后一个停顿: 注入探针 (180px/s@30%) → 40% 级段长 = 135px/320px·s⁻¹
        //   = 422ms (行程闭环校正比 = 1: 上一级实测行程 = 目标)
        g_pad_probe_px_s.store(180.0f); g_pad_probe_d.store(0.30f);
        g_pad_probe_travel_px.store(PAD_CAL_TRAVEL_PX);
        for (int a = 0; a < PAD_CAL_AXES_N; ++a)
            for (int l = 0; l < PAD_CAL_LEVELS_N; ++l) g_pad_mid_n[a][l].store(16);
        auto p1 = drive_segments(s, 6, 300000, noop);       // 槽位 6..11 (进入 40% 级)
        int t8 = -1;
        for (size_t i = 0; i < p1.size(); ++i) if (p1[i].idx == 8) t8 = p1[i].ticks;
        std::cout << "      (40% 级首段 " << t8 << " 拍 (探针级 258 拍))\n";
        CHECK(t8 >= 420 && t8 <= 424,
              "级间段长按探针等行程自适应 422 拍 (不再恒为下限)");

        int n = 0;
        while (g_calib_collect.load() && n < 600000) {
            for (int a = 0; a < PAD_CAL_AXES_N; ++a)
                for (int l = 0; l < PAD_CAL_LEVELS_N; ++l) g_pad_mid_n[a][l].store(16);
            s = pad_calib_step(0, 120); ++n;
        }
        CHECK(!g_calib_collect.load() && g_calib_request.load(),
              "激励结束 → 采样关, 请求 ai_thread 拟合");
        CHECK(s.active && s.dx == 0 && s.dy == 0, "等待回执期摇杆静置 (律让位)");
        auto plan_done = g_pad_exc_plan.snapshot();
        CHECK(plan_done.size() == (size_t)PAD_CAL_SEGS_N
              && plan_done.front().t1 > plan_done.front().t0
              && plan_done.back().t1 > plan_done.front().t1,
              "段窗口逐段记录 (首尾时间戳自洽, 拟合按它归段)");
        int np = 0; for (auto& g : plan_done) np += g.pause ? 1 : 0;
        CHECK(np == PAD_CAL_SEGS_N / 2, "停顿段窗口同样记录 (40 条)");

        g_calib_request.store(false);
        g_calib_done.store(1);
        run(2);
        CHECK(s.dy == PAD_CAL_ANIM_DEFL && s.dx == 0, "回执成功 → 纵向点头收尾 (30%)");
        run(ms_to_ticks(120) * 6 + 1);
        CHECK(!s.active, "收尾结束 → 回到空闲");
        g_calib_done.store(0);

        // 失败回执 (背景不动/越界 → ai_thread 判失败): 横向摇头收尾, 不回写
        g_padcalib_request.store(true);
        s = pad_calib_step(0, 120);
        CHECK(s.active, "热参 padcalib=1 → 同样触发");
        CHECK(!g_padcalib_request.load(), "标定请求一次消费即清 (exchange)");
        run(edge * 4 + ms_to_ticks(500) + 1);
        n = 0;
        while (g_calib_collect.load() && n < 600000) {
            for (int a = 0; a < PAD_CAL_AXES_N; ++a)
                for (int l = 0; l < PAD_CAL_LEVELS_N; ++l) g_pad_mid_n[a][l].store(16);
            s = pad_calib_step(0, 120); ++n;
        }
        g_calib_request.store(false);
        g_calib_done.store(2);
        run(2);
        CHECK(s.dx == PAD_CAL_ANIM_DEFL && s.dy == 0, "回执失败 → 横向摇头收尾 (30%)");
        run(ms_to_ticks(120) * 6 + 1);
        CHECK(!s.active, "失败收尾后回到空闲 (可重复触发)");
        g_calib_done.store(0);

        for (int i = 0; i < PAD_CAL_TRIGGER_TICKS + 5; ++i) s = pad_calib_step(PADBTN_L3, 120);
        CHECK(!s.active, "只按 L3 (满 5s) 不触发");

        g_padcalib_request.store(true);
        s = pad_calib_step(0, 120);
        g_padcalib_request.store(false);
        CHECK(s.active, "重新触发 (为复位用例)");
        g_aim_enabled.store(false);
        s = pad_calib_step(0, 120);
        CHECK(!s.active, "接管关闭 → 进行中的标定复位 (标定不可达)");
        g_padcalib_request.store(true);
        s = pad_calib_step(0, 120);
        CHECK(!s.active && !g_padcalib_request.load(), "接管关闭 → 请求被忽略且消费");
        g_aim_enabled.store(true);
        for (int i = 0; i < PAD_CAL_TRIGGER_TICKS + 5; ++i)
            s = pad_calib_step(PADBTN_L3 | PADBTN_R3, 120);
        CHECK(s.active, "重新开启接管 → 长按可再次触发 (状态机未卡死)");
        g_aim_enabled.store(false);
        pad_calib_step(0, 120);                       // 复位
        g_aim_enabled.store(saved_aim);
        g_calib_collect.store(false); g_calib_request.store(false); g_calib_done.store(0);
    }

    std::cout << "[11b] 取样不足 → 重播阶梯 (级内加倍 → 上限) 与整轮重跑 (仅一次)\n";
    {
        bool saved_aim = g_aim_enabled.load();
        g_aim_enabled.store(true);
        PadCalibStep s{false, 0, 0};
        for (int a = 0; a < PAD_CAL_AXES_N; ++a)
            for (int l = 0; l < PAD_CAL_LEVELS_N; ++l)
                g_pad_mid_n[a][l].store(PAD_CAL_MID_N_NONE);
        g_padcalib_request.store(true);
        s = pad_calib_step(0, 120);
        g_padcalib_request.store(false);
        for (int i = 0; i < ms_to_ticks(240) * 4 + ms_to_ticks(500) + 1; ++i)
            s = pad_calib_step(0, 120);
        CHECK(g_calib_collect.load(), "进入激励期 (热参触发 + 起始十字)");

        // 30% 级全程报"中段样本 3 < 8" (仅样本数不足) → 级边界连续重播到上限
        auto on_tick = [](int axis, int level, bool pause) {
            if (!pause && axis == 0 && level == 0) g_pad_mid_n[0][0].store(3);
        };
        const int L8 = 2 * PAD_CAL_SEGS_PER_LEVEL;          // 一级 = 8 个计划段
        auto played = drive_segments(s, 3 * L8, 900000, on_tick);
        std::cout << "      (激励段 计划序号@拍数 —";
        for (int i = 0; i < (int)played.size(); ++i)
            if (!played[(size_t)i].pause)
                std::cout << " " << played[(size_t)i].idx << "@" << played[(size_t)i].ticks;
        std::cout << ")\n";
        const int lo_ms = pad_cal_seg_ms_min(120);
        CHECK((int)played.size() >= 3 * L8 - 1 && played[2 * L8].idx == 0,
              "取到 3 次尝试的逐段记录 (第 3 次尝试已开播)");
        CHECK(played[0].idx == 0 && played[0].ticks >= lo_ms - 2 && played[0].ticks <= lo_ms + 2,
              "首次尝试: 30% 级首段取下限 258 拍 (探针级: 无实测)");
        CHECK(played[L8].idx == 0 && played[L8].ticks >= 2 * lo_ms - 4,
              "第 1 次重播: 回退到该级首段, 段长加倍 (258→516)");
        CHECK(played[2 * L8].idx == 0 && played[2 * L8].ticks >= PAD_CAL_SEG_MS_MAX - 2,
              "第 2 次重播: 再加倍到上限 600ms");
        CHECK(played[L8 + 2].idx == 2 && !played[L8 + 2].pause,
              "重播重走该级的对称段序 (计划槽位复用, 非新增槽位)");
        CHECK(played[L8 - 1].pause && played[L8 - 1].idx == 2 * PAD_CAL_SEGS_PER_LEVEL - 1,
              "首次尝试走完 4 段 + 4 停顿后才到级边界");

        // 上限后不再重播 → 正常进入下一挡
        auto rest = drive_segments(s, 4, 900000, on_tick);
        bool again = false, nxt = false;
        for (size_t i = 0; i < rest.size(); ++i) {
            if (rest[i].idx == 0) again = true;
            if (!rest[i].pause && rest[i].axis == 0 && rest[i].level == 1) nxt = true;
        }
        CHECK(!again && nxt, "阶梯到顶后不再重播 → 正常进入下一挡 (40%)");

        int n = 0;
        while (g_calib_collect.load() && n < 900000) { s = pad_calib_step(0, 120); ++n; }
        CHECK(g_calib_request.load(), "整轮结束 → 请求拟合");
        g_calib_request.store(false);

        // 回执 3 (仅样本数不足) → 整轮重跑: 下限抬高一档 (258→516ms), 计划表重建
        g_calib_done.store(3);
        s = pad_calib_step(0, 120);
        CHECK(s.active && g_calib_collect.load() && g_calib_done.load() == 0,
              "回执 3 → 整轮重跑: 采样重开, 回执码清零");
        {
            auto pz = g_pad_exc_plan.snapshot();
            bool any = false; for (size_t i = 0; i < pz.size(); ++i) any |= pz[i].begun;
            CHECK(!any, "重跑重建窗口表 (首轮尝试的时间戳全部作废)");
        }
        auto p2 = drive_segments(s, 1, 900000, on_tick);
        CHECK(!p2.empty() && p2[0].idx == 0 && p2[0].ticks >= 2 * lo_ms - 4,
              "重跑的首段 = 抬高一档的下限 (516ms) — 整轮下限整体上移");

        // 第二次回执 3: 已重跑过 → 不再重跑, 转失败收尾 (不写垃圾值)
        n = 0;
        while (g_calib_collect.load() && n < 900000) { s = pad_calib_step(0, 120); ++n; }
        g_calib_request.store(false);
        g_calib_done.store(3);
        s = pad_calib_step(0, 120);
        CHECK(!s.active || s.dx == 0, "第二次回执 3 当拍: 尚未进入收尾序列");
        s = pad_calib_step(0, 120);
        CHECK(s.active && !g_calib_collect.load() && s.dx == PAD_CAL_ANIM_DEFL && s.dy == 0,
              "第二次回执 3 → 不再重跑 (仅一次), 转失败摇头收尾");
        for (int i = 0; i < ms_to_ticks(120) * 6 + 2; ++i) s = pad_calib_step(0, 120);
        CHECK(!s.active, "收尾结束 → 空闲 (失败路径可重复触发)");
        g_calib_collect.store(false); g_calib_request.store(false); g_calib_done.store(0);
        g_aim_enabled.store(saved_aim);
    }

    std::cout << "[12] 激励期独占右摇杆与账本\n";
    {
        auto af = [&](int k) { return at_ms(60000.0 + 2.0 * k); };
        PadLogical warm;
        pad_merge(warm, 0.0f, 0.0f, PAD_STICK_GAIN_DEFAULT, PAD_STICK_GAIN_DEFAULT, af(0));
        auto l0 = g_pad_ledger.cum();
        PadLogical h; h.rx = 9999; h.ry = -7777; h.lx = -123; h.lt = 200; h.rt = 31;
        h.btns = PADBTN_A | PADBTN_DPAD_LEFT;
        PadLogical o = pad_excite(h, 16384, -22937, af(1));
        CHECK(o.rx == 16384 && o.ry == -22937,
              "激励期右摇杆 = 激励偏转 (人类右摇杆被忽略)");
        CHECK(o.lx == h.lx && o.lt == h.lt && o.rt == h.rt && o.btns == h.btns,
              "激励期其余字段逐位直通人类态");
        auto l1 = g_pad_ledger.cum();
        CHECK(l1.first - l0.first == 16384LL * 2 && l1.second - l0.second == -22937LL * 2,
              "激励按偏转×拍时长入账 (标定拟合的数据源)");
    }

    std::cout << "[13] 量纲换算与设计带\n";
    {
        const float g = PAD_STICK_GAIN_DEFAULT, s_rp = pad_s_rp_from_gain(g);
        CHECK(std::abs(s_rp * PAD_AXIS_MAX * 1000.0f - g) < 1e-3f,
              "s_rp × 32767 × 1000 = 满偏转屏速 px/s");
        CHECK(std::abs(pad_gain_from_s_rp(s_rp) - g) < 1e-3f, "逆换算往返一致");
        CHECK(std::abs(s_rp * PAD_AXIS_MAX * TICK_MS - g * TICK_MS / 1000.0f) < 1e-3f,
              "满偏一拍的屏移 = 满偏屏速 × 拍长 (账本单位的物理含义)");
        CHECK(pad_gain_clamp(1.0f) == PAD_GAIN_MIN && pad_gain_clamp(1e9f) == PAD_GAIN_MAX
              && pad_gain_clamp(g) == g, "设计带钳制 (带内不动)");
        CHECK(pad_calib_accept(g) && !pad_calib_accept(PAD_GAIN_MIN)
              && !pad_calib_accept(PAD_GAIN_MAX) && !pad_calib_accept(0.0f),
              "逐轴带内判定: 落带边 (= 钳制咬合) 与带外 → 不接受");
        CHECK(pad_cal_shift_max_px(106) > 70.0f && pad_cal_shift_max_px(106) < 71.0f,
              "可靠每帧位移上界 = 半分辨率块宽 1/3 ×2 = 70.7px (块 106px)");
    }

    std::cout << "[14] 拟合: 停顿法复原两轴幂律与延迟\n";
    {
        own_motion_ledger_set(true);
        std::deque<CalibSample> hist;
        double base = 120000.0;

        // (a) 两轴曲线不同 (Y 为加速曲线 p=1.5) + 零噪声: 逐位复原, 无串扰
        {
            Synth sy;
            sy.A[0] = 3000.0f; sy.p[0] = 1.0f;
            sy.A[1] = 1200.0f; sy.p[1] = 1.5f;
            auto win = synth_play(sy, base, hist);
            base += 20000.0;
            PadCalibResult r = pad_calib_fit(hist, win, SYN_SHIFT_MAX);
            for (int ax = 0; ax < PAD_CAL_AXES_N; ++ax) {
                std::cout << "      [lv " << (ax ? "Y" : "X") << "] ";
                for (int li = 0; li < PAD_CAL_LEVELS_N; ++li)
                    std::cout << (int)(r.lv[ax][li].d * 100 + .5f) << "%:"
                              << (r.lv[ax][li].valid ? "ok" : "NO")
                              << "(" << r.lv[ax][li].why << ") ";
                std::cout << "\n";
            }
            std::cout << "      (X A=" << r.gain[0] << " p=" << r.p[0]
                      << " | Y A=" << r.gain[1] << " p=" << r.p[1]
                      << " | L=" << r.l_est << "ms edge " << r.l_edge_ms
                      << "ms | sigma " << r.sigma[0] << "/" << r.sigma[1]
                      << " | nlv " << r.nlv[0] << "/" << r.nlv[1] << ")\n";
            // ± 一致性门是 3σ 统计门 (每级虚弃率 ~0.3%), 单次实现里允许丢 1 级 —
            //   丢了也照常降级拟合, 下面的复原断言才是实质
            CHECK(r.ok && r.nlv[0] >= PAD_CAL_LEVELS_N - 1 && r.nlv[1] >= PAD_CAL_LEVELS_N - 1,
                  "两轴各 ≥3 级有效 (3σ 一致性门允许噪声丢 1 级)");
            CHECK(near(r.gain[0], 3000.0f, 5e-3f) && near(r.p[0], 1.0f, 1e-2f),
                  "X 轴复原 A=3000px/s p=1 (线性)");
            CHECK(near(r.gain[1], 1200.0f, 5e-3f) && near(r.p[1], 1.5f, 1e-2f),
                  "Y 轴复原 A=1200px/s p=1.5 (加速曲线) — 两轴互不串扰");
            CHECK(std::fabs(r.l_est - 50.0f) <= 4.0f,
                  "延迟复原 50ms (停顿位移和, 亚帧精度)");
            CHECK(std::fabs(r.l_edge_ms - 50.0f) <= 12.0f,
                  "边沿读数与之一致 (增益无关的独立交叉检查)");
            CHECK(pad_calib_accept(r.gain[0]) && pad_calib_accept(r.gain[1]), "复原值落设计带内");
        }
        // (b) 噪声 0.3px: 容差放宽但仍精确
        {
            Synth sy;
            sy.A[0] = 6000.0f; sy.p[0] = 1.2f;
            sy.A[1] = 2600.0f; sy.p[1] = 0.9f;
            sy.noise = 0.3f; sy.L_true = 42.0;
            auto win = synth_play(sy, base, hist);
            base += 20000.0;
            PadCalibResult r = pad_calib_fit(hist, win, SYN_SHIFT_MAX);
            std::cout << "      (含噪 0.3px: X A=" << r.gain[0] << " p=" << r.p[0]
                      << " | Y A=" << r.gain[1] << " p=" << r.p[1]
                      << " | L=" << r.l_est << "ms n=" << r.l_n
                      << " | 边沿 " << r.l_edge_ms << "ms n=" << r.l_edge_n
                      << " | sigma " << r.sigma[0] << "/" << r.sigma[1]
                      << " | nlv " << r.nlv[0] << "/" << r.nlv[1] << ")\n";
            CHECK(near(r.gain[0], 6000.0f, 2e-2f) && near(r.gain[1], 2600.0f, 2e-2f),
                  "含噪 0.3px: 两轴满偏屏速仍在 2% 内");
            CHECK(std::fabs(r.p[0] - 1.2f) < 0.06f && std::fabs(r.p[1] - 0.9f) < 0.06f,
                  "含噪: 幂律指数在 0.06 内");
            CHECK(std::fabs(r.l_est - 42.0f) <= 6.0f, "含噪: 延迟在 6ms 内");
            CHECK(std::fabs(r.sigma[0] - 0.3f) < 0.06f && std::fabs(r.sigma[1] - 0.3f) < 0.06f,
                  "停顿样本估出的噪声底 ≈ 注入的 0.3px (激励充分性判据的尺度)");
        }
        // (c) 动机用例: T 与 L 同量级 (T=250ms, L=100ms = 设计保证上界) —
        //     中段取样无偏, 而"段内直接平均"被段首瞬态系统性低估
        {
            Synth sy;
            sy.A[0] = 3000.0f; sy.p[0] = 1.0f;
            sy.A[1] = 1500.0f; sy.p[1] = 1.0f;
            sy.L_true = 100.0;
            sy.seg_ms = 250;
            auto win = synth_play(sy, base, hist);
            base += 20000.0;
            PadCalibResult r = pad_calib_fit(hist, win, SYN_SHIFT_MAX);
            double g_naive = 0, g_true = true_g(sy, 0, 0.25f);
            int cnt = 0;
            for (auto& sg : win)
                if (!sg.pause && sg.axis == 0 && sg.level == 1) {
                    g_naive += naive_gain(hist, sg, 1000.0 / 120.0); ++cnt; }
            g_naive = cnt ? g_naive / cnt : 0;
            std::cout << "      (L=100ms, T=250ms: 停顿法 A=" << r.gain[0]
                      << " p=" << r.p[0] << " | 段内直接平均 g/g_true=" << (g_naive / g_true)
                      << " | L=" << r.l_est << "ms | nlv " << r.nlv[0] << "/" << r.nlv[1] << ")\n";
            CHECK(near(r.gain[0], 3000.0f, 1e-3f) && near(r.p[0], 1.0f, 1e-3f),
                  "T 与 L 同量级: 中段取样增益仍无偏 (A 复原)");
            CHECK(std::fabs(r.l_est - 100.0f) <= 4.0f, "T 与 L 同量级: 延迟仍复原 (100ms)");
            CHECK(g_naive / g_true > 0.30 && g_naive / g_true < 0.80,
                  "对照: 段内直接平均把段首瞬态算进均值 → 增益被系统性低估 (≈1−L/T, 非垃圾值)");
        }
        // (d) L > P: 停顿留不住瞬态 → 自校验失败 (不硬算)
        {
            Synth sy;
            sy.A[0] = 3000.0f; sy.A[1] = 3000.0f;
            sy.L_true = 170.0;                       // > P = 150ms
            auto win = synth_play(sy, base, hist);
            base += 20000.0;
            PadCalibResult r = pad_calib_fit(hist, win, SYN_SHIFT_MAX);
            std::cout << "      (L=170ms > P: err=\"" << r.err << "\", L=" << r.l_est
                      << "ms 边沿 " << r.l_edge_ms << "ms)\n";
            CHECK(!r.ok && r.err[0] != 0,
                  std::string("L > P 的自校验: 标定失败且原因可读: ").append(r.err).c_str());
        }
        own_motion_ledger_set(false);
    }

    std::cout << "[15] 级有效性判定与降级\n";
    {
        own_motion_ledger_set(true);
        std::deque<CalibSample> hist;
        double base = 260000.0;

        // (a) 60% 级超量程 (位移放大模拟回卷/游戏太快) → 整级丢弃, 其余级照常拟合
        {
            Synth sy;
            sy.A[0] = 3000.0f; sy.p[0] = 1.3f;
            sy.A[1] = 1200.0f; sy.p[1] = 1.0f;
            sy.scale[0][3] = 6.0f;
            auto win = synth_play(sy, base, hist);
            base += 20000.0;
            PadCalibResult r = pad_calib_fit(hist, win, SYN_SHIFT_MAX);
            const PadCalLevelDiag& d = r.lv[0][3];
            CHECK(!d.valid && d.why[0] != 0 && d.val > d.lim,
                  std::string("60% 级因超量程整级丢弃, 原因可读: ").append(d.why).c_str());
            CHECK(r.ok && r.nlv[0] == 4 && r.nlv[1] == 5,
                  "X 取其余 4 级 (60% 被丢), Y 仍 5 级 — 各自有效级数入日志");
            CHECK(near(r.gain[0], 3000.0f, 1e-3f) && near(r.p[0], 1.3f, 1e-3f),
                  "丢掉 60% 级后仍复原 X 的 A/p (幂律由其余级定出)");
            CHECK(near(r.gain[1], 1200.0f, 1e-3f), "Y 轴不受 X 级丢弃影响");
        }
        // (b) 块间离散超量程 (回卷的另一种证据) → 整级丢弃
        {
            Synth sy;
            sy.A[0] = 2000.0f; sy.p[0] = 1.0f;
            sy.A[1] = 2000.0f;
            sy.spread[0][1] = SYN_SHIFT_MAX + 20.0f;
            auto win = synth_play(sy, base, hist);
            base += 20000.0;
            PadCalibResult r = pad_calib_fit(hist, win, SYN_SHIFT_MAX);
            CHECK(!r.lv[0][1].valid && r.nlv[0] == 4 && r.ok,
                  "块间离散越界 → 该级丢弃 (非静默取值), 其余四级仍拟合");
            CHECK(near(r.gain[0], 2000.0f, 1e-3f), "丢弃后 A 仍复原");
        }
        // (c) 夹紧 (屏幕不响应注入): 中段位移≈0 → 整级丢弃
        {
            Synth sy;
            sy.A[0] = 3000.0f; sy.p[0] = 1.0f;
            sy.A[1] = 3000.0f;
            sy.scale[0][2] = 0.0f;                   // X 50% 级夹紧
            sy.scale[1][3] = 0.0f;                   // Y 70% 级夹紧
            auto win = synth_play(sy, base, hist);
            base += 20000.0;
            PadCalibResult r = pad_calib_fit(hist, win, SYN_SHIFT_MAX);
            CHECK(!r.lv[0][2].valid && !r.lv[1][3].valid && r.ok,
                  "夹紧级 (中段位移≈0) 整级丢弃, 其余级照常");
            CHECK(near(r.gain[0], 3000.0f, 1e-3f) && near(r.gain[1], 3000.0f, 1e-3f),
                  "各轴失一级仍复原 A=3000px/s (逐轴独立)");
        }
        // (d) 响应过低 → 该级丢弃
        {
            Synth sy;
            sy.A[0] = 3000.0f; sy.p[0] = 1.0f;
            sy.A[1] = 3000.0f;
            sy.lowresp[0][0] = true;
            auto win = synth_play(sy, base, hist);
            base += 20000.0;
            PadCalibResult r = pad_calib_fit(hist, win, SYN_SHIFT_MAX);
            CHECK(!r.lv[0][0].valid && r.nlv[0] <= 4 && r.ok,
                  "相关响应过低 → 该级丢弃 (其余级拟合)");
        }
        // (e) 恰好 1 级有效 → 线性回退 (p=1, A = 该级屏速/d), 线性植物下精确
        {
            Synth sy;
            sy.A[0] = 3000.0f; sy.p[0] = 1.0f;
            sy.A[1] = 1500.0f; sy.p[1] = 1.0f;
            for (int l = 0; l < PAD_CAL_LEVELS_N; ++l)
                if (l != 1) { sy.scale[0][l] = 0.0f; sy.scale[1][l] = 0.0f; }
            auto win = synth_play(sy, base, hist);
            base += 20000.0;
            PadCalibResult r = pad_calib_fit(hist, win, SYN_SHIFT_MAX);
            CHECK(r.ok && r.nlv[0] == 1 && r.nlv[1] == 1 && r.linear[0] && r.linear[1],
                  "只剩 25% 级有效 → 单级线性回退 (回退旗标入日志)");
            CHECK(r.p[0] == 1.0f && near(r.gain[0], 3000.0f, 1e-3f)
                  && near(r.gain[1], 1500.0f, 1e-3f),
                  "线性回退: p=1 且 A = 该级屏速/d (线性植物下精确)");
        }
        // (f) 全部级丢弃 → 失败 (不得静默给错值)
        {
            Synth sy;
            for (int a = 0; a < 2; ++a) for (int l = 0; l < PAD_CAL_LEVELS_N; ++l)
                sy.scale[a][l] = 0.0f;
            auto win = synth_play(sy, base, hist);
            base += 20000.0;
            PadCalibResult r = pad_calib_fit(hist, win, SYN_SHIFT_MAX);
            CHECK(!r.ok && r.nlv[0] == 0 && r.nlv[1] == 0,
                  "全部级无效 (背景不动/无游戏) → 失败收尾 (不写值)");
        }
        own_motion_ledger_set(false);
    }

    std::cout << "[16] 标定回写 (VAR 名与条数参数化)\n";
    {
        char tmpl[] = "/tmp/pad_test_persist_XXXXXX";
        char* dir = mkdtemp(tmpl);
        CHECK(dir != nullptr, "临时目录");
        if (dir) {
            const std::string path = std::string(dir) + "/game.sh";
            { std::ofstream o(path);
              o << "#!/bin/bash\nS_EST=9.9900  # 占位\nL_EST=9.9\n"
                   "PAD_STICK_GAIN_X=1.0000  # 占位\nPAD_STICK_GAIN_Y=1.0000\n"
                   "L_EST_PAD=9.9\nOUTPUT_MODE=\"pad\"\n"; }
            chmod(path.c_str(), 0750);
            const CalibVar hid[2] = {{"S_EST", 1.25f, "%.4f"}, {"L_EST", 61.5f, "%.1f"}};
            CHECK(persist_calibration(path, hid, 2), "hid 回写 (S_EST/L_EST) 成功");
            std::string txt = read_file(path);
            CHECK(has_line(txt, "S_EST=1.2500") && has_line(txt, "L_EST=61.5"),
                  "hid 值写入: %.4f 与 %.1f 格式");
            CHECK(has_line(txt, "PAD_STICK_GAIN_X=1.0000") && has_line(txt, "PAD_STICK_GAIN_Y=1.0000")
                  && has_line(txt, "L_EST_PAD=9.9") && has_line(txt, "OUTPUT_MODE=\"pad\""),
                  "hid 回写不触碰 pad 的 VAR (两套互不覆盖)");
            CHECK(txt.find("# 占位") != std::string::npos, "行内注释保留");
            const CalibVar pad[3] = {{PAD_CAL_VAR_GAIN_X, 3041.25f, "%.4f"},
                                     {PAD_CAL_VAR_GAIN_Y, 1198.5f, "%.4f"},
                                     {PAD_CAL_VAR_L, 58.5f, "%.1f"}};
            CHECK(persist_calibration(path, pad, 3), "pad 回写 (双轴增益 + L_EST_PAD) 成功");
            txt = read_file(path);
            CHECK(has_line(txt, "PAD_STICK_GAIN_X=3041.2500")
                  && has_line(txt, "PAD_STICK_GAIN_Y=1198.5000")
                  && has_line(txt, "L_EST_PAD=58.5"), "pad 双轴与延迟写入");
            CHECK(has_line(txt, "S_EST=1.2500") && has_line(txt, "L_EST=61.5"),
                  "pad 回写不触碰 hid 的 VAR (回归)");
            struct stat st{};
            CHECK(stat(path.c_str(), &st) == 0 && (st.st_mode & 07777) == 0750,
                  "原文件权限 (0750) 保留");
            CHECK(access((path + ".tmp").c_str(), F_OK) != 0, "无 .tmp 残留 (原子替换)");

            const std::string path2 = std::string(dir) + "/old.sh";
            { std::ofstream o(path2);
              o << "#!/bin/bash\nS_EST=9.9900\nL_EST=9.9\n"; }
            CHECK(persist_calibration(path2, pad, 3), "缺行脚本回写成功");
            txt = read_file(path2);
            CHECK(has_line(txt, "PAD_STICK_GAIN_X=3041.2500") && has_line(txt, "L_EST_PAD=58.5")
                  && has_line(txt, "S_EST=9.9900"),
                  "缺失的 VAR 追加写入, 既有行不动");
            CHECK(!persist_calibration(std::string(dir) + "/no_such.sh", hid, 2),
                  "不存在的脚本 → 回写失败 (不创建文件)");
            unlink(path.c_str()); unlink(path2.c_str()); rmdir(dir);
        }
    }


    std::cout << "[17] 采样状态: 探针判据与旧的哨兵缺陷 (实机根因复核)\n";
    {
        own_motion_ledger_set(true);
        std::deque<CalibSample> hist;
        double base = 500000.0;
        Synth sy; sy.A[0] = 3000.0f; sy.p[0] = 1.0f;
        sy.A[1] = 3000.0f; sy.p[1] = 1.0f; sy.L_true = 45.0; sy.seg_ms = 258;
        auto win = synth_play(sy, base, hist);
        // 只把前 14 段标成"已开播" (X 30% 级已播完, 40% 级进行中)
        for (size_t i = 14; i < win.size(); ++i) win[i].begun = false;
        int old_last = -1;                       // 旧判据: t1 >= t0 (未被触碰的槽位也满足)
        for (size_t i = 0; i < win.size(); ++i) if (win[i].t1 >= win[i].t0) old_last = (int)i;
        int new_last = -1;                       // 新判据: begun
        for (size_t i = 0; i < win.size(); ++i) if (win[i].begun) new_last = (int)i;
        CHECK(old_last == (int)win.size() - 1 && new_last == 13,
              "旧哨兵 (t1>=t0) 命中未触碰的最后槽位; 新判据 (begun) 命中真正已播段");
        // 旧代码的等价状态: 未触碰槽位 = 默认时间戳 (t0==t1), 但被旧判据当成"已播" —
        //   探针于是永远池化最后一个槽位所属的级 (Y 70%), 其窗口在激励早期全空 → cc=0
        //   → 探针永不发布 → 段长恒取下限 → 级仍无效 (实机"每级都标 (探针级)"的死循环)
        std::vector<PadExcSeg> oldwin = win;
        for (size_t i = 0; i < oldwin.size(); ++i) {
            oldwin[i].begun = true;
            if (i >= 14) { oldwin[i].t0 = std::chrono::steady_clock::time_point{};
                           oldwin[i].t1 = oldwin[i].t0; }
        }
        g_pad_probe_px_s.store(0.0f); g_pad_probe_d.store(0.0f);
        pad_calib_update_probe(hist, oldwin);
        CHECK(g_pad_probe_px_s.load() == 0.0f, "旧判据下探针恒 0 (窗口全空的自强化死循环)");
        pad_calib_update_probe(hist, win);
        std::cout << "      (探针 " << g_pad_probe_px_s.load() << "px/s @ d="
                  << g_pad_probe_d.load() << " | 行程读数 "
                  << g_pad_probe_travel_px.load() << "px | 级采样状态 (0,2)="
                  << g_pad_mid_n[0][2].load() << ")\n";
        CHECK(g_pad_probe_px_s.load() > 0.0f && near(g_pad_probe_d.load(), 0.4f, 1e-3f),
              "新判据下探针可用 (最后已播段所属级 = 40%)");
        CHECK(near(g_pad_probe_px_s.load(), 3000.0f * 0.4f, 0.05f),
              "探针屏速 ≈ 该级真值 (宽松判据 2σ + 量级钳制)");
        CHECK(g_pad_mid_n[0][1].load() >= PAD_CAL_SEG_MIN_N,
              "级采样状态 ≥ 样本下限 (重播裁决的输入)");
        // 行程读数 = 该级整段位移积分 = v·(段长−L) (合成链路用固定 258ms 段长,
        //   不是自适应段长 → 此处只校验口径, 不是目标行程)
        const double trav_expect = 3000.0 * 0.4 * (258.0 - 45.0) / 1000.0;
        std::cout << "      (行程读数 " << g_pad_probe_travel_px.load() << "px, 口径预期 "
                  << trav_expect << "px = v·(T−L))\n";
        CHECK(near(g_pad_probe_travel_px.load(), (float)trav_expect, 0.15f),
              "实测行程读数 = v·(段长−L) (闭环校正的输入口径)");
        // ---- 旧设计点的复现 (根因 1: 中段窗零余量) ----
        // 旧下限 = P + 帧长×(1+4) = 192ms → 中段窗 = T−P−帧长 = 33.7ms ≈ 4.04 帧
        //   (标称 120fps)。实测采样周期只要略慢于标称 (处理线程跟不上采集), 样本数
        //   就掉到 3 < 4 → **每级**"中段样本不足" → 全级无效; 探针又因哨兵缺陷恒 0 →
        //   段长恒取下限 → 自我强化 (实机日志"每级都标 (探针级)"+"全部级无效")。
        // 新下限 = P + 帧长×13 = 258ms → 同一采样周期下有 10 个样本 (≥8)。
        {
            const double f110 = 1000.0 / 110.0;      // 实测采样周期 (略慢于标称 120fps)
            const int n192 = (int)((192.0 - PAD_CAL_PAUSE_MS - f110) / f110) + 1;
            const int n258 = (int)((258.0 - PAD_CAL_PAUSE_MS - f110) / f110) + 1;
            std::cout << "      (中段样本 @110fps: 旧下限 192ms → " << n192
                      << " 个 (旧判据需 ≥4) | 新下限 258ms → " << n258
                      << " 个 (需 ≥8))"<< std::endl;
            CHECK(n192 <= 4 && n258 >= PAD_CAL_SEG_MIN_N + 2,
                  "旧下限的窗恰卡在旧判据门限上 (3–4 个 = 零余量), 新下限 10–11 个有真实余量");
            Synth old_sy; old_sy.fps = 110; old_sy.A[0] = old_sy.A[1] = 3000.0f;
            old_sy.seg_ms = 192;
            auto w192 = synth_play(old_sy, base, hist); base += 40000.0;
            PadCalibResult r192 = pad_calib_fit(hist, w192, SYN_SHIFT_MAX);
            bool all_short = (r192.nlv[0] == 0 && r192.nlv[1] == 0);
            for (int l = 0; l < PAD_CAL_LEVELS_N && all_short; ++l)
                all_short = !r192.lv[0][l].valid && r192.lv[0][l].why_samples;
            CHECK(all_short, "旧设计点复现: 段长 192ms → 两级全无效, 原因全是中段样本不足");
            old_sy.seg_ms = 258;
            auto w258 = synth_play(old_sy, base, hist); base += 40000.0;
            PadCalibResult r258 = pad_calib_fit(hist, w258, SYN_SHIFT_MAX);
            CHECK(r258.ok && r258.nlv[0] >= PAD_CAL_LEVELS_N - 1 && r258.nlv[1] >= PAD_CAL_LEVELS_N - 1
                  && near(r258.gain[0], 3000.0f, 0.03f),
                  "同一采样周期下新下限 258ms → 各级有效且 A 复原 (余量起作用)");
        }
        own_motion_ledger_set(false);
    }

    std::cout << "[18] 闭环合成 e2e: 触发→激励→停顿→拟合→回写\n";
    {
        char tmpl[] = "/tmp/pad_test_loop_XXXXXX";
        char* dir = mkdtemp(tmpl);
        const std::string script = dir ? std::string(dir) + "/game.sh" : std::string();
        if (!script.empty()) {
            std::ofstream o(script);
            o << "#!/bin/bash\nPAD_STICK_GAIN_X=0.0000\nPAD_STICK_GAIN_Y=0.0000\nL_EST_PAD=0.0\n";
        }
        // 中速游戏: 30% 级探针 → 40%/50% 级段长按等行程拉长, 高挡位再落回下限 —
        //   一次真实节拍的闭环同时验证探针、自适应段长、行程闭环校正与拟合复原
        LoopCfg mid; mid.A[0] = 800.0f; mid.A[1] = 800.0f;
        mid.p[0] = mid.p[1] = 1.0f; mid.noise = 0.02f; mid.seed = 11;
        LoopOut rm = run_loop(mid, script);
        std::cout << "      (X A=" << rm.res.gain[0] << " p=" << rm.res.p[0] << " L="
                  << rm.res.l_est << "ms | 段长";
        for (int l = 0; l < PAD_CAL_LEVELS_N; ++l) std::cout << " " << rm.seg_ms[0][l][0];
        std::cout << "ms | 行程";
        for (int l = 0; l < PAD_CAL_LEVELS_N; ++l) std::cout << " " << rm.travel[0][l];
        std::cout << "px | done=" << rm.done << ")\n";
        if (rm.done != 1) {
            std::cerr << "      [dbg] err=[" << rm.res.err << "] sigma=" << rm.res.sigma[0]
                      << "/" << rm.res.sigma[1] << " nlv=" << rm.res.nlv[0] << "/" << rm.res.nlv[1]
                      << " hist=" << rm.hist_n << " probe=" << rm.probe_px_s << std::endl;
            for (int l = 0; l < PAD_CAL_LEVELS_N; ++l)
                std::cerr << "      [dbg] lv" << l << " ok=" << rm.res.lv[0][l].seg_ok
                          << "/" << rm.res.lv[0][l].seg_all << " why=" << rm.res.lv[0][l].why
                          << " val=" << rm.res.lv[0][l].val << " lim=" << rm.res.lv[0][l].lim
                          << std::endl;
        }
        CHECK(rm.done == 1 && rm.res.ok && rm.wrote, "闭环: 标定成功并回写脚本");
        CHECK(near(rm.res.gain[0], 800.0f, 0.06f) && near(rm.res.gain[1], 800.0f, 0.06f),
              "闭环: 两轴满偏屏速复原在 6% 内 (探针 → 段长 → 中段取样 → 幂律外推)");
        CHECK(std::fabs(rm.res.l_est - 45.0f) <= 8.0f, "闭环: L 复原 (停顿位移和)");
        CHECK(rm.seg_ms[0][1][0] > rm.seg_ms[0][0][0] + 20.0,
              "闭环: 40% 级段长 > 30% 级下限 (探针驱动的等行程自适应生效)");
        CHECK(rm.seg_ms[0][4][0] <= rm.seg_ms[0][1][0],
              "闭环: 更高挡位段长不增 (T = 目标/V̂)");
        int nb = 0; double worst = 0;
        for (int l = 0; l < PAD_CAL_LEVELS_N; ++l) {          // 未被夹住的级: 行程贴近目标
            if (rm.seg_ms[0][l][0] <= pad_cal_seg_ms_min(120) + 1.0) continue;
            if (rm.seg_ms[0][l][0] >= PAD_CAL_SEG_MS_MAX - 1.0) continue;
            ++nb; worst = std::max(worst, std::fabs(rm.travel[0][l] - (double)PAD_CAL_TRAVEL_PX));
        }
        std::cout << "      (未被夹住的级 n=" << nb << " 最大行程偏差 " << worst << "px)\n";
        CHECK(nb > 0 && worst <= 0.5 * (double)PAD_CAL_TRAVEL_PX,
              "闭环: 行程闭环校正使未被夹住级的实测行程贴近目标 (±50%)");
        if (!script.empty()) unlink(script.c_str());
        if (dir) rmdir(dir);
    }

    std::cout << "[18b] 逐方向判定 (合成窗, 快速): 无纹理单方向采纳 / 夹紧丢弃 / 死区丢弃\n";
    {
        own_motion_ledger_set(true);
        std::deque<CalibSample> hist;
        double base = 700000.0;
        // (a) 死区级: 两方向都不响应 → 整级丢弃, 原因不是样本数 (重播/重跑救不了)
        {
            Synth sy; sy.seg_ms = 258; sy.A[0] = sy.A[1] = 3000.0f;
            sy.scale[0][0] = 0.0f;
            auto win = synth_play(sy, base, hist); base += 40000.0;
            PadCalibResult r = pad_calib_fit(hist, win, SYN_SHIFT_MAX);
            CHECK(r.ok && !r.lv[0][0].valid && !r.lv[0][0].why_samples && r.nlv[0] == 4,
                  "死区级整级丢弃 (位移≈0), 原因不是样本数");
            CHECK(near(r.gain[0], 3000.0f, 0.03f), "丢一级后 A 仍复原");
            CHECK(!pad_cal_deadzone(r, 0).hit, "被丢弃的挡位不进死区诊断");
        }
        // (b) 单方向无纹理 (+向相关峰压低) → 采用 −向并标注, 该级仍有效
        {
            Synth sy; sy.seg_ms = 258; sy.A[0] = sy.A[1] = 3000.0f;
            sy.resp_dir[0][0] = 0.005f;
            auto win = synth_play(sy, base, hist); base += 40000.0;
            PadCalibResult r = pad_calib_fit(hist, win, SYN_SHIFT_MAX);
            CHECK(r.ok && r.lv[0][0].valid && r.lv[0][0].note[0] != 0,
                  "单方向无纹理 → 采纳另一方向并标注 (级仍有效)");
            CHECK(near(r.gain[0], 3000.0f, 0.03f), "单方向测量无偏 (A 复原)");
            CHECK(!pad_cal_deadzone(r, 0).hit, "单方向测量不触发死区诊断");
        }
        // (c) 单方向夹紧 (+向位移≈0 但相关正常) → 整级丢弃 (判据不放宽)
        {
            Synth sy; sy.seg_ms = 258; sy.A[0] = sy.A[1] = 3000.0f;
            sy.dir_scale[0][0] = 0.0f;
            auto win = synth_play(sy, base, hist); base += 40000.0;
            PadCalibResult r = pad_calib_fit(hist, win, SYN_SHIFT_MAX);
            CHECK(!r.lv[0][0].valid && r.lv[0][0].note[0] == 0
                  && std::string(r.lv[0][0].why).find("位移") != std::string::npos,
                  "单方向夹紧 → 整级丢弃且不标注 (夹紧不是换个方向能回答的)");
            CHECK(r.nlv[0] == 0 && r.nlv[1] == 5,
                  "该轴全级丢弃 (该方向的注入被夹住 = 该轴不可标定), 另一轴不受扰");
            CHECK(near(r.gain[1], 3000.0f, 0.03f), "另一轴仍复原 A (逐轴独立)");
        }
        // (d) 轻度死区 (低挡位显著低于过最高挡位的直线) → 死区诊断命中 (仅日志)
        {
            Synth sy; sy.seg_ms = 258; sy.A[0] = sy.A[1] = 3000.0f;
            sy.scale[0][0] = 0.15f;
            auto win = synth_play(sy, base, hist); base += 40000.0;
            PadCalibResult r = pad_calib_fit(hist, win, SYN_SHIFT_MAX);
            PadCalDeadzone z = pad_cal_deadzone(r, 0);
            std::cout << "      (死区诊断: hit=" << z.hit << " 低挡 " << z.v_lo
                      << "px/s vs 直线 " << z.v_line << "px/s)\n";
            CHECK(z.hit && z.lo_li == 0 && z.v_lo < z.v_line, "低挡位低于直线外推 → 诊断命中");
            CHECK(!pad_cal_deadzone(r, 1).hit, "另一轴正常 → 不命中 (逐轴独立)");
        }
        own_motion_ledger_set(false);
    }

    std::cout << "[19] 回执码与整轮重跑判定 (合成结果, 快速)\n";
    {
        PadCalibResult r{};
        r.ok = true; r.gain[0] = 3000.0f; r.gain[1] = 1200.0f;
        CHECK(pad_cal_done_code(r) == 1, "成功且落带内 → 回执 1");
        r.gain[1] = PAD_GAIN_MAX + 1.0f;
        CHECK(pad_cal_done_code(r) == 2, "带外 → 回执 2 (失败, 不回写)");
        r.ok = false; r.gain[1] = 1200.0f; r.nlv[0] = 1; r.nlv[1] = 5;
        for (int a = 0; a < PAD_CAL_AXES_N; ++a)
            for (int l = 0; l < PAD_CAL_LEVELS_N; ++l) {
                r.lv[a][l].valid = true; r.lv[a][l].why_samples = false; }
        r.lv[0][0].valid = false; r.lv[0][0].why_samples = true;
        CHECK(pad_cal_done_code(r) == 3, "仅因中段样本不足且某轴 <2 级 → 回执 3 (建议整轮重跑)");
        r.lv[0][1].valid = false; r.lv[0][1].why_samples = true;   // 仍 <2 级, 原因同为样本数
        CHECK(pad_cal_done_code(r) == 3, "多级样本不足仍 → 回执 3");
        r.lv[0][2].valid = false; r.lv[0][2].why_samples = false;  // 混入可信度原因
        CHECK(pad_cal_done_code(r) == 2, "丢级原因不纯 (含量程/一致性) → 回执 2, 不重跑");
        PadCalibResult e{}; e.err = "静止参考样本不足";
        CHECK(pad_cal_done_code(e) == 2, "整体不可测 → 回执 2 (重跑救不了)");
    }

    std::cout << "[20] hid 十字+停顿: 行程居中与 lag 量化下的 s 偏差\n";
    {
        // (a) 行程对称于起点 (由激励表直接积分: 每段位移 = dx·ticks)
        auto traj = [](const CalibSeg* segs, int n, int loops, long out[6]) {
            long x = 0, y = 0, mxx = 0, mnx = 0, mxy = 0, mny = 0;
            for (int l = 0; l < loops; ++l)
                for (int i = 0; i < n; ++i) {
                    x += (long)segs[i].dx * segs[i].ticks;
                    y += (long)segs[i].dy * segs[i].ticks;
                    mxx = std::max(mxx, x); mnx = std::min(mnx, x);
                    mxy = std::max(mxy, y); mny = std::min(mny, y);
                }
            out[0]=mxx; out[1]=mnx; out[2]=mxy; out[3]=mny; out[4]=x; out[5]=y;
        };
        const int ne = (int)(sizeof(CAL_EXCITE_SEQ) / sizeof(CalibSeg));
        const int ns = (int)(sizeof(CAL_START_SEQ) / sizeof(CalibSeg));
        long a[6], b[6];
        traj(CAL_EXCITE_SEQ, ne, 1, a);
        traj(CAL_START_SEQ, ns, 1, b);
        CHECK(a[0] == 500 && a[1] == -500 && a[2] == 500 && a[3] == -500,
              "激励单圈: 最大偏移 ±500 counts **对称于起点** (老方波是 0..+500 单侧)");
        CHECK(a[4] == 0 && a[5] == 0, "每圈回到起点 (画面内容相似 → 块相关更稳)");
        CHECK(b[0] == 480 && b[1] == -480 && b[2] == 480 && b[3] == -480
              && b[4] == 0 && b[5] == 0, "起始十字: ±480 counts 同样对称于起点");
        int edges = 0, cx = 0, cy = 0;
        for (int i = 0; i < ne; ++i) {
            const CalibSeg& sg = CAL_EXCITE_SEQ[i];
            if (sg.dx != cx || sg.dy != cy) ++edges;
            cx = sg.dx; cy = sg.dy;
        }
        CHECK(edges == 16, "激励单圈 16 个指令边沿 (方波 4 个) → lag 对齐的辨识更强");
        CHECK(ne == 16 && CAL_EXCITE_SEQ[1].dx == 0 && CAL_EXCITE_SEQ[1].dy == 0
              && CAL_EXCITE_SEQ[1].ticks * (double)TICK_MS == 150.0,
              "腿间零指令停顿 150ms (与 pad 的 P 同源: L 上界 100ms + 余量)");

        // (b) 非整数 L: 新网格 (细步 = TICK_MS = 1ms) 复原在 ±1ms 内
        own_motion_ledger_set(false);
        const std::vector<CalibSeg> cross(CAL_EXCITE_SEQ, CAL_EXCITE_SEQ + ne);
        const std::vector<CalibSeg> square = {{2,0,ms_to_ticks(250)},{0,2,ms_to_ticks(250)},
                                              {-2,0,ms_to_ticks(250)},{0,-2,ms_to_ticks(250)}};
        std::deque<CalibSample> h1;
        HidSyn hs; hs.s_true = 1.0f; hs.L_true = 47.3; hs.noise = 0.1f; hs.fps = 120;
        hid_synth(cross, 5, hs, 1000000.0, h1);
        float s1 = 1.0f, l1 = 47.3f;
        CHECK(run_calibration(h1, s1, l1, CALIB_BAND_COUNTS), "hid 合成链路: 拟合成功");
        std::cout << "      (非整数 L=47.3ms → 复原 L=" << l1 << "ms s=" << s1
                  << " (真 1.0); 旧网格半步 1.0ms / 新网格半步 0.5ms)\n";
        CHECK(std::fabs(l1 - 47.3f) <= 1.0f, "lag 网格 (细步 = TICK_MS) 复原在 ±1ms 内");
        CHECK(near(s1, 1.0f, 0.03f), "s 复原在 3% 内 (非整数 L 的残余失配下)");

        // (c) 停顿的量化理由: 把真 L 放在网格最坏相位 (锚点整数 + 0.5ms), 比较两条轨迹
        //     的 s 偏差。100fps 采样 → 粗步 10ms (整数) → 可达 lag 集为 1ms 栅格,
        //     真 L = 锚点 + 0.5 时拟合只能落在 ±0.5ms → 残差固定且可比。
        auto run_traj = [&](const std::vector<CalibSeg>& segs, int loops, double base,
                            double& s_hat, double& l_hat) {
            HidSyn h2; h2.s_true = 1.0f; h2.L_true = 47.5; h2.noise = 0.1f;
            h2.fps = 100; h2.seed = 31;
            std::deque<CalibSample> hh;
            hid_synth(segs, loops, h2, base, hh);
            float sa = 1.0f, la = 47.0f;                 // 锚点 47.0 → 真值在半点上
            run_calibration(hh, sa, la, CALIB_BAND_COUNTS);
            s_hat = sa; l_hat = la;
        };
        double sc = 0, lc = 0, sq = 0, lq = 0;
        run_traj(cross, 5, 2000000.0, sc, lc);
        run_traj(square, 5, 3000000.0, sq, lq);
        std::cout << "      (lag 半数失配: 十字+停顿 s=" << sc << " (偏差 "
                  << 100.0 * (sc - 1.0) << "%) L=" << lc << "ms | 无停顿方波 s=" << sq
                  << " (偏差 " << 100.0 * (sq - 1.0) << "%) L=" << lq << "ms)\n";
        CHECK(std::fabs(sc - 1.0) <= std::fabs(sq - 1.0) + 1e-3,
              "同一 lag 量化失配下, 有停顿的十字 s 偏差不大于无停顿方波");

        // (d) 停顿段静止 → 噪声底 (与注入噪声同量级; 只进日志诊断)
        {
            std::vector<float> nz;
            double t = 1000000.0;
            std::vector<double> pause_mid;
            for (int l = 0; l < 5; ++l)
                for (int i = 0; i < ne; ++i) {
                    const double dur = CAL_EXCITE_SEQ[i].ticks * (double)TICK_MS;
                    if (CAL_EXCITE_SEQ[i].dx == 0 && CAL_EXCITE_SEQ[i].dy == 0)
                        pause_mid.push_back(t + dur * 0.5);
                    t += dur;
                }
            for (const CalibSample& smp : h1)
                for (size_t k = 0; k < pause_mid.size(); ++k)
                    if (std::fabs(elapsed_ms(smp.t, at_ms(pause_mid[k]))) <= 40.0) {
                        nz.push_back(smp.sx); nz.push_back(smp.sy); break; }
            std::sort(nz.begin(), nz.end());
            const float sigma = nz.empty() ? 0.0f
                              : 1.4826f * std::fabs(nz[nz.size() / 2]);
            std::cout << "      (停顿样本估噪声底 σ=" << sigma << "px (注入 0.1px), n="
                      << nz.size() << ")\n";
            CHECK(nz.size() > 50 && sigma < 0.5f,
                  "停顿段静止 → 可估噪声底且与注入噪声同量级 (诊断用, 不改验收判据)");
        }
        own_motion_ledger_set(false);
    }

    std::cout << (g_fail ? "FAILED\n" : "ALL PASS\n");
    return g_fail ? 1 : 0;
}
