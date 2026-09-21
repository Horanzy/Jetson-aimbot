// ============================================================================
//  calib_test — build/calib_test 单测 (scripts/compile.sh 构建并执行, 断言失败即非零
//    退出并终止整个编译):
//    [1] 采样几何不变量 (居中裁切 / 尺度 / 相关域边长 / 可靠每帧位移界 / 量化底)
//    [2] 块统计与采样整链: 注入已知位移的复原, 静止 HUD 占多数时朴素中位被劫持而
//        剔除后精确, 少数静块不改结论, 单块异常回退, 剔除门的尺度无关性
//    [3] 采样方案对照 (一维投影 vs 二维块相关, 同一合成图): 符号口径一致、亚像素
//        误差更小、耗时更低 —— 采样侧选一维的实测依据 (真机数字见 core/calib.h)
//    [4] 状态机: 计划/相位推进, 到位即停, 超时与不响应轴的跳过, 回执路径, 成功/失败
//        收尾, 纯透传复位, 上一轮判定复位, padcalib 重入丢弃
//    [5] 合成闭环 e2e (核心验收): 虚拟游戏 (已知 L; 可选一阶滞后; 可选噪声与丢帧;
//        60/120fps; 逐轴可给不同的 L) 跑完整一轮, 报出 L 的偏差与散度; 两轴延迟不同的
//        两条链钉住"逐轴并列报出、回写取全轮中位" (大差被散度门拦下, 小差照常成功)
//    [6] 不可测的诚实性: 画面不响应 / 只有噪声 / 尾迹被停顿截断 → 整轮失败、不写回
//  全部断言通过输出 ALL PASS 并返回 0。
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <unistd.h>

#include "core/calib.h"
#include "core/control.h"
#include "core/state.h"
#include "io/calib_run.h"
#include "io/pad_input.h"

static int g_fail = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { std::cout << "  ❌ " << (msg) << "\n"; ++g_fail; } \
    else         { std::cout << "  ✅ " << (msg) << "\n"; } } while (0)

using clk = std::chrono::steady_clock;
static clk::time_point g_t0 = clk::now();
static clk::time_point at_ms(double ms) {
    return g_t0 + std::chrono::duration_cast<clk::duration>(
                      std::chrono::duration<double, std::milli>(ms));
}

// ========================= 合成图与注入 =========================
namespace {

// 带限噪声 + 结构性内容 (相位相关需要宽带纹理; 统计与真机采集画面同量级)
cv::Mat synth(int side, unsigned seed) {
    cv::Mat m(side, side, CV_32F);
    std::mt19937 rng(seed);
    std::normal_distribution<float> nz(0, 1);
    cv::Mat low(side, side, CV_32F), high(side, side, CV_32F);
    for (int y = 0; y < side; ++y) for (int x = 0; x < side; ++x) {
        low.at<float>(y, x) = nz(rng); high.at<float>(y, x) = nz(rng); }
    cv::Mat l2, h2; cv::blur(low, l2, cv::Size(9, 9)); cv::blur(high, h2, cv::Size(3, 3));
    m = l2 * 60.f + h2 * 25.f;
    cv::rectangle(m, cv::Rect(side / 6, side / 5, side / 4, side / 3), cv::Scalar(80), -1);
    cv::rectangle(m, cv::Rect(side / 2, side / 2, side / 5, side / 4), cv::Scalar(-60), -1);
    return m;
}
// 循环亚像素平移 (傅里叶位移定理): 注入量与真实位移精确对应
cv::Mat shift_fft(const cv::Mat& src, double dx, double dy) {
    const int n = src.cols;
    cv::Mat planes[] = {cv::Mat_<float>(src), cv::Mat::zeros(src.size(), CV_32F)};
    cv::Mat cpx; cv::merge(planes, 2, cpx);
    cv::dft(cpx, cpx, cv::DFT_COMPLEX_OUTPUT);
    for (int v = 0; v < n; ++v) for (int u = 0; u < n; ++u) {
        const double ph = -2 * CV_PI * (u * dx + v * dy) / n;
        const cv::Vec2f p = cpx.at<cv::Vec2f>(v, u);
        cpx.at<cv::Vec2f>(v, u) = cv::Vec2f((float)(p[0]*std::cos(ph) - p[1]*std::sin(ph)),
                                            (float)(p[0]*std::sin(ph) + p[1]*std::cos(ph)));
    }
    cv::Mat out; cv::dft(cpx, out, cv::DFT_INVERSE | cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
    return out;
}
// 对照基线: 二维 3×3 块相位相关 (归档口径)。只存在于本单测 —— 用来给出"一维投影
//   更准更省"的实测对照; 采样侧不在库里实现它, core/calib.h 的对照表由本用例复现。
double blocks_2d_x(const cv::Mat& a, const cv::Mat& b, const cv::Mat& hann) {
    double dx[CALIB_BLOCKS_N], rq[CALIB_BLOCKS_N];
    int n = 0;
    for (int by = 0; by < CALIB_GRID_N; ++by)
        for (int bx = 0; bx < CALIB_GRID_N; ++bx) {
            const cv::Rect r(bx*CALIB_BLOCK_PX, by*CALIB_BLOCK_PX,
                             CALIB_BLOCK_PX, CALIB_BLOCK_PX);
            double resp = 0;
            const cv::Point2d sh = cv::phaseCorrelate(a(r), b(r), hann, &resp);
            if (resp > CALIB_BLOCK_RESP) { dx[n] = sh.x; rq[n] = (float)resp; ++n; }
        }
    if (n < CALIB_BLOCK_MIN) return 0.0;
    double mx = 0;
    for (int i = 0; i < n; ++i) mx = std::max(mx, std::fabs(dx[i]));
    const double thr = CALIB_STATIC_FRAC * mx;
    std::vector<double> sel;
    for (int i = 0; i < n; ++i) if (std::fabs(dx[i]) >= thr) sel.push_back(dx[i]);
    if (sel.size() < (size_t)CALIB_CLUSTER_MIN)
        { sel.clear(); for (int i = 0; i < n; ++i) sel.push_back(dx[i]); }
    std::nth_element(sel.begin(), sel.begin()+sel.size()/2, sel.end());
    return sel[sel.size()/2] * CALIB_SAMPLE_SCALE;   // 相关域 → 屏幕域 (与库同一尺度)
}

} // namespace

// ========================= 合成闭环: 状态机 + 采样 + 虚拟游戏 =========================
namespace {

struct Plant {
    double L = 40;         // 真实环路延迟 (ms)
    double v = 1.5;        // 激励下的屏幕速度 (px/ms)
    double tau = 0;        // 响应的一阶滞后 (ms; 0 = 纯延迟)
    double sigma = 0.03;   // 逐帧位移噪声 (px; 真机数字采集 0.03–0.1)
    double drop_p = 0;     // 逐帧样本丢失概率
    double move = 1.0;     // 屏幕响应比例 (0 = 完全不动)
    double L_y = -1;       // Y 轴自己的延迟 (ms; <0 = 与 L 相同) —— 逐轴策略不同的合成
};

struct E2E {
    bool  ok = false;                    // 拟合整体成功 (判定通过)
    bool  ran = false;                   // 拟合确实跑过 (无论成败)
    CalResult r;
    int   ticks = 0;                     // 本轮实际拍数 (时长)
    std::string err;
};

// 一轮完整标定: 状态机按 1ms 合成拍推进 (now 由本用例给, 故整轮可离线重放),
//   采样按 cam_fps 的帧网格出样本 (虚拟游戏给出位移), 拟合在 g_calib_request 时跑。
E2E run_e2e(CalMode mode, const Plant& plant, int cam_fps, unsigned seed, bool freeze) {
    E2E e;
    g_aim_enabled.store(true);
    g_ads_down.store(false);
    g_spd_x.store(SPD_BASE); g_spd_y.store(SPD_BASE);
    g_ads_spd_x.store(SPD_BASE); g_ads_spd_y.store(SPD_BASE);
    // 口径复位: 上一用例可能停在半路 (接管关闭一拍即整轮复位), 再断言从空闲起跑
    g_aim_enabled.store(false);
    for (int i = 0; i < 3; ++i) cal_step(mode, 0, cam_fps, at_ms(0));
    g_aim_enabled.store(true);
    g_calib_done.store(0);
    g_padcalib_request.store(false);
    g_calib_collect.store(false);
    g_calib_request.store(false);

    const double dt = 1000.0 / (double)cam_fps;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    std::normal_distribution<double> nz(0.0, 1.0);
    std::deque<CalibSample> hist;
    const size_t hist_max = (size_t)cal_hist_frames(mode, cam_fps);

    const uint16_t trig = (mode == CAL_MODE_PAD)
        ? (uint16_t)(PADBTN_L3 | PADBTN_R3) : (uint16_t)BOTH_SIDE_KEYS;
    // 虚拟游戏: 世界累积位移按 1ms 拍积分 (一阶滞后作用在速度上)。同一时刻只有一条
    //   轴被激励 (计划逐轴逐个方向), 故命令量取非零的那一轴, 位移落在对应轴上 —
    //   与真机一致 (hid 的激励写 cx/cy, pad 的写摇杆偏转)。
    std::vector<double> D;                       // 世界在每拍末的累积位移 (px)
    D.reserve(60000);
    double d_acc = 0.0, v_filt = 0.0;
    const double lag_a = (plant.tau > 0) ? (1.0 - std::exp(-1.0 / plant.tau)) : 1.0;
    double next_sample = 0.0, last_ts = -1.0;
    int ex_axis = 0;
    const int MAX_TICKS = 40000;
    for (int tick = 0; tick < MAX_TICKS && !e.ran; ++tick) {
        const double t = (double)tick;
        const uint16_t btns = (tick < 5000) ? trig : 0;      // 触发键按满 5s (触发判据) 后松开
        const CalStep st = cal_step(mode, btns, cam_fps, at_ms(t));
        if (st.cx != 0) ex_axis = 0;
        else if (st.cy != 0) ex_axis = 1;
        const double cmd = st.active
            ? ((mode == CAL_MODE_PAD)
                   ? (double)(ex_axis ? st.cy : st.cx) / (double)PAD_AXIS_MAX
                   : (double)(ex_axis ? st.cy : st.cx))
            : 0.0;
        const double dir = (cmd > 0) ? +1.0 : (cmd < 0 ? -1.0 : 0.0);
        const double v_tgt = freeze ? 0.0 : plant.v * plant.move * dir;
        v_filt += lag_a * (v_tgt - v_filt);
        d_acc += v_filt * 1.0;                                // 1ms 拍 (v 单位 px/ms)
        D.push_back(d_acc);
        ++e.ticks;
        auto Dof = [&](double tau_w)->double {                // 世界在 tau_w 处的累积位移
            if (tau_w <= 0) return 0.0;
            const double i = tau_w;
            if (i >= (double)D.size()) return D.back();
            const size_t i0 = (size_t)std::floor(i);
            const size_t i1 = std::min(i0 + 1, D.size() - 1);
            const double f = i - (double)i0;
            return D[i0] * (1.0 - f) + D[i1] * f;
        };
        // 采样帧 (帧网格与采集侧同构: 每 dt 一个样本, 其 dt 记的是距上一有效样本)
        while (t + 1.0 > next_sample + 1e-9) {
            const double ts = next_sample;
            next_sample += dt;
            if (uni(rng) >= plant.drop_p) {
                // 观测: 画面在上一有效样本到本样本之间走过的距离 (丢帧即间隔变大 —
                //   与真机相邻有效帧的相关口径一致)
                const double ts_prev = (last_ts < 0) ? (ts - dt) : last_ts;
                // 逐轴延迟: Y 轴可以有自己的 L (两轴平滑策略不同的合成链)
                const double L_ax = (ex_axis == 1 && plant.L_y >= 0) ? plant.L_y : plant.L;
                const double disp = Dof(ts - L_ax) - Dof(ts_prev - L_ax);
                CalibSample s;
                s.t = at_ms(ts);
                s.dt_ms = (float)((last_ts < 0) ? dt : (ts - last_ts));
                last_ts = ts;
                const double noise = plant.sigma * nz(rng);
                s.sx = (float)(ex_axis == 0 ? disp + noise : noise);
                s.sy = (float)(ex_axis == 1 ? disp + noise : plant.sigma * nz(rng));
                s.sx_all = s.sx; s.sy_all = s.sy;
                s.ok[0] = s.ok[1] = true;
                s.resp[0] = s.resp[1] = 0.7f;
                s.spread[0] = s.spread[1] = 0.3f;
                s.slot = cal_note_sample(s);                  // 在线累计 + 打槽位
                hist.push_back(s);
                while (hist.size() > hist_max) hist.pop_front();
            }
        }
        if (g_calib_request.exchange(false)) {                // 采样侧拟合 (真机在采集线程)
            e.r = cal_fit(mode, hist, g_cal_win.snapshot());
            g_calib_done.store(cal_done_code(e.r));
            e.ran = true; e.ok = e.r.ok;
            e.err = e.r.err;
            if (getenv("CAL_E2E_VERBOSE")) cal_print_diag(mode, e.r, hist.size());
        }
    }
    return e;
}

} // namespace

int main() {
    g_t0 = clk::now();
    std::cout << "=== calib_test (标定: 采样几何 / 块统计 / 状态机 / 合成闭环) ===\n";

    std::cout << "[1] 采样几何不变量\n";
    {
        const CalCropRect r = calib_crop_rect(1920, 1080);
        CHECK(r.x == (1920 - CALIB_CROP_PX) / 2 && r.y == (1080 - CALIB_CROP_PX) / 2
              && r.w == CALIB_CROP_PX && r.h == CALIB_CROP_PX,
              "标定裁剪居中且 1:1 (640 正方形; 推理链路的裁剪不受影响)");
        CHECK(CALIB_SAMPLE_PX == CALIB_CROP_PX / 2 && CALIB_BLOCK_PX == CALIB_SAMPLE_PX / 3,
              "640 裁切 → 320 相关域 → 3×3 块 (块宽 106 相关域 px)");
        CHECK(CALIB_SAMPLE_SCALE == 2.0f, "尺度 = 2 屏幕 px / 相关域 px (由裁剪/采样两边导出)");
        CHECK(std::fabs((double)calib_shift_max_px() - 106.0) < 0.01,
              "可靠每帧位移界 = 域内块宽一半 × 尺度 = 106px (循环相关峰唯一界)");
        CHECK(std::fabs((double)calib_edge_floor_px() - 0.5) < 1e-6,
              "边沿/移动判据的量化底 = 0.5px (亚峰内插分辨率的屏幕域值)");
    }

    std::cout << "[2] 块统计与采样整链\n";
    {
        CalibSampler sampler;
        const cv::Mat big = synth(CALIB_CROP_PX, 7u);
        cv::Mat d0; cv::resize(big, d0, cv::Size(CALIB_SAMPLE_PX, CALIB_SAMPLE_PX),
                               0, 0, cv::INTER_AREA);
        {
            const cv::Mat d1 = shift_fft(d0, 6.0, 0.0);      // 域内 6px = 屏幕 12px
            const CalibSampler::Frame f = sampler.measure(d0, d1);
            CHECK(f.ok[0] && std::fabs((double)f.shift[0] + 12.0) < 0.2,
                  "整链复原: 域内注入 6px → 屏幕位移 −12px (方向 = 命令方向, 尺度 2)");
            CHECK(f.ok[1] && std::fabs((double)f.shift[1]) < 0.2,
                  "旁轴读数 ≈ 0 (纯水平注入不产生垂直读数)");
        }
        for (double inj_d : {8.0, 25.0}) {
            const cv::Mat d1 = shift_fft(d0, inj_d, 0.0);
            cv::Mat hud = d1.clone();
            for (int bx = 0; bx < 3; ++bx) for (int by = 0; by < 2; ++by)
                d0(cv::Rect(bx*CALIB_BLOCK_PX, by*CALIB_BLOCK_PX,
                            CALIB_BLOCK_PX, CALIB_BLOCK_PX))
                    .copyTo(hud(cv::Rect(bx*CALIB_BLOCK_PX, by*CALIB_BLOCK_PX,
                                         CALIB_BLOCK_PX, CALIB_BLOCK_PX)));
            const CalibSampler::Frame f = sampler.measure(d0, hud);
            const double want = -inj_d * CALIB_SAMPLE_SCALE;
            CHECK(f.ok[0] && std::fabs((double)f.shift[0] - want) < 0.3,
                  "静止 HUD 占 6/9 块: 剔除后复原精确");
            CHECK(std::fabs((double)f.shift_all[0]) < 0.3 * std::fabs(want),
                  "对照量: 剔除前的全体中位被静止簇劫持到 ≈0 (现场可见的失败模式)");
            CHECK(f.n_static[0] >= 5, "被剔除的静止块数 ≥5");
        }
        {
            const cv::Mat d1 = shift_fft(d0, 20.0, 0.0);
            cv::Mat hud = d1.clone();
            for (int bx = 0; bx < 3; ++bx)
                d0(cv::Rect(bx*CALIB_BLOCK_PX, 0, CALIB_BLOCK_PX, CALIB_BLOCK_PX))
                    .copyTo(hud(cv::Rect(bx*CALIB_BLOCK_PX, 0,
                                         CALIB_BLOCK_PX, CALIB_BLOCK_PX)));
            const CalibSampler::Frame f = sampler.measure(d0, hud);
            CHECK(f.ok[0] && std::fabs((double)f.shift[0] + 40.0) < 0.3,
                  "静止块占少数 (3/9): 剔除前后都给出正确位移");
        }
        {
            float sh[CALIB_BLOCKS_N], rq[CALIB_BLOCKS_N];
            for (int i = 0; i < 6; ++i) { sh[i] = 10.0f; rq[i] = 0.7f; }
            sh[6] = 300.0f; rq[6] = 0.9f;                 // 单块回卷异常
            sh[7] = 0.02f;  rq[7] = 0.8f;                 // 两个静块
            sh[8] = 0.01f;  rq[8] = 0.8f;
            const CalAxisStats st = calib_axis_stats(9, sh, rq);
            CHECK(st.ok && std::fabs((double)st.shift + 20.0) < 1.0,
                  "单块异常顶高门限 → 回退全体中位 (多数块仍在, 结论不崩)");
        }
        {
            float sh[CALIB_BLOCKS_N], rq[CALIB_BLOCKS_N];
            for (int i = 0; i < 9; ++i) { sh[i] = (i < 5) ? 0.5f : 40.0f; rq[i] = 0.7f; }
            const CalAxisStats a = calib_axis_stats(9, sh, rq);
            for (int i = 0; i < 9; ++i) sh[i] *= 100.0f;
            const CalAxisStats b = calib_axis_stats(9, sh, rq);
            CHECK(a.n_static == 5 && b.n_static == 5
                  && std::fabs((double)a.shift * 100.0 - (double)b.shift) < 1.0,
                  "剔除门是最大块位移的比例 (无量纲): 位移放大 100 倍结论不变");
        }
        {
            float sh[CALIB_BLOCKS_N], rq[CALIB_BLOCKS_N];
            for (int i = 0; i < 9; ++i) { sh[i] = 10.0f; rq[i] = 0.7f; }
            const CalAxisStats st = calib_axis_stats(3, sh, rq);   // 只纳入 3 块
            CHECK(!st.ok, "纳入块数 < 4 (画面大面积无纹理) → 该帧不出样本");
        }
    }

    std::cout << "[3] 采样方案对照: 一维投影 vs 二维块相关\n";
    {
        const cv::Mat big = synth(CALIB_CROP_PX, 23u);
        cv::Mat d0; cv::resize(big, d0, cv::Size(CALIB_SAMPLE_PX, CALIB_SAMPLE_PX),
                               0, 0, cv::INTER_AREA);
        cv::Mat hann; cv::createHanningWindow(hann, cv::Size(CALIB_BLOCK_PX, CALIB_BLOCK_PX),
                                              CV_32F);
        CalibSampler sampler;
        double err2d = 0, err1d = 0, worst2d = 0, worst1d = 0;
        for (double inj : {5.0, 15.0, 30.0}) {
            const cv::Mat d1 = shift_fft(d0, inj, 0.0);
            const double want = -inj * CALIB_SAMPLE_SCALE;      // 库口径 (命令方向)
            const double e2 = std::fabs(blocks_2d_x(d0, d1, hann) - (-want));
            const CalibSampler::Frame f = sampler.measure(d0, d1);
            const double e1 = std::fabs((double)f.shift[0] - want);
            err2d += e2; err1d += e1;
            worst2d = std::max(worst2d, e2); worst1d = std::max(worst1d, e1);
        }
        printf("     亚像素误差 (屏幕 px, 三个注入点): 二维 均 %.3f 最大 %.3f | "
               "一维 均 %.3f 最大 %.3f\n", err2d/3, worst2d, err1d/3, worst1d);
        CHECK(err1d <= err2d, "一维投影的亚像素误差 ≤ 二维 (真机真实纹理: 0.01 vs 0.04–0.15px)");
        CHECK(err1d / 3.0 < 0.3, "一维投影复原误差 < 0.3px (屏幕域)");
        const cv::Mat d1 = shift_fft(d0, 7.0, 3.0);
        volatile double acc = 0;
        auto t_a = clk::now();
        for (int i = 0; i < 200; ++i) acc += blocks_2d_x(d0, d1, hann);
        auto t_b = clk::now();
        for (int i = 0; i < 200; ++i) { CalibSampler::Frame f = sampler.measure(d0, d1);
                                        acc += f.shift[0] + f.shift[1]; }
        auto t_c = clk::now();
        const double ms2d = std::chrono::duration<double, std::milli>(t_b - t_a).count()/200.0;
        const double ms1d = std::chrono::duration<double, std::milli>(t_c - t_b).count()/200.0;
        printf("     耗时 (同一对帧 ×200): 二维 3×3 域 320 %.2f ms/帧 | 一维投影两轴 %.2f ms/帧\n",
               ms2d, ms1d);
        CHECK(ms1d < ms2d, "一维投影更快 (真机实测 0.90 vs 5.19 ms/帧; 11% vs 62% @120fps)");
        (void)acc;
    }

    std::cout << "[4] 状态机\n";
    {
        const std::vector<CalPlanSeg> ph = cal_plan(CAL_MODE_HID);
        const std::vector<CalPlanSeg> pp = cal_plan(CAL_MODE_PAD);
        CHECK(ph.size() == (size_t)(2 * 2 * cal_pairs(CAL_MODE_HID) * 2)
              && pp.size() == (size_t)(2 * cal_pairs(CAL_MODE_PAD) * 2),
              "计划 = 轴 × 方向 × 对 × (激励+停顿); hid 两轴, pad 单轴 (只扫水平)");
        bool shape = true;
        for (size_t i = 0; i + 1 < pp.size(); i += 2)
            if (pp[i].pause || pp[i].dir == 0 || !pp[i + 1].pause) shape = false;
        bool sgn = true;
        for (size_t i = 0; i + 1 < pp.size(); i += 2)
            if (pp[i].dir != ((i / 2 % 2 == 0) ? +1 : -1)) sgn = false;
        CHECK(shape && sgn, "每段激励后跟停顿; 方向符号交替 (画面净行程回到起点)");
        printf("     hid 振幅 X=%.4g/ms Y=%.4g/ms (设计屏速 %.2f/%.2f px/ms) | pad 偏转 %.3g\n",
               (double)ph[0].amp, (double)ph[4].amp,
               (double)(CAL_TRAVEL_PX[0] / (float)CAL_SEG_MIN_MS),
               (double)(CAL_TRAVEL_PX[1] / (float)CAL_SEG_MIN_MS), (double)pp[0].amp);
        CHECK(ph[0].amp > 0 && pp[0].amp >= CAL_PAD_DEFL_MIN && pp[0].amp <= 1.0f,
              "pad 偏转落在 [死区地板 30%, 满偏] (实测 10% 偏转几乎不动画面)");
        CHECK(cal_plan_worst_ms(CAL_MODE_HID) < 20000
              && cal_plan_worst_ms(CAL_MODE_PAD) < 20000,
              "整轮最坏时长有界 (每轴连续 N 段超时即跳过该轴)");
        CHECK(cal_plan_worst_ms(CAL_MODE_HID) - cal_plan_span_ms(CAL_MODE_HID) == CAL_TRIGGER_MS
              && cal_plan_worst_ms(CAL_MODE_PAD) - cal_plan_span_ms(CAL_MODE_PAD) == CAL_TRIGGER_MS,
              "触发时打印的是计划时长 (不含触发长按窗): 两个数恰好差一个 CALIB_TRIGGER_TICKS");
        printf("     计划 ≤%.1fs (hid) / ≤%.1fs (pad) — 触发后还要多久; "
               "含触发长按 %ds 的整轮上限 ≤%.1fs / ≤%.1fs (采样窗深度的出处)\n",
               (double)cal_plan_span_ms(CAL_MODE_HID) / 1000.0,
               (double)cal_plan_span_ms(CAL_MODE_PAD) / 1000.0, CAL_TRIGGER_MS / 1000,
               (double)cal_plan_worst_ms(CAL_MODE_HID) / 1000.0,
               (double)cal_plan_worst_ms(CAL_MODE_PAD) / 1000.0);
        CHECK(cal_hist_frames(CAL_MODE_HID, 120) >= 1000, "采样窗深度覆盖整轮最坏时长");
        // 触发: 双侧键长按 5s (hid) → 起始十字 → 激励相位; 松手不中断
        g_aim_enabled.store(true);
        g_calib_collect.store(false);
        for (int i = 0; i < 3; ++i) cal_step(CAL_MODE_HID, 0, 120, at_ms(0));
        bool active0 = false;
        for (int i = 0; i < 4900; ++i) active0 |= cal_step(CAL_MODE_HID, BOTH_SIDE_KEYS, 120,
                                                           at_ms(i)).active;
        CHECK(!active0, "长按不足 5s 不触发");
        bool trig = false;
        for (int i = 4900; i < 5200; ++i)
            trig |= cal_step(CAL_MODE_HID, BOTH_SIDE_KEYS, 120, at_ms(i)).active;
        CHECK(trig, "双侧键长按满 5s → 进入标定 (起始十字/准备期)");
        CHECK(g_calib_collect.load() == false,
              "起始十字期不采样 (g_calib_collect 未开, 采样只在激励段)");
        // 推进到激励相位: collect 打开
        bool collecting = false;
        for (int i = 0; i < 2200 && !collecting; ++i) {
            cal_step(CAL_MODE_HID, 0, 120, at_ms(5200 + i));
            collecting = g_calib_collect.load();
        }
        CHECK(collecting, "起始十字播完 → 激励相位 (g_calib_collect 开, 推理链跳过)");
        // 到位即停: 喂一个已越过目标的累计行程 → 该段立即结束, 段窗口置 begun/非暂停
        const size_t seg_before = g_cal_win.size();
        g_cal_live_slot.store(0); g_cal_live_travel.store(CAL_TRAVEL_PX[0] + 1.0f);
        for (int i = 0; i < 5; ++i) cal_step(CAL_MODE_HID, 0, 120, at_ms(8000 + i));
        CHECK(seg_before > 0 && g_cal_win.snapshot()[0].begun
              && g_cal_win.snapshot()[0].timeout == false
              && g_cal_win.snapshot()[1].begun,
              "到位即停: 累计行程达目标 → 段立即结束并进入停顿 (段长是测量不是参数)");
        // 纯透传复位
        g_aim_enabled.store(false);
        cal_step(CAL_MODE_HID, 0, 120, at_ms(9000));
        CHECK(!g_calib_collect.load() && g_cal_win.size() == 0,
              "-a n (纯透传): 进行中的标定复位且不可达 (激励与透传互斥)");
        g_aim_enabled.store(true);
        // padcalib 热参请求: 空闲时触发; 进行中到达则丢弃
        g_calib_collect.store(false);
        for (int i = 0; i < 3; ++i) cal_step(CAL_MODE_PAD, 0, 120, at_ms(0));
        g_padcalib_request.store(true);
        bool padtrig = cal_step(CAL_MODE_PAD, 0, 120, at_ms(1)).active;
        CHECK(padtrig, "热参 padcalib=1 → 手柄模式触发标定");
        g_padcalib_request.store(true);
        cal_step(CAL_MODE_PAD, 0, 120, at_ms(2));
        CHECK(!g_padcalib_request.load(), "进行中到达的请求一次消费即清 (绝不重入)");
        // 上一轮判定复位: 采样侧写 1 后, 新一轮开跑必须清零
        g_calib_done.store(1);
        g_aim_enabled.store(false); cal_step(CAL_MODE_PAD, 0, 120, at_ms(3));
        g_aim_enabled.store(true);
        g_padcalib_request.store(true);
        for (int i = 0; i < 2000; ++i) cal_step(CAL_MODE_PAD, 0, 120, at_ms(100 + i));
        CHECK(g_calib_done.load() == 0 && g_calib_collect.load(),
              "新一轮开跑时上一轮判定复位 (不清则点头/摇头报的是上一轮的结论)");
        g_aim_enabled.store(false); cal_step(CAL_MODE_PAD, 0, 120, at_ms(3000));
        g_aim_enabled.store(true);
    }

    std::cout << "[5] 合成闭环 e2e: 偏差与散度 (核心验收)\n";
    {
        struct Case { const char* name; Plant p; int fps; bool freeze; double tol; };
        const std::vector<Case> cases = {
            {"纯延迟 L=20 @120fps σ=0.10",   {20, 1.5, 0,  0.10, 0, 1}, 120, false, 1.0},
            {"纯延迟 L=40 @120fps σ=0.03",   {40, 1.5, 0,  0.03, 0, 1}, 120, false, 1.0},
            {"纯延迟 L=40 @ 60fps σ=0.03",   {40, 1.5, 0,  0.03, 0, 1},  60, false, 1.0},
            {"纯延迟 L=70 @120fps σ=0.10",   {70, 1.5, 0,  0.10, 0, 1}, 120, false, 1.0},
            {"纯延迟 L=70 @ 60fps σ=0.10",   {70, 1.5, 0,  0.10, 0, 1},  60, false, 1.0},
            {"一阶滞后 L=40 τ=15 @120fps",    {40, 1.5, 15, 0.03, 0, 1}, 120, false, 4.0},
            {"一阶滞后 L=40 τ=15 @ 60fps",    {40, 1.5, 15, 0.05, 0, 1},  60, false, 4.0},
            {"丢帧12% L=40 @120fps σ=0.10",  {40, 1.5, 0,  0.10, 0.12, 1}, 120, false, 2.0},
            {"慢游戏 600px/s L=40 @120fps",   {40, 0.6, 0,  0.05, 0, 1}, 120, false, 1.0},
        };
        printf("     %-32s %-8s | 尾迹  停止沿 起始沿 | 偏差    MAD    σx    MSAD\n", "用例", "结果");
        int n_ok = 0;
        for (const Case& c : cases) {
            const E2E e = run_e2e(CAL_MODE_HID, c.p, c.fps, 1u, c.freeze);
            if (e.ok) {
                // 真值 = 纯延迟 + 响应自身的一阶时间常数 (尾迹读的就是这个等效滞后)
                const double truth = c.p.L + c.p.tau;
                const double bias = (double)e.r.l_est - truth;
                ++n_ok;
                CHECK(std::fabs(bias) <= c.tol, "偏差落在用例容差内 (逐用例的偏差与散度见上表)");
                CHECK((double)e.r.l_tail_mad <= (double)e.r.dt_ms,
                      "读数族散度 (MAD) 不超一个实测采样间隔");
                printf("     %-32s ok       | %5.1f %6.1f %6.1f | %+6.1f %5.1f %.3f %5.1f\n",
                       c.name, (double)e.r.l_tail, (double)e.r.l_stop, (double)e.r.l_onset,
                       bias, (double)e.r.l_tail_mad, (double)e.r.sigma[0],
                       (double)e.r.dt_ms);
            } else
                printf("     %-32s 失败: %s\n", c.name, e.r.err);
        }
        printf("     (一阶滞后用例的真值 = L + τ: 尾迹读的是画面全部残余运动的等效滞后,\n"
               "      这正是控制律要补偿的那个量; 边沿读数因平滑磨圆而略偏)\n");
        CHECK(n_ok == (int)cases.size(), "全部用例整轮成功 (无一失败)");

        // 两轴延迟不同的两条合成链 (hid 两轴各是一份独立读数; 逐轴中位并列报出, 回写取
        //   全轮中位 —— 见 calib_run.h 的聚合段, 那里没有逐轴一致性判定: 4 条读数不足以
        //   再分一层, 而两轴真分得开时先被尾迹族的散度门拦下)
        {
            Plant p{40, 1.5, 0, 0.03, 0, 1};
            p.L_y = 70;                                     // 两轴差 30ms = 3.6 个采样间隔
            const E2E e = run_e2e(CAL_MODE_HID, p, 120, 11u, false);
            CHECK(!e.ok && std::string(e.r.err).find("离散") != std::string::npos,
                  "两轴延迟相差 30ms → 尾迹族散度门拦下整轮 (读数不是同一次物理测量的重复)");
            CHECK(e.r.l_axis_n[0] > 0 && e.r.l_axis_n[1] > 0 && e.r.l_axis[1] > e.r.l_axis[0],
                  "逐轴中位仍并列报出 (现场证据), 且 Y 轴高在真实的那一侧");
            printf("     两轴差异大: X %.1fms / Y %.1fms → %s\n",
                   (double)e.r.l_axis[0], (double)e.r.l_axis[1], e.r.err);
        }
        {
            Plant p{40, 1.5, 0, 0.03, 0, 1};
            p.L_y = 44;                                     // 小差 (4ms < 半个采样间隔)
            const E2E e = run_e2e(CAL_MODE_HID, p, 120, 13u, false);
            CHECK(e.ok, "两轴小差 (4ms) 整轮成功 (同一次物理测量的两半)");
            CHECK(std::fabs((double)e.r.l_axis[0] - 40.0) <= 1.5
                  && std::fabs((double)e.r.l_axis[1] - 44.0) <= 1.5,
                  "逐轴中位各自落在本轴的真值上 (并列诊断读得出这个差)");
            CHECK(std::fabs((double)e.r.l_est - 42.0) <= 2.5
                  && (double)e.r.l_est >= (double)e.r.l_axis[0] - 1.0
                  && (double)e.r.l_est <= (double)e.r.l_axis[1] + 1.0,
                  "回写值 = 全轮 (两轴合并) 中位: 落在两轴中位之间, 不是某一轴的数");
            printf("     两轴差异小: X %.1fms / Y %.1fms → 回写 %.1fms (全轮中位)\n",
                   (double)e.r.l_axis[0], (double)e.r.l_axis[1], (double)e.r.l_est);
        }
    }

    std::cout << "[6] 不可测的诚实性: 失败 + 不写回\n";
    {
        const std::string path = "/tmp/calib_test_script.sh";
        // 模板形态的脚本: 三套输出各一格延迟, 都是守卫写法 —— 回写要落在本模式那一格,
        //   且不该把守卫写法抹成裸赋值 (control_test [7] 逐字钉住写法的细节)
        const std::string script =
            "#!/bin/bash\n"
            "HID_L_EST=\"${HID_L_EST:-60.0}\"\n"
            "PAD_L_EST=\"${PAD_L_EST:-60.0}\"\n"
            "P5G_L_EST=\"${P5G_L_EST:-60.0}\"\n";
        { std::ofstream o(path, std::ios::trunc); o << script; }
        const auto slurp = [&]() { std::ifstream in(path);
            return std::string((std::istreambuf_iterator<char>(in)),
                               std::istreambuf_iterator<char>()); };
        // 画面完全冻结: σ=0 → 无响应
        {
            const E2E e = run_e2e(CAL_MODE_HID, {40, 1.5, 0, 0.0, 0, 1}, 120, 3u, true);
            CHECK(!e.ok && std::string(e.r.err).find("静止") != std::string::npos,
                  "画面完全静止 → 失败并说明原因 (不允许硬算)");
            CHECK(!cal_writeback(CAL_VAR_HID, e.r, path), "失败路径不写回");
        }
        // 只有噪声、没有任何响应 (画面在抖但不跟命令)
        {
            Plant p{40, 1.5, 0, 0.10, 0, 0.0};       // move=0: 屏幕不响应
            const E2E e = run_e2e(CAL_MODE_HID, p, 120, 5u, false);
            CHECK(!e.ok, "屏幕不响应 (只有噪声位移) → 整轮失败");
            CHECK(std::string(e.r.err).find("运动") != std::string::npos
                  || std::string(e.r.err).find("超时") != std::string::npos,
                  "失败原因指向'无任何激励段测到运动'");
            printf("     不响应用例: 实际拍数 %d, 上限 %d\n", e.ticks,
                   cal_plan_worst_ms(CAL_MODE_HID));
            CHECK(e.ticks <= cal_plan_worst_ms(CAL_MODE_HID),
                  "不响应轴被跳过 → 整轮时长不超过上限 (不会跑满最坏时长)");
            CHECK(e.ticks >= CALIB_TRIGGER_TICKS
                  && cal_plan_worst_ms(CAL_MODE_HID) > cal_plan_span_ms(CAL_MODE_HID),
                  "整轮拍数含触发长按窗 (故历史上限含它), 打印的计划时长不含它");
            CHECK(!cal_writeback(CAL_VAR_HID, e.r, path), "失败路径不写回");
        }
        // 尾迹被停顿截断: L 远超静止参考窗起点 → 失败而不是给一个偏小的数
        {
            const E2E e = run_e2e(CAL_MODE_HID, {200, 1.5, 0, 0.05, 0, 1}, 120, 7u, false);
            CHECK(!e.ok, "L 超出停顿能容纳的范围 → 失败");
            CHECK(std::string(e.r.err).find("截断") != std::string::npos
                  || std::string(e.r.err).find("物理带") != std::string::npos,
                  "失败原因 = 尾迹被停顿截断 / 超出物理带 (绝不给截断后的偏小值)");
            CHECK(slurp() == script, "失败路径下三格延迟逐字未动 (绝不写编造的值)");
        }
        // 成功路径: 写本模式那一格 (pad 与 p5g 共用激励计划, 但各写各的延迟槽)
        {
            const E2E e = run_e2e(CAL_MODE_PAD, {40, 1.5, 0, 0.03, 0, 1}, 120, 9u, false);
            CHECK(e.ok, "手柄模式 (单轴) 整轮成功");
            CHECK(cal_writeback(CAL_VAR_PAD, e.r, path), "成功路径写回");
            char want[128];
            snprintf(want,sizeof(want),"PAD_L_EST=\"${PAD_L_EST:-%.1f}\"\n",(double)e.r.l_est);
            const std::string all = slurp();
            CHECK(all.find(want) != std::string::npos,
                  "只写本模式的 VAR (PAD_L_EST), 且守卫写法原样保住");
            CHECK(all.find("HID_L_EST=\"${HID_L_EST:-60.0}\"") != std::string::npos
                  && all.find("P5G_L_EST=\"${P5G_L_EST:-60.0}\"") != std::string::npos,
                  "另两格延迟原值不变");
            CHECK(cal_writeback(CAL_VAR_P5G, e.r, path)
                  && slurp().find("P5G_L_EST=\"${P5G_L_EST:-") != std::string::npos,
                  "第三种输出 (p5g) 写的是它自己那一格 (VAR 名由调用方三选一)");
        }
        ::unlink(path.c_str());
    }

    std::cout << (g_fail ? "FAILED\n" : "ALL PASS\n");
    return g_fail ? 1 : 0;
}
