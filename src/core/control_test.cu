// ============================================================================
//  control_test — build/control_test 单测 (scripts/compile.sh 构建并执行一次):
//    [1] spd=100 即基线: 干净合成目标下实发 counts 等于基线灵敏度 1 px/count 的
//        手算值 (即 1 count = 1 px), 且不顶 ±120 的量化钳制
//    [2] 逐轴独立: spdx 翻倍 → x 的 counts 翻倍而 y 不变; 反之只有 spdy 动 y
//    [3] ADS 当拍切换: 右键按下那一拍即整套换 adsspd 那一对, 并把键位状态导出到
//        g_ads_down 供逐帧消费者取同一状态
//    [4] 三消费者共用同一份逐轴有效灵敏度 s_hid_now: 注入换算 (v·TICK_MS/s)、
//        在飞补偿 (s·Δcounts) 与估计器自身运动补偿 (预测减法/清洗创新/自身加速度
//        活动门) 在同一状态下取同一值, 且随 spd 成比例变化
//    [5] 夹取: spd_clamp 的 [SPD_MIN, SPD_MAX] 带 (CLI 与热参共用的唯一定义点)
//    [6] 热参路径: spdx 改后下一拍生效; 非数值/未知 key (含已删除的 s) 被拒绝
//    [7] 标定回写: 只写调用方给的那个延迟 VAR (速度倍率逐字不变, 无速度 VAR),
//        换名即另一套 (手柄输出的延迟 VAR 与 hid 各占一行, 互不覆盖)
//  全部断言通过输出 ALL PASS 并返回 0; 任一断言失败返回非零 (compile.sh 的 set -e
//  终止编译)。
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <string>

#include <sys/stat.h>
#include <unistd.h>

#include "core/calib.h"
#include "core/control.h"
#include "core/estimator.h"
#include "core/state.h"
#include "io/hotctl.h"

static int g_fail = 0;

#define CHECK(cond, msg) do { \
    if (cond) { std::cout << "  ok  " << msg << "\n"; } \
    else { std::cerr << "  FAIL " << msg << "\n"; ++g_fail; } \
} while (0)

// 与律同式的设计量: kp = 2ζ·wn, wn = (90°−PM)π/180/L (L = 本测注入的标定延迟)
static const double T_L = 50.0;
static double kp_for(double L_ms) {
    return 2.0 * (double)FF_ZETA * (90.0 - (double)FF_PM_DEG) * 3.14159265358979
           / 180.0 / L_ms;
}
// 实发 counts 的期望 = trunc(v·TICK_MS/s_eff)
static int counts_expected(double v_px_per_ms, double s_eff) {
    return (int)std::trunc(v_px_per_ms * (double)TICK_MS / s_eff);
}
// 律对静止目标的该轴命令 (px/ms): v = kp·(目标 px − 在飞像素) — 由 header 常量导出
static double cmd_px(double px, double in_flight_px) { return kp_for(T_L) * (px - in_flight_px); }

// 账本推进到指定的累计 counts (add 记增量)
static void ledger_set(std::chrono::steady_clock::time_point t, long long cx, long long cy) {
    auto cur = g_counts.cum();
    g_counts.add(t, (int)(cx - cur.first), (int)(cy - cur.second));
}
// 查询时刻 tq 两侧放一对**同累计值**的采样 (at() 在两点间线性插值, 两侧同值 ⇒
//   查询值与之逐位相等)。查询时刻落在 Lc 的 float 舍入之内, 只在单侧放点会插出
//   半程值, 故平台的宽度 (2ms) 远大于该舍入。
static void ledger_plateau(std::chrono::steady_clock::time_point tq, long long cx, long long cy) {
    ledger_set(shift_ms(tq, -2.0), cx, cy);
    ledger_set(shift_ms(tq, 2.0), cx, cy);
}

// 透传一拍: 清零跨拍静态量 (量化余量/积分器/标定相位) 与账本
static void reset_tick() {
    g_aim_enabled.store(false);
    uint8_t r[HID_REPORT_LEN] = {};
    control_apply(120, r, 0, 0);
    g_counts.clear();
}

// 设置一个干净目标: 静止 (v̂=â=0)、无 CUSUM 告警
static void arm_target(float px, float py) {
    std::lock_guard<std::mutex> lk(g_target.mtx);
    g_target.px = px; g_target.py = py;
    g_target.vx = 0; g_target.vy = 0;
    g_target.ax_e = 0; g_target.ay_e = 0; g_target.cs = 0;
    g_target.last_dt = 1000.0f / 120.0f;
    g_target.last_alpha = PRED_ALPHA0; g_target.last_beta = PRED_BETA0;
    g_target.l_est_ms = (float)T_L;
    g_target.valid = true;
    g_target.t_pub = shift_ms(std::chrono::steady_clock::now(), -10.0);
}

// 被考察的一拍: 键位 → 报文 → 取实发 counts (int16 LE)
static void fire_tick(uint16_t btn, int& cx, int& cy) {
    uint8_t r[HID_REPORT_LEN] = {};
    r[1] = (uint8_t)(btn & 0xFF); r[2] = (uint8_t)(btn >> 8);
    g_aim_enabled.store(true);
    control_apply(120, r, 0, 0);
    cx = (int)(int16_t)(r[3] | (r[4] << 8));
    cy = (int)(int16_t)(r[5] | (r[6] << 8));
}

// 一次完整测量: 清态 → (可选)植入在飞 counts → 设目标 → 报键位 → 取 counts
static void measure(float px, float py, uint16_t btn, int& cx, int& cy,
                    const std::function<void(std::chrono::steady_clock::time_point)>& seed
                        = nullptr) {
    reset_tick();
    arm_target(px, py);
    if (seed) {
        std::lock_guard<std::mutex> lk(g_target.mtx);
        seed(g_target.t_pub);
    }
    fire_tick(btn, cx, cy);
}

int main() {
    std::cout << "== control_test: 拉枪速度倍率 (spd) 落点 ==\n";
    g_max_v.store(20.0f);                 // 抬高速度帽: 本测考察换算刻度, 不考察钳制
    g_fov_radius.store(1000.0f);          // 扩大 FOV: 免积分器重置路径
    g_aim_mode.store(2);                  // -k both: 左键与右键都触发
    g_spd_x.store(SPD_BASE); g_spd_y.store(SPD_BASE);
    g_ads_spd_x.store(SPD_BASE); g_ads_spd_y.store(SPD_BASE);

    std::cout << "[1] spd=100 即基线 (1 count = 1 px)\n";
    {
        const double v = cmd_px(180.0, 0.0);
        int cx = 0, cy = 0;
        measure(180.0f, 0.0f, LEFT_KEY, cx, cy);
        std::printf("      vcx=%.4f px/ms → counts=(%d,%d); 基线尺度 %d\n",
                    v, cx, cy, counts_expected(v, S_HID_BASE));
        CHECK(cx == counts_expected(v, S_HID_BASE),
              "spd=100: 实发 counts = 基线灵敏度 (1 px/count) 下算出的值");
        CHECK(cx == (int)std::trunc(v) && cx > 0,
              "spd=100: 一拍 1 count = 1 px —— 实发 counts 就是命令的像素数");
        CHECK(std::abs(cx) < 120, "counts 远小于 ±120 量化钳制 (换算错到别的刻度会顶到 120)");
        CHECK(cy == 0, "静止目标的另一轴不发 counts");
    }

    std::cout << "[1b] 纯透传 (-a n): 不注入任何 counts\n";
    {
        g_aim_enabled.store(false);
        uint8_t r[HID_REPORT_LEN] = {};
        r[1] = (uint8_t)(LEFT_KEY & 0xFF);
        control_apply(120, r, -37, 19);
        const int cx = (int)(int16_t)(r[3] | (r[4] << 8));
        const int cy = (int)(int16_t)(r[5] | (r[6] << 8));
        CHECK(cx == -37 && cy == 19,
              "接管关闭: 报文位移 = 真实鼠标位移逐位 (律的注入与倍率一概不参与)");
        g_aim_enabled.store(true);
    }

    std::cout << "[2] 逐轴独立 (spdx 只管 x, spdy 只管 y)\n";
    {
        const double v = cmd_px(180.0, 0.0);
        const int base = counts_expected(v, S_HID_BASE);
        const int dbl  = counts_expected(v, S_HID_BASE * 100.0 / 200.0);
        int cx = 0, cy = 0;
        g_spd_x.store(200); g_spd_y.store(100);
        measure(180.0f, 180.0f, LEFT_KEY, cx, cy);
        std::printf("      spdx=200 spdy=100 → counts=(%d,%d); 期望 (%d,%d)\n", cx, cy, dbl, base);
        CHECK(cx == dbl, "spdx=200: x 有效灵敏度减半 → 同一命令的 counts 翻倍");
        CHECK(cy == base, "spdy=100: y 与基线相同 (x 的倍率不串到 y)");
        g_spd_x.store(100); g_spd_y.store(200);
        measure(180.0f, 180.0f, LEFT_KEY, cx, cy);
        CHECK(cx == base, "spdx=100: x 回到基线 (y 的倍率不串到 x)");
        CHECK(cy == dbl, "spdy=200: y 的 counts 翻倍");
        g_spd_x.store(400);
        measure(180.0f, 180.0f, LEFT_KEY, cx, cy);
        CHECK(cx == counts_expected(v, S_HID_BASE * 100.0 / 400.0),
              "spdx=400: counts 随显示值走完整区间, 无隐藏比例");
        g_spd_x.store(SPD_BASE); g_spd_y.store(SPD_BASE);
    }

    std::cout << "[3] ADS 键: 按住那一拍整套换 adsspd\n";
    {
        const double v = cmd_px(180.0, 0.0);
        int cx = 0, cy = 0;
        const int base = counts_expected(v, S_HID_BASE);
        g_ads_spd_x.store(200); g_ads_spd_y.store(400);
        measure(180.0f, 180.0f, LEFT_KEY, cx, cy);
        CHECK(!ads_down(), "只按左键: 导出的 ADS 状态 = 未按住");
        CHECK(cx == base && cy == base, "只按左键: 用腰射对 spdx/spdy (100/100)");
        measure(180.0f, 180.0f, (uint16_t)(LEFT_KEY | RIGHT_KEY), cx, cy);
        std::printf("      按住右键那拍 → counts=(%d,%d); ADS 期望 (%d,%d)\n", cx, cy,
                    counts_expected(v, S_HID_BASE * 100.0 / 200.0),
                    counts_expected(v, S_HID_BASE * 100.0 / 400.0));
        CHECK(ads_down(), "按住右键那一拍: 导出的 ADS 状态 = 按住");
        CHECK(cx == counts_expected(v, S_HID_BASE * 100.0 / 200.0),
              "按住那一拍 x 即用 adsspdx (200), 无需再过一拍");
        CHECK(cy == counts_expected(v, S_HID_BASE * 100.0 / 400.0),
              "按住那一拍 y 即用 adsspdy (400), 逐轴独立");
        g_ads_spd_x.store(SPD_BASE); g_ads_spd_y.store(SPD_BASE);
    }

    std::cout << "[4] 三消费者共用同一份逐轴有效灵敏度\n";
    {
        const double v = cmd_px(180.0, 0.0);
        const float s100 = s_hid_now(false, 0);
        int cx = 0, cy = 0;
        measure(180.0f, 0.0f, LEFT_KEY, cx, cy);
        CHECK(std::fabs((double)cx * s100 - v * (double)TICK_MS) < 1.0,
              "注入换算: counts × s_hid_now = 命令的像素数 (同一刻度)");
        // 在飞补偿: 飞行窗内植入 5 counts → 折成像素后从 ê 里减掉
        const auto seed_flight = [](std::chrono::steady_clock::time_point tp) {
            const auto tq = shift_ms(tp, -(double)((float)T_L * PRED_L_COMP));
            ledger_plateau(tq, 0, 0);                   // 窗外基准: 查询值 = 0
            ledger_set(shift_ms(tq, 4.0), 5, 0);        // 窗内: 在飞 5 counts
        };
        int ax = 0, ay = 0;
        measure(180.0f, 0.0f, LEFT_KEY, ax, ay, seed_flight);
        const int exp100 = counts_expected(cmd_px(180.0, 5.0 * s100), s100);
        std::printf("      在飞 5 counts: spd=100 → counts=%d (无在飞时 %d); 期望 %d\n",
                    ax, cx, exp100);
        CHECK(ax == exp100,
              "在飞补偿: 5 counts 以 s_hid_now 折成像素 (spd=100 即 5px) 从 ê 里减掉");
        g_spd_x.store(200);
        measure(180.0f, 0.0f, LEFT_KEY, ax, ay, seed_flight);
        const float s200 = s_hid_now(false, 0);
        CHECK(std::fabs(s200 * 2.0f - s100) < 1e-6f, "spd 翻倍 → 该轴有效灵敏度减半");
        CHECK(ax == counts_expected(cmd_px(180.0, 5.0 * s200), s200),
              "在飞补偿随 spd 成比例变化 (命令放大多少, 补偿跟随多少)");
        g_spd_x.store(SPD_BASE);
        // 估计器自身运动补偿: 帧窗内植入已知 counts, 发布值可精确手算
        //   (首拍后 fx=fvx=0 ⇒ px_pred = −s_x·Δcounts, inx = 观测 − px_pred)。
        //   换算比例由估计器按输出模式与 spd 自己取 (io/pad_output.h 的
        //   own_motion_scale), 故这里改 spd 原子而不传比例。
        const auto est_case = [](int spd, float& out_px, float& out_py) {
            g_spd_x.store(spd); g_spd_y.store(spd);
            EstimatorState est;
            g_counts.clear();
            auto t0 = std::chrono::steady_clock::now();
            estimator_step(est, t0, true, 0.0f, 0.0f, (float)T_L, 20.0f);
            auto t1 = shift_ms(t0, 10.0);
            ledger_plateau(shift_ms(t1, -65.0), 0, 0);      // 飞行窗 [t1−65, t1−55]:
            ledger_set(shift_ms(t1, -60.0), 10, 4);         //   窗内 Δcounts = (10,4)
            ledger_plateau(shift_ms(t1, -55.0), 10, 4);
            estimator_step(est, t1, true, -8.0f, -3.0f, (float)T_L, 20.0f);
            std::lock_guard<std::mutex> lk(g_target.mtx);
            out_px = g_target.px; out_py = g_target.py;
        };
        const float alpha = std::min(0.90f, PRED_ALPHA0 * 10.0f / PRED_DT0);
        // fx = −s·Δcounts + α·(观测 + s·Δcounts)
        const auto pred = [&](float s, float counts, float obs) {
            return -s * counts + alpha * (obs + s * counts);
        };
        float px1, py1, px2, py2;
        est_case(SPD_BASE, px1, py1);
        est_case(SPD_BASE * 2, px2, py2);                   // spd=200 → 有效灵敏度减半
        g_spd_x.store(SPD_BASE); g_spd_y.store(SPD_BASE);
        std::printf("      估计器 s=%.2f → (%.3f,%.3f); s=%.2f → (%.3f,%.3f); 手算 (%.3f,%.3f)/(%.3f,%.3f)\n",
                    s100, px1, py1, s100 * 0.5f, px2, py2,
                    pred(s100, 10.0f, -8.0f), pred(s100, 4.0f, -3.0f),
                    pred(s100 * 0.5f, 10.0f, -8.0f), pred(s100 * 0.5f, 4.0f, -3.0f));
        CHECK(std::fabs(px1 - pred(s100, 10.0f, -8.0f)) < 1e-3f
              && std::fabs(py1 - pred(s100, 4.0f, -3.0f)) < 1e-3f,
              "估计器自身运动补偿: 预测减法的逐轴 s 与注入同一份 (发布值与手算相符)");
        CHECK(std::fabs(px2 - pred(s100 * 0.5f, 10.0f, -8.0f)) < 1e-3f
              && std::fabs(py2 - pred(s100 * 0.5f, 4.0f, -3.0f)) < 1e-3f,
              "估计器补偿随 spd 成比例变化 (与注入/在飞补偿同源)");
        CHECK(std::fabs((px2 - px1) - 10.0f * (1.0f - alpha) * (s100 * 0.5f)) < 1e-3f,
              "该轴变化量 = Δcounts·(1−α)·|Δs|: 刻度确实进到了预测减法, 而非只改了个变量");
    }

    std::cout << "[5] spd 夹取带 [SPD_MIN, SPD_MAX]\n";
    {
        CHECK(spd_clamp(0) == SPD_MIN && spd_clamp(-5) == SPD_MIN,
              "0 与负数 → 夹到下限 (防误输入)");
        CHECK(spd_clamp(SPD_MAX + 1) == SPD_MAX && spd_clamp(999999) == SPD_MAX,
              "超上限 → 夹到上限");
        CHECK(spd_clamp(100) == 100 && spd_clamp(105) == 105 && spd_clamp(2000) == 2000,
              "带内取值一一保留 (整数步进 105/109 的用法)");
        CHECK(s_hid_eff(5) == 20.0f && s_hid_eff(2000) == 0.05f,
              "有意义的带 5..2000 = 有效灵敏度 20–0.05 px/count (本库灵敏度设计带)");
        CHECK(s_hid_eff(SPD_BASE) == S_HID_BASE && gain_pad_eff(SPD_BASE) == GAIN_PAD_BASE,
              "100 = 基线: 有效值与 base 常量相等 (hid 1.0 px/count, pad 3000 px/s)");
    }

    std::cout << "[6] 热参路径 (spdx 下一拍生效; 非法值被拒绝)\n";
    {
        CHECK(hotctl_apply("spdx", "150") && g_spd_x.load() == 150,
              "spdx=150 被接受并写入");
        const double v = cmd_px(180.0, 0.0);
        int cx = 0, cy = 0;
        measure(180.0f, 0.0f, LEFT_KEY, cx, cy);
        CHECK(cx == counts_expected(v, S_HID_BASE * 100.0 / 150.0),
              "热参改 spdx 后下一拍即生效 (无需重启/重标定)");
        CHECK(hotctl_apply("adsspdy", "90") && g_ads_spd_y.load() == 90,
              "adsspdy 逐轴独立写入");
        g_spd_x.store(SPD_BASE);
        const int keep = g_spd_x.load();
        CHECK(!hotctl_apply("spdx", "abc") && g_spd_x.load() == keep,
              "spdx=abc 被拒绝且不改动现值 (非数值)");
        CHECK(!hotctl_apply("spdx", "1e3x") && g_spd_x.load() == keep,
              "spdx=1e3x 被拒绝 (整对消费, 尾随垃圾不收)");
        CHECK(!hotctl_apply("spd", "5") && !hotctl_apply("s", "5"),
              "未知 key (含已删除的 s 与拼错的 spd) 被拒绝");
        CHECK(hotctl_apply("spdx", "0") && g_spd_x.load() == SPD_MIN,
              "热参侧的越界值同样被夹取 (与 CLI 同一个 spd_clamp)");
        g_spd_x.store(SPD_BASE); g_ads_spd_y.store(SPD_BASE);
        CHECK(!hotctl_apply("aim", "yes") && !hotctl_apply("k", "up"),
              "既有键的非法值照旧被拒绝 (白名单语义未变)");
    }

    std::cout << "[7] 标定回写: 只写调用方给的那个延迟 VAR\n";
    {
        const std::string path = "/tmp/control_test_script.sh";
        { std::ofstream o(path, std::ios::trunc);
          o << "#!/bin/bash\nSPDX=105\nL_EST=60.0\nSPDY=110\n"; }
        ::chmod(path.c_str(), 0755);
        const auto slurp = [&]() {
            std::ifstream in(path);
            return std::string((std::istreambuf_iterator<char>(in)),
                               std::istreambuf_iterator<char>()); };
        const bool ok = persist_calibration(path, L_VAR_HID, 42.5f);
        const std::string all = slurp();
        struct stat st{};
        ::stat(path.c_str(), &st);
        CHECK(ok && all.find("L_EST=42.5") != std::string::npos,
              "L 写进调用方给的 VAR (L_VAR_HID = L_EST)");
        CHECK(all.find("SPDX=105") != std::string::npos
              && all.find("SPDY=110") != std::string::npos
              && all.find("S_EST") == std::string::npos,
              "速度倍率 VAR 逐字不变, 也没有任何速度 VAR 被写进脚本");
        CHECK((st.st_mode & 0777) == 0755, "原子替换保留执行位");
        const bool ok2 = persist_calibration(path, "L_EST_PAD", 77.0f);
        const std::string all2 = slurp();
        CHECK(ok2 && all2.find("L_EST_PAD=77.0") != std::string::npos,
              "换一个 VAR 名即另一套 (手柄输出的名字由调用方给), 追加而非覆盖");
        CHECK(all2.find("L_EST=42.5") != std::string::npos,
              "两套延迟 VAR 同时存在, 各自独立");
        ::unlink(path.c_str());
    }

    std::cout << (g_fail ? "FAILED\n" : "ALL PASS\n");
    return g_fail ? 1 : 0;
}
