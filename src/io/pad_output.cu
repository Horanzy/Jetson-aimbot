// ============================================================================
//  pad_output.cu — pad_output.h 的实现: 注入换算与合并钳制 (纯换算), 标定激励期
//    整只手柄由激励独占 (右摇杆 = 激励, 人手摇杆/扳机归中, 按键照旧透传), 摇杆
//    账本入账, 模式路由选择子与账本→像素比例, 最终逻辑态发布点, pad 控制拍组装
//    (标定拍 → 触发键位 → 律期望速度 → 合并 → 发布) 与 --pad-dump 节流打印。
//    输出后端 (XInput over raw_gadget) 缝合在发布点上, 不进入本文件。
// ============================================================================
#include "io/pad_output.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "core/control.h"
#include "core/state.h"
#include "io/calib_run.h"

CountsHistory g_pad_ledger;
PadPublishState g_pad_publish;
std::atomic<float> g_pad_trig_thr{PAD_TRIG_THR_PCT};

namespace { bool ledger_pad = false; }

void own_motion_ledger_set(bool pad) { ledger_pad = pad; }
const CountsHistory& own_motion_ledger() { return ledger_pad ? g_pad_ledger : g_counts; }

LedgerPxScale own_motion_scale() {
    // 换算一律用该轴的**有效**增益 (基数 × 100/spd_axis): 账本记的是实发偏转/
    //   counts, 像素换算必须与注入同口径 — 否则 spd 放大命令多少, 自身运动补偿
    //   就错多少。ADS 状态在此取一次, 两轴同源。
    const bool ads = ads_down();
    if (!ledger_pad) return { s_hid_now(ads, 0), s_hid_now(ads, 1) };
    return { pad_s_rp_from_gain(gain_pad_eff(spd_axis(ads, 0))),
             pad_s_rp_from_gain(gain_pad_eff(spd_axis(ads, 1))) };
}

PadLogical pad_publish_snapshot(uint64_t* seq) {
    std::lock_guard<std::mutex> lk(g_pad_publish.mtx);
    if (seq) *seq = g_pad_publish.seq;
    return g_pad_publish.st;
}

namespace {

// 拍时长 h = 距上拍的实际间隔 (timerfd 实际到期; 抖动/合并唤醒按实际时长入账 —
//   账本度量游戏真实收到的运动, 与 hid 按实际报文记账同口径)。首拍取 TICK_MS。
float tick_h(std::chrono::steady_clock::time_point now) {
    static bool have_prev = false;
    static std::chrono::steady_clock::time_point prev{};
    float h = have_prev ? (float)elapsed_ms(now, prev) : TICK_MS;
    if (h < 0) h = 0;
    prev = now; have_prev = true;
    return h;
}

// 入账: 本拍游戏侧实收的右摇杆偏转 × 拍时长 (偏转·ms)。"账本 × s_rp" 就是按
//   该轴换算系数折算的像素运动 (s_rp 由该轴有效满偏屏速给出, 与注入换算互为逆)
//   — 自身运动补偿的口径因此与应用给游戏的那条命令一致。
void ledger_add(const PadLogical& out, std::chrono::steady_clock::time_point now) {
    float h = tick_h(now);
    g_pad_ledger.add(now, (int)std::lround((float)out.rx * h),
                          (int)std::lround((float)out.ry * h));
}

} // namespace

// 行程形状 = 圆 (径向), 实测驱动: G7 Pro 的合成幅度被限制在半径 32767 的圆内 —
//   四方向单轴可达 ±32767, 对角两轴各约 0.71 满偏; 上机采样 max|(rx,ry)|
//   ≈ 33074 (= 32767×1.009, 设备固件自身的径向限幅加约 1% 容差), 无任何样本
//   出现逐轴同时满偏 (方形会给出 √2 倍幅度)。故合并与注入的几何一律径向:
//   - 注入向量 (律的速度指令) 先径向缩放到 |r| ≤ 1: 律要的是方向 + 速度, 逐轴
//     钳制会在对角方向改写方向 (自瞄方向即由此失真); 律本身对本函数一无所知
//     (独立算速度), 现实在此处收口;
//   - 合并向量 (人类通道 + 注入) 再径向缩放到 |·| ≤ 满偏 — 与设备/引擎的行程
//     形状一致 (把方形对角喂给圆形行程, 引擎仍会径向压回并改写方向);
//   - 账本按最终提交值入账: 自身运动补偿的口径 = 游戏实收 (对角方向若按逐轴满偏
//     记账会高估最多 √2 倍, 直接污染估计器)。
PadLogical pad_merge(const PadLogical& human, float aim_vx, float aim_vy,
                     float gain_x, float gain_y, std::chrono::steady_clock::time_point now) {
    PadLogical out = human;
    // 注入换算 = 线性满偏比例 d = v/A_eff (io/pad_output.h 的 pad_defl_ratio)。
    //   游戏响应曲线是凸的时 (实测 COD p≈2.7), 这条线性换算在**工作点**上由 spd
    //   整定补回来 (spd 与 A_eff 成反比, 于是命令→偏转的斜率可任意拧), 但响应
    //   不再是命令的线性函数: 大命令偏大、小命令偏小 — 这是放弃曲线模型的代价,
    //   现场由 spdx/spdy 在主要交战距离开度上定住工作点。
    float fx = pad_defl_ratio(aim_vx, gain_x);
    float fy = pad_defl_ratio(aim_vy, gain_y);
    float r = std::hypot(fx, fy);
    if (r > 1.0f) { fx /= r; fy /= r; }      // 注入向量径向限幅 (方向保持)
    float mx = (float)human.rx + fx*PAD_AXIS_MAX;
    float my = (float)human.ry + fy*PAD_AXIS_MAX;
    float m = std::hypot(mx, my);
    if (m > (float)PAD_AXIS_MAX) {           // 合并向量径向限幅 (方向保持)
        float k = (float)PAD_AXIS_MAX / m;
        mx *= k; my *= k;
    }
    out.rx = (int16_t)std::lround(mx);
    out.ry = (int16_t)std::lround(my);
    ledger_add(out, now);
    return out;
}

// 标定激励: **整只手柄由程序独占** — 右摇杆 = 激励偏转, 左摇杆与两扳机一律置中,
//   键位直通 (L3/R3 必须到游戏才能长按触发; 见 pad_output.h)。为什么整只手柄: 被测
//   的是"摇杆偏转 → 屏幕位移"这条响应, 而人手通道会改变这条响应本身而非只叠加运动
//   — 左摇杆让角色走动 = 整幅画面平移 (直接偏置行程累计), 扳机可能把游戏置于瞄准镜/
//   开火状态 (灵敏度被缩放、后坐让画面持续漂移)。两者都不进停顿窗 (停顿窗只看命令,
//   命令是零, 它们与命令无关), 所以噪声底抓不到它们 — 结构性置中才是"测量期间人手
//   不参与"的保证, 而不是对操作者的要求。
PadLogical pad_excite(const PadLogical& human, int16_t dx, int16_t dy,
                      std::chrono::steady_clock::time_point now) {
    PadLogical out{};
    out.btns=human.btns;
    out.rx=dx; out.ry=dy;
    ledger_add(out,now);
    return out;
}

namespace {

// --pad-dump 节流: ≥50ms 一行 — 调试日志与控制拍解耦的最小打印周期
const int PAD_DUMP_PERIOD_MS = 50;

void pad_dump_line(const PadLogical& p, bool fire, bool ads, bool gate) {
    static auto last = std::chrono::steady_clock::now() - std::chrono::hours(1);
    auto now = std::chrono::steady_clock::now();
    if (elapsed_ms(now, last) < PAD_DUMP_PERIOD_MS) return;
    last = now;
    printf("[PAD] lx=%d ly=%d rx=%d ry=%d lt=%u rt=%u btns=0x%04x fire=%d ads=%d aim_gate=%d\n",
           (int)p.lx, (int)p.ly, (int)p.rx, (int)p.ry,
           (unsigned)p.lt, (unsigned)p.rt, (unsigned)p.btns,
           fire?1:0, ads?1:0, gate?1:0);
    fflush(stdout);
}

} // namespace

void pad_tick(int cam_fps, PadState& in, bool dump) {
    PadLogical h = pad_input_snapshot(in);
    // 触发判定: RT≥阈值 = fire, LT≥阈值 = ads — 两键共享 g_pad_trig_thr (% 满
    //   量程), **只有判定用阈值, 模拟量 1:1 直映** (h.lt/h.rt 原值进合并与发布点)。
    const int thr = pad_trig_thr_counts(g_pad_trig_thr.load());
    const bool fire = h.rt >= thr;           // 与 hid 同一触发语义 (-k fire/ads/both)
    const bool ads  = h.lt >= thr;
    const uint16_t btns = (uint16_t)((fire?LEFT_KEY:0) | (ads?RIGHT_KEY:0));
    auto now = std::chrono::steady_clock::now();
    // 标定拍 (L3+R3 长按 / 热参 padcalib): 激励期律本拍不参与, 右摇杆由激励独占
    const CalStep cal = cal_step(CAL_MODE_PAD, h.btns, cam_fps, now);
    PadLogical out;
    bool gate = false;
    if (cal.active) out = pad_excite(h, cal.cx, cal.cy, now);
    else {
        float vx = 0, vy = 0;
        gate = control_apply_pad(cam_fps, btns, vx, vy);
        // spd 落点: 注入换算逐轴用有效满偏屏速 A_eff = 基数×100/spd_axis (调大 =
        //   偏转更大 = 更快); 速度帽逐轴取 min(-x, A_eff/1000), 在律里收口 (满偏行程
        //   是物理上限)。合并与账本按同一 A_eff 的口径 — 律刚把本拍的 ADS 键状态写进
        //   g_ads_down, 与上面的 ads 同值 (LT ≥ 阈值)。
        out = pad_merge(h, vx, vy,
                        gain_pad_eff(spd_axis(ads, 0)),
                        gain_pad_eff(spd_axis(ads, 1)), now);
    }
    {   // 发布点覆盖写 (最新报告槽语义, 契约见 pad_output.h)
        std::lock_guard<std::mutex> lk(g_pad_publish.mtx);
        g_pad_publish.st = out;
        ++g_pad_publish.seq;
    }
    if (dump) pad_dump_line(out, fire, ads, gate);
}
