// ============================================================================
//  pad_output.cu — pad_output.h 的实现: 注入换算与合并钳制 (纯换算), 摇杆
//    账本入账, 模式路由选择子, 最终逻辑态发布点, pad 控制拍组装 (触发键位 →
//    律期望速度 → 合并 → 发布) 与 --pad-dump ≥50ms 节流打印。输出后端
//    (XInput over raw_gadget) 缝合在发布点上, 不进入本文件。
// ============================================================================

#include "io/pad_output.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "core/control.h"
#include "core/state.h"

CountsHistory g_pad_ledger;
PadPublishState g_pad_publish;

namespace { bool ledger_pad = false; }

void own_motion_ledger_set(bool pad) { ledger_pad = pad; }
const CountsHistory& own_motion_ledger() { return ledger_pad ? g_pad_ledger : g_counts; }

PadLogical pad_publish_snapshot(uint64_t* seq) {
    std::lock_guard<std::mutex> lk(g_pad_publish.mtx);
    if (seq) *seq = g_pad_publish.seq;
    return g_pad_publish.st;
}

PadLogical pad_merge(const PadLogical& human, float aim_vx, float aim_vy,
                     float stick_gain, std::chrono::steady_clock::time_point now) {
    // 拍时长 h = 距上拍的实际间隔 (timerfd 实际到期; 抖动/合并唤醒按实际时长
    //   入账 — 账本度量游戏真实收到的运动, 与 hid 按实际报文记账同口径)。
    //   首拍取 TICK_MS。
    static bool have_prev = false;
    static std::chrono::steady_clock::time_point prev{};
    float h = have_prev ? (float)elapsed_ms(now, prev) : TICK_MS;
    if (h < 0) h = 0;
    prev = now; have_prev = true;
    PadLogical out = human;
    float fx = aim_vx*1000.0f/stick_gain;    // px/ms → px/s → 满偏转比例
    float fy = aim_vy*1000.0f/stick_gain;
    out.rx = (int16_t)std::clamp((int)human.rx + (int)std::lround(std::clamp(fx,-1.0f,1.0f)*PAD_AXIS_MAX),
                                 -PAD_AXIS_MAX, PAD_AXIS_MAX);
    out.ry = (int16_t)std::clamp((int)human.ry + (int)std::lround(std::clamp(fy,-1.0f,1.0f)*PAD_AXIS_MAX),
                                 -PAD_AXIS_MAX, PAD_AXIS_MAX);
    g_pad_ledger.add(now, (int)std::lround(out.rx*h), (int)std::lround(out.ry*h));
    return out;
}

namespace {

// --pad-dump 节流: ≥50ms 一行 — 干跑日志与控制拍解耦的最小打印周期
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
    bool fire = h.rt != 0;                   // RT 有值 = fire, LT 有值 = ads — 与 hid 同一
    bool ads  = h.lt != 0;                   //   触发语义, 扳机模拟量 1:1 直映无阈值
    uint16_t btns = (uint16_t)((fire?LEFT_KEY:0) | (ads?RIGHT_KEY:0));
    float vx, vy;
    bool gate = control_apply_pad(cam_fps, btns, vx, vy);
    PadLogical out = pad_merge(h, vx, vy, PAD_STICK_GAIN_DEFAULT,
                               std::chrono::steady_clock::now());
    {   // 发布点覆盖写 (最新报告槽语义, 契约见 pad_output.h)
        std::lock_guard<std::mutex> lk(g_pad_publish.mtx);
        g_pad_publish.st = out;
        ++g_pad_publish.seq;
    }
    if (dump) pad_dump_line(out, fire, ads, gate);
}
