// ============================================================================
//  pad_output.h — pad 模式的合并层 (输入侧之后、XInput over raw_gadget 输出后端
//    之前): 控制律期望速度 (px/ms) → 右摇杆注入偏转换算, 与人类通道合并钳制,
//    摇杆账本 Σ(合并偏转·拍时长), 自身运动账本的模式路由 (hid=g_counts /
//    pad=摇杆账本, 不变式 3 的消费端), 最终逻辑状态的发布点 (输出后端的接入
//    契约, 见 PadPublishState), 以及 --pad-dump 干跑打印。pad 拍与控制拍同一
//    节拍 (拍率 = DEFAULT_FREQ)。
// ============================================================================

#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>

#include "core/control.h"                // LEFT_KEY/RIGHT_KEY (-k 触发语义)
#include "core/state.h"                  // CountsHistory (账本/路由的类型)
#include "io/pad_input.h"                // PadLogical/PadState

// 满偏转屏速 (px/s): 摇杆打满 32767 对应的准星屏速。选择规则: AGENTS.md 速度帽
//   推导 — 行为库最快屏速 1.99 px/ms ≈ 2000 px/s 是"必须能跟上的下界",
//   缺省取同一推导的上一档整值, 使控制律的正常速度指令不致打满摇杆。pad 有效
//   速度帽 = min(-x, 本值) (law_tick 内强制)。pad 模式的 s = 本值/(32767·1000)
//   px per (偏转·ms) — 摇杆账本单位制下的自身运动换算系数。
const float PAD_STICK_GAIN_DEFAULT = 3000.0f;

// 摇杆账本: 每 tick 游戏侧将收到的右摇杆合并偏转 (人类+注入, ±32767 钳制后)
//   × 实际拍时长 (偏转·ms), 结构与窗口同 CountsHistory。g_counts 不变式 3
//   ("记录游戏实际收到的全部 counts") 的 pad 对应物 — 估计器与控制律的自身
//   运动补偿经 own_motion_ledger 读它。
extern CountsHistory g_pad_ledger;

// 自身运动账本的模式路由 (不变式 3 的消费端): hid = g_counts (px = s·Δcounts),
//   pad = 摇杆账本 (px = s_rp·Δ(偏转·ms), s_rp = stick_gain/(32767·1000))。
//   估计器 (创新清洗/自身活动门) 与控制律 (in-flight 补偿) 共用这一个来源
//   选择, 两账本同为 CountsHistory, 消费端数学逐字符一致。模式由 main 启动
//   时设定 (两模式单次运行只居其一), 缺省 hid。
void own_motion_ledger_set(bool pad);
const CountsHistory& own_motion_ledger();

// 发布点: 合并后的最终逻辑手柄态 — 输出后端 (XInput over raw_gadget) 的接入
//   契约:
//   - pad_tick 每控制拍 (DEFAULT_FREQ) 互斥覆盖写最新槽并递增 seq — "最新
//     报告槽" 语义, 与 io/usbraw 的 submit 相同: 不排队, 慢消费者读到的永远
//     是最新合并态;
//   - 后端以自身发送节拍轮询 pad_publish_snapshot (返回状态拷贝, 可带出 seq);
//     seq 未变 = 无新帧, 后端可跳过重发;
//   - 摇杆账本已按发布内容入账 (后端转发本槽即"游戏实际收到", 不变式 3 的
//     度量在合并层完成) — 后端不得再改写摇杆值, 否则账本失真;
//   - 字段语义: 摇杆 int16 ±32767 (上/左为负), 扳机 uint8 0–255 模拟量,
//     btns 为 PadBtn 位表 — 后端负责映射到自己的设备协议, 不回写。
struct PadPublishState {
    std::mutex mtx;
    PadLogical st{};
    uint64_t seq = 0;                     // 单调递增, 每拍 +1
};
extern PadPublishState g_pad_publish;
PadLogical pad_publish_snapshot(uint64_t* seq = nullptr);

// 合并: rs_aim = v·1000/stick_gain 为满偏转比例 (v: px/ms → px/s), 注入偏转
//   = 比例×32767, 与人类 rx/ry 相加后 ±32767 钳制; 其余字段逐位直通 (全透传:
//   按键/左摇杆 1:1, 扳机模拟量 1:1 无阈值)。账本按合并偏转×实际拍时长入账
//   (拍内偏转为分段常数, CountsHistory 线性插值因而精确)。
PadLogical pad_merge(const PadLogical& human, float aim_vx, float aim_vy,
                     float stick_gain, std::chrono::steady_clock::time_point now);

// pad 控制拍 (main 主循环调用, 拍率 = DEFAULT_FREQ): 人类态快照 → RT/LT 触发
//   键位字 (fire→LEFT_KEY, ads→RIGHT_KEY, 复用律的 -k 语义与 KEEP_ALIVE 窗)
//   → control_apply_pad 取期望速度 → pad_merge 合并+账本 → 发布点覆盖写 →
//   --pad-dump 节流打印。输出后端只消费发布点, 不进入本函数。
void pad_tick(int cam_fps, PadState& in, bool dump);
