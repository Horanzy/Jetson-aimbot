// ============================================================================
//  pad_output.h — pad 模式的合并层 (输入侧之后、XInput over raw_gadget 输出后端
//    之前): 控制律期望速度 (px/ms) → 右摇杆注入偏转换算 (逐轴满偏转屏速),
//    与人类通道合并 (行程形状 = 圆, 径向限幅; 实测依据见 pad_output.cu),
//    摇杆账本 Σ(合并偏转·拍时长), 自身运动账本的模式路由 (hid=g_counts /
//    pad=摇杆账本, 不变式 3 的消费端) 与账本→像素的每轴比例, 最终逻辑状态的发布点
//    (输出后端的接入契约, 见 PadPublishState), 以及 --pad-dump 干跑打印。pad 拍与
//    控制拍同一节拍 (拍率 = DEFAULT_FREQ)。
// ============================================================================

#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>

#include "core/control.h"                // LEFT_KEY/RIGHT_KEY (-k 触发语义)
#include "core/state.h"                  // CountsHistory (账本/路由的类型)
#include "io/pad_input.h"                // PadLogical/PadState

// 满偏转屏速缺省 (px/s): 摇杆打满 32767 对应的准星屏速。选择规则: AGENTS.md 速度帽
//   推导 — 行为库最快屏速 1.99 px/ms ≈ 2000 px/s 是"必须能跟上的下界", 缺省取同一
//   推导的上一档整值, 使控制律的正常速度指令不致打满摇杆。pad 有效速度帽 =
//   min(-x, 满偏转屏速) (law_tick 内强制)。
// 运行期每轴值经 -G x[,y] 给出并被 pad 标定逐轴回写覆盖 (g_pad_stick_gain_x/_y,
//   见 io/pad_calib.h); 本常量只是缺省与设计点 (标定值带的中心)。
const float PAD_STICK_GAIN_DEFAULT = 3000.0f;
// 满偏转屏速设计带 (px/s): 标定值的可信域 = 设计点的一个数量级。下界 300 = 物理
//   必需屏速 (2000 px/s) 的 1/10 — 比"必须能跟上的下界"还慢十倍的全偏转屏速
//   无法用于注入, 触发它只可能是测量失败; 上界 30000 = 设计点的 10 倍, 即
//   120fps 下 250px/帧的屏移 — 超出块相位相关的测量量程 (半分辨率块 106px)。
//   标定结果落带外按失败收尾, 不回写 (见 pad_calib_accept)。
const float PAD_GAIN_MIN = 300.0f, PAD_GAIN_MAX = 30000.0f;

// 运行期满偏转屏速 (px/s, 逐轴): 注入换算 (pad_merge) · pad 速度帽 (law_tick) ·
//   账本→像素比例 (own_motion_scale) 三处共用的唯一来源; 启动值 = -G x[,y] 或
//   PAD_STICK_GAIN_DEFAULT (均经设计带钳制), pad 标定成功后由 AI 线程
//   (io/capture.cu) 逐轴更新 — 立即对手柄拍生效。
extern std::atomic<float> g_pad_stick_gain_x, g_pad_stick_gain_y;

float pad_gain_clamp(float gain);
// 单位换算: 满偏屏速 (px/s) ↔ 摇杆账本单位制灵敏度 s_rp (px per 偏转·ms)。
//   两者是同一物理量的两种单位, 恒等式: s_rp × 32767 × 1000 = 满偏屏速。
inline float pad_s_rp_from_gain(float gain) {
    return gain / ((float)PAD_AXIS_MAX * 1000.0f);
}
inline float pad_gain_from_s_rp(float s_rp) {
    return s_rp * (float)PAD_AXIS_MAX * 1000.0f;
}

// 摇杆账本: 每 tick 游戏侧将收到的右摇杆合并偏转 (人类+注入, 径向限幅到满偏后)
//   × 实际拍时长 (偏转·ms), 结构与窗口同 CountsHistory。g_counts 不变式 3
//   ("记录游戏实际收到的全部 counts") 的 pad 对应物 — 估计器与控制律的自身
//   运动补偿经 own_motion_ledger 读它。
// 深度: pad 标定的账本必须覆盖整段分级激励窗 (计划最坏时长 8s, 见 io/pad_calib.h
//   的 PAD_CAL_PLAN_MS) 加每样本的延迟/帧长回溯 (L_MAX+帧长 ≈0.3s), 故比 hid 的
//   3s 窗深 — 该文件的 static_assert 把本深度与激励计划时长绑在一起。
const size_t PAD_LEDGER_TICKS = (size_t)10 * DEFAULT_FREQ;
extern CountsHistory g_pad_ledger;

// 自身运动账本的模式路由 (不变式 3 的消费端): hid = g_counts (px = s·Δcounts),
//   pad = 摇杆账本 (px = s_rp·Δ(偏转·ms))。估计器 (创新清洗/自身活动门) 与控制律
//   (in-flight 补偿) 共用这一个来源选择, 两账本同为 CountsHistory。模式由 main
//   启动时设定 (两模式单次运行只居其一), 缺省 hid。
void own_motion_ledger_set(bool pad);
const CountsHistory& own_motion_ledger();

// 账本 → 像素 的每轴比例 (自身运动补偿的唯一换算来源; 消费端数学两模式同形:
//   px = 比例 × 账本增量): hid = 标定 s (px/count, 两轴同值 — 鼠标只有一个灵敏度);
//   pad = 各轴满偏屏速换算 px per 偏转·ms = gain_axis/(32767·1000) (只有各轴分离
//   才能使账本换算与注入换算互为逆, 否则 Y 轴系统性偏差)。运行时按轴取原
//   子, pad 标定成功后立即生效。
struct LedgerPxScale { float x, y; };
LedgerPxScale own_motion_scale(float s_hid);

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

// 合并: fx = v_x·1000/gain_x、fy = v_y·1000/gain_y 为逐轴满偏转比例 (v: px/ms →
//   px/s), 注入偏转 = 比例×32767, 与人类 rx/ry 相加后按圆形行程径向钳制 (见
//   pad_output.cu 的实测依据); 其余字段逐位直通 (全透传: 按键/左摇杆 1:1, 扳机
//   模拟量 1:1 无阈值)。账本按合并偏转×实际拍时长入账 (拍内偏转为分段常数,
//   CountsHistory 线性插值因而精确)。
PadLogical pad_merge(const PadLogical& human, float aim_vx, float aim_vy,
                     float gain_x, float gain_y, std::chrono::steady_clock::time_point now);

// 标定激励的右摇杆注入 (pad 标定独占该轴, 见 io/pad_calib.h): 输出右摇杆 = 激励
//   偏转 (该级的比例×满偏, 单轴) 本身, 人类右摇杆被忽略 (人类若同时推杆会污染
//   激励), 其余字段逐位直通人类态。与 pad_merge 同一入账路径 — 拟合的账本就是
//   这条账本。
PadLogical pad_excite(const PadLogical& human, int16_t dx, int16_t dy,
                      std::chrono::steady_clock::time_point now);

// pad 控制拍 (main 主循环调用, 拍率 = DEFAULT_FREQ): 人类态快照 → pad 标定拍
//   (io/pad_calib.h: L3+R3 长按或热参触发; 标定期独占右摇杆并跳过律) → 否则
//   RT/LT 触发键位字 (fire→LEFT_KEY, ads→RIGHT_KEY, 复用律的 -k 语义与
//   KEEP_ALIVE 窗) → control_apply_pad 取期望速度 → pad_merge 合并+账本 →
//   发布点覆盖写 → --pad-dump 节流打印。输出后端只消费发布点, 不进入本函数。
void pad_tick(int cam_fps, PadState& in, bool dump);
