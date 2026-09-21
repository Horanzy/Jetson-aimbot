// ============================================================================
//  pad_output.h — 手柄输出模式 (pad) 的合并层 (输入侧之后、输出后端之前):
//    控制律期望速度 (px/ms) → 右摇杆注入偏转换算 (逐轴有效满偏屏速), 与人类
//    通道合并 (行程形状 = 圆, 径向限幅; 实测依据见 pad_output.cu), 摇杆账本
//    Σ(合并偏转·拍时长), 自身运动账本的模式路由 (hid = g_counts / pad = 摇杆
//    账本, 不变式 3 的消费端) 与账本→像素的逐轴比例, 最终逻辑状态的发布点
//    (输出后端的接入契约, 见 PadPublishState), 以及 --pad-dump 调试打印。
//    pad 拍与控制拍同一节拍 (拍率 = DEFAULT_FREQ)。
// ============================================================================

#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>

#include "core/control.h"                // LEFT_KEY/RIGHT_KEY (-k 触发语义)
#include "core/state.h"                  // spd_axis/gain_pad_eff/CountsHistory
#include "io/pad_input.h"                // PadLogical/PadState/PAD_AXIS_MAX

// 满偏比例换算: 期望屏幕速度 (px/ms) → 带符号的摇杆满偏比例 d = v/A_eff
//   (A_eff = 该轴有效满偏屏速 px/s = gain_pad_eff(spd_axis(ads, axis)), 见
//   core/state.h)。返回值可以 >1 (命令速度超过该轴满偏屏速) — 由调用方的
//   径向钳制收口: 那是设备/引擎的行程形状, 不在这里改写方向。
inline float pad_defl_ratio(float v_px_per_ms, float gain_px_s) {
    if (!(gain_px_s > 0.0f)) return 0.0f;
    return v_px_per_ms * 1000.0f / gain_px_s;
}

// 账本单位制灵敏度: 满偏屏速 (px/s) → 每 (偏转·ms) 的像素数 s_rp = A/(32767×1000)。
//   它是同一物理量的另一种单位 (恒等式 s_rp × 32767 × 1000 = 满偏屏速), 也是
//   摇杆账本 → 像素的唯一换算来源 — 与注入换算互为逆, 故账本 × s_rp 恰好是
//   应用给游戏的那条命令的像素运动。
inline float pad_s_rp_from_gain(float gain_px_s) {
    return gain_px_s / ((float)PAD_AXIS_MAX * 1000.0f);
}

// 触发阈值 (% 满量程): RT ≥ 阈值 = fire, LT ≥ 阈值 = ads — 两键**共享同一个
//   阈值**, 由 -T <百分比> 给出初值、热参 padthr 运行中可调。阈值只作用于"是否
//   触发自瞄"的判定; 扳机的**模拟量仍 1:1 直映** (透传路径不加阈值)。
//   缺省 6% 的出处: 该手柄扳机轴实测 flat (内核抗抖带) 15/255 ≈ 5.9%, 上取整到
//   6% — "移出抗抖带"即"这一下按压是有意的"的最小判据, 不是调出来的数。
//   0% 退化为"任何非零即触发", 100% = 必须拉到底。
const float PAD_TRIG_THR_PCT = 6.0f;
extern std::atomic<float> g_pad_trig_thr;              // 单位: % (0–100)

// 阈值 % → 判定门 (满量程 255): 0% 映射到 1 (等价"任何非零即触发"), 100% 映射到
//   满量程; round 后钳在 [1,255] 保证两端都不退化。纯函数 — 单测直接断言边界。
inline int pad_trig_thr_counts(float pct) {
    return std::clamp((int)std::lround(pct * 255.0f / 100.0f), 1, 255);
}

// 摇杆账本: 每拍游戏侧收到的右摇杆合并偏转 (人类 + 注入, 径向限幅到满偏后) ×
//   实际拍时长 (偏转·ms), 结构与回溯深度同 CountsHistory (3s 墙钟: 只服务控制律的
//   在飞补偿与估计器的自身运动门, 两者回溯 ≤ 0.3s)。g_counts 不变式 3 ("记录游戏
//   实际收到的全部 counts") 的 pad 对应物 — 估计器与控制律的自身运动补偿经
//   own_motion_ledger 读它。
extern CountsHistory g_pad_ledger;

// 自身运动账本的模式路由 (不变式 3 的消费端): hid = g_counts (px = s·Δcounts),
//   pad = 摇杆账本 (px = s_rp·Δ(偏转·ms))。估计器 (预测减法/创新清洗/自身活动门)
//   与控制律 (在飞补偿) 共用这一个来源选择, 两账本同为 CountsHistory。模式由 main
//   启动时设定 (两模式单次运行只居其一), 缺省 hid。
void own_motion_ledger_set(bool pad);
const CountsHistory& own_motion_ledger();

// 账本 → 像素 的逐轴比例 (自身运动补偿的唯一换算来源; 消费端数学两模式同形:
//   px = 比例 × 账本增量)。hid = 该轴有效灵敏度 s_hid_now(ads, axis) (px/count);
//   pad = 该轴有效满偏屏速的换算 px per 偏转·ms = pad_s_rp_from_gain(gain_pad_eff)。
//   逐轴是必须的 — 两轴的 spd 可以不同, 一个因子会让 Y 轴系统性偏差。
//   ADS 键状态与注入换算取自同一时刻的 g_ads_down (拍内自洽)。
struct LedgerPxScale { float x, y; };
LedgerPxScale own_motion_scale();

// 发布点: 合并后的最终逻辑手柄态 — 输出后端 (XInput over raw_gadget) 的接入
//   契约:
//   - pad_tick 每控制拍 (DEFAULT_FREQ) 互斥覆盖写最新槽并递增 seq — "最新报告
//     槽" 语义, 与 io/usbraw 的 submit 相同: 不排队, 慢消费者读到的永远是最新
//     合并态;
//   - 后端以自身发送节拍轮询 pad_publish_snapshot (返回状态拷贝, 可带出 seq);
//     seq 未变 = 无新帧, 后端可跳过重发;
//   - 摇杆账本已按发布内容入账 (后端转发本槽即"游戏实际收到", 不变式 3 的度量
//     在合并层完成) — 后端不得再改写摇杆值, 否则账本失真;
//   - 字段语义: 摇杆 int16 ±32767 (上/左为负), 扳机 uint8 0–255 模拟量, btns 为
//     PadBtn 位表 — 后端负责映射到自己的设备协议, 不回写。
struct PadPublishState {
    std::mutex mtx;
    PadLogical st{};
    uint64_t seq = 0;                     // 单调递增, 每拍 +1
};
extern PadPublishState g_pad_publish;
PadLogical pad_publish_snapshot(uint64_t* seq = nullptr);

// 合并: 注入偏转 = pad_defl_ratio 按该轴有效满偏屏速线性换算出的满偏比例
//   ×32767, 与人类 rx/ry 相加后按圆形行程径向钳制 (见 pad_output.cu 的实测
//   依据); 其余字段逐位直通 (全透传: 按键/左摇杆 1:1, 扳机模拟量 1:1 无阈值)。
//   账本按合并偏转 × 实际拍时长入账 (拍内偏转为分段常数, CountsHistory 线性
//   插值因而精确)。
PadLogical pad_merge(const PadLogical& human, float aim_vx, float aim_vy,
                     float gain_x, float gain_y, std::chrono::steady_clock::time_point now);

// pad 控制拍 (main 主循环调用, 拍率 = DEFAULT_FREQ): 人类态快照 → RT/LT 触发
//   键位字 (fire→LEFT_KEY, ads→RIGHT_KEY, 复用律的 -k 语义与 KEEP_ALIVE 窗) →
//   标定拍 (io/calib_run.h 的状态机; 激励期由 pad_excite 独占整只手柄) 或
//   律取期望速度 → pad_merge 合并+账本 → 发布点覆盖写 → --pad-dump 节流打印。
//   输出后端只消费发布点, 不进入本函数。
void pad_tick(int cam_fps, PadState& in, bool dump);

// 标定激励: 整只手柄由程序独占 — 右摇杆 = 激励偏转 (dx/dy, 逻辑量程), 左摇杆与两
//   扳机置中, 按键照旧透传 (L3/R3 才到得了游戏)。理据见 pad_output.cu: 人手通道会
//   改变被测量的那条响应本身 (走动 = 整幅画面平移; 扳机 = 瞄准镜/开火状态), 而不是
//   只叠加一份运动, 且它们与命令无关, 停顿窗的噪声底抓不到。账本按激励偏转入账。
PadLogical pad_excite(const PadLogical& human, int16_t dx, int16_t dy,
                      std::chrono::steady_clock::time_point now);
