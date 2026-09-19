// ============================================================================
//  pad_calib.h — 手柄标定 (pad 模式) 的触发、激励轨迹与量纲换算。与 hid 的
//    双侧键标定完全独立: 分开触发 (L3+R3 长按 / 热参 padcalib), 分开存放
//    (PAD_STICK_GAIN / L_EST_PAD), 分开回写 (同一原子回写机制, VAR 名不同),
//    自成一态 (状态机由 1kHz pad 拍驱动, 见 io/pad_output.cu 的 pad_tick)。
//
//  标定量 (最小集):
//    L    (ms)     与 hid 同一物理含义: 注入 → 屏幕 → 采集 → 检测 的环路延迟
//    gain (px/s)   满偏转屏速: 右摇杆满偏 (±32767) 对应的准星屏速 — 注入换算
//                  v·1000/gain 的分子分母来源, 也是 pad 有效速度帽的来源
//
//  测量方法 = hid 标定的同一链路: 标定期固件独占右摇杆, 播放满偏方波激励
//    (画正方形同构), 采样沿 g_calib_collect → io/capture.cu 的块相位相关 →
//    账本 (own_motion_ledger 路由到摇杆账本) 上的最小二乘 + 延迟粗/细双扫
//    (core/calib.cu 的 run_calibration, 数学与单位制无关)。pad 的账本单位是
//    偏转·ms, 故灵敏度 s_rp 的单位是 px per (偏转·ms); 乘 PAD_AXIS_MAX×1000
//    即满偏屏速 px/s (单位换算见 io/pad_output.h 的 pad_gain_from_s_rp)。
// ============================================================================

#pragma once

#include <chrono>
#include <cstdint>

#include "core/calib.h"                  // CalibSeg / CalibBand / run_calibration
#include "core/state.h"                  // ms_to_ticks — 拍数一律按墙钟导出
#include "io/pad_input.h"                // PadBtn / PAD_AXIS_MAX
#include "io/pad_output.h"               // PAD_GAIN_MIN/MAX 与 s_rp ↔ 满偏屏速换算

// L3+R3 长按触发 (与 hid 的 CALIB_TRIGGER_TICKS 同一纪律: 拍数按墙钟导出)
const int PAD_CAL_TRIGGER_TICKS = ms_to_ticks(5000);

// 回写 VAR 名 (脚本 VAR 块; hid 侧为 S_EST/L_EST, 两套互不覆盖)
constexpr const char* PAD_CAL_VAR_GAIN = "PAD_STICK_GAIN";
constexpr const char* PAD_CAL_VAR_L    = "L_EST_PAD";

// ---- 激励轨迹 (设计量: 幅度 = 满偏转, 段时长与 hid 标定同设计值) ----
// 幅度取满偏转: 标定前摇杆增益未知, 唯一"标定前已知"的幅度就是全偏转 — 与
//   hid 的 2 counts/拍 同一处境 (其屏速同样要标定出 s 才知道)。
// 段时长沿用 hid 的设计值 (边界 240/250ms, 静置 300ms, 收尾 60ms): 采样窗
//   (~2.5s @120fps) 与单边时长、块相位相关的帧间位移量程同尺度。
// 每边位移 = 满偏转 × 段时长 (deflection·ms), 屏移 = 段时长 × 满偏屏速。
inline const CalibSeg PAD_CAL_START_SEQ[] = {          // 起始方块 (纯视觉开始信号)
    {PAD_AXIS_MAX,0,ms_to_ticks(240)},{0,PAD_AXIS_MAX,ms_to_ticks(240)},
    {-PAD_AXIS_MAX,0,ms_to_ticks(240)},{0,-PAD_AXIS_MAX,ms_to_ticks(240)},
    {0,0,ms_to_ticks(500)}};
inline const CalibSeg PAD_CAL_EXCITE_SEQ[] = {         // 激励方波单圈基元 (播 5 圈)
    {PAD_AXIS_MAX,0,ms_to_ticks(250)},{0,PAD_AXIS_MAX,ms_to_ticks(250)},
    {-PAD_AXIS_MAX,0,ms_to_ticks(250)},{0,-PAD_AXIS_MAX,ms_to_ticks(250)}};
inline const CalibSeg PAD_CAL_SETTLE_SEQ[] = {{0,0,ms_to_ticks(300)}};
inline const CalibSeg PAD_CAL_END_OK_SEQ[] = {         // 成功 = 纵向点头 2 次
    {0,PAD_AXIS_MAX,ms_to_ticks(60)},{0,-PAD_AXIS_MAX,ms_to_ticks(60)},
    {0,PAD_AXIS_MAX,ms_to_ticks(60)},{0,-PAD_AXIS_MAX,ms_to_ticks(60)},
    {0,PAD_AXIS_MAX,ms_to_ticks(60)},{0,-PAD_AXIS_MAX,ms_to_ticks(60)}};
inline const CalibSeg PAD_CAL_END_FAIL_SEQ[] = {       // 失败 = 横向摇头
    {PAD_AXIS_MAX,0,ms_to_ticks(60)},{-PAD_AXIS_MAX,0,ms_to_ticks(60)},
    {PAD_AXIS_MAX,0,ms_to_ticks(60)},{-PAD_AXIS_MAX,0,ms_to_ticks(60)},
    {PAD_AXIS_MAX,0,ms_to_ticks(60)},{-PAD_AXIS_MAX,0,ms_to_ticks(60)}};

// pad 灵敏度钳制带 (run_calibration 用, 单位 = px per 偏转·ms): 由满偏屏速设计带
//   换算 (带的选择规则见 io/pad_output.h 的 PAD_GAIN_MIN/MAX)。
inline const CalibBand CALIB_BAND_PAD{
    pad_s_rp_from_gain(PAD_GAIN_MIN), pad_s_rp_from_gain(PAD_GAIN_MAX)};

// 拟合结果的带内判定: 钳制把越界的拟合停在带边, 故"落带内"等价于"拟合未越界"。
//   带外 = 激励/背景不可信 (例如画面不响应注入), 标定按失败收尾 — 带边垃圾值
//   绝不回写。hid 侧的带内钳制是既有行为 (S_MIN/S_MAX), 此处只约束 pad。
inline bool pad_calib_accept(float gain) {
    return gain > PAD_GAIN_MIN && gain < PAD_GAIN_MAX;
}

// 标定拍的一步: btns 为人类逻辑按键位表 (PadBtn), 返回值给出本拍是否处于标定中
//   及其右摇杆激励偏转 — active 时 pad_tick 以 pad_excite 独占右摇杆并跳过律。
struct PadCalibStep { bool active; int16_t dx, dy; };

// 状态机一步 (由 1kHz pad 拍每拍调用一次):
//   触发 = 人类 L3+R3 长按 PAD_CAL_TRIGGER_TICKS 拍, 或热参 padcalib=1 (一次
//   消费即清, 与 g_calib_request 同一 exchange 语义); 接管关闭 (-a n / aim=0)
//   时标定不可达且进行中的标定复位 (激励是程序注入的移动, 与纯透传互斥 —
//   与 hid 标定分支同一纪律)。相位/计时器是本模块私有 static, 与 hid 的状态
//   机 (core/control.cu 的 law_tick) 无共享。
PadCalibStep pad_calib_step(uint16_t btns);
