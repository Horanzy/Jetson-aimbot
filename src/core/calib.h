// ============================================================================
//  calib.h — 环路延迟 L (ms) 的在线标定: counts 历史与背景位移的最小二乘 +
//    延迟粗/细双扫 (run_calibration; 拟合同时解出的灵敏度 s 只是该拟合的旁产物,
//    仅作诊断量打印 — 手感走 core/state.h 的 spd 倍率), 单 VAR 的脚本原子回写
//    (persist_calibration, VAR 名由调用方给出), 采集卡设备名解析
//    (resolve_cam_device), 以及标定的激励轨迹表 (CalibSeg — 由 core/control.cu
//    的状态机播放)。采样在 io/capture.cu (块相位相关, 不依赖 AI 检测)。
//  时长一律由墙钟导出 (ms_to_ticks, core/state.h), 拍数只是它的换算结果。
// ============================================================================

#pragma once

#include <chrono>
#include <deque>
#include <string>

#include "core/state.h"        // DEFAULT_FREQ / ms_to_ticks — 拍数一律按墙钟导出

// ========================= 标定 =========================
const int   CALIB_TRIGGER_TICKS    = ms_to_ticks(5000);   // 双侧键长按 5s
const int   CALIB_WINDOW           = 90;             // 最少样本帧数
const float CALIB_MIN_EXCITE       = 4000.0f;        // 最小 ΣC² 激发量
const float S_MIN = 0.05f, S_MAX = 20.0f;            // 灵敏度设计带 (px/count): 拟合出的
                                                     //   s 的合法带, 与 spd 的有效灵敏度
                                                     //   带同界 (core/state.h: spd 5..2000)
const float L_MIN = 0.0f,  L_MAX = 200.0f;
const int   CALIB_WAIT_TIMEOUT     = ms_to_ticks(2000);   // 等待标定计算回执超时 2s

// 轨迹段 = 每毫秒位移 (counts/ms) × 拍数 (由墙钟毫秒导出); 每拍实际注入的整数
//   counts 由余量量化得到 (与律自身的 rem += v·TICK_MS/s 同一套手法, 见
//   control.cu)。速度与段时长是两个物理量, 拍率只决定这条曲线被采样得多细:
//   段的总位移恒为 v×段毫秒数, 与拍率无关。段速度为设计量 (s=1 时 1 counts/ms
//   = 1 px/ms = 1000 px/s), 激励段取 2 counts/ms = 2000 px/s — 与默认速度帽同
//   量级, 即被测的正是瞄准会用到的速度段。
struct CalibSeg { float vx, vy; int ticks; };
// 起始方块: 纯视觉开始信号 (采样自激励段才开始, 本段不参与估计), 1.5 counts/ms
inline const CalibSeg CAL_START_SEQ[] = {
    {1.5f,0,ms_to_ticks(240)},{0,1.5f,ms_to_ticks(240)},
    {-1.5f,0,ms_to_ticks(240)},{0,-1.5f,ms_to_ticks(240)},{0,0,ms_to_ticks(500)}};
// 激励方波单圈基元: 每边 2 counts/ms × 250ms = 500 counts (control.cu 重复
//   CAL_EXCITE_LOOPS 圈) — 5 圈 ≈ 5s 激励, 120fps 下 ≈600 帧, 同时盖过直方图
//   的 300 帧容量与 CALIB_WINDOW 的最小窗口
inline const CalibSeg CAL_EXCITE_SEQ[] = {
    {2,0,ms_to_ticks(250)},{0,2,ms_to_ticks(250)},
    {-2,0,ms_to_ticks(250)},{0,-2,ms_to_ticks(250)}};
constexpr int CAL_EXCITE_LOOPS = 5;
// 静置段: 激励结束到回执之间留出的干净窗口
inline const CalibSeg CAL_SETTLE_SEQ[] = {{0,0,ms_to_ticks(300)}};
// 回执: 成功 = 点头, 失败 = 摇头 (4 counts/ms = 每程 240 counts 的短促甩动)
inline const CalibSeg CAL_END_OK_SEQ[] = {
    {0,4,ms_to_ticks(60)},{0,-4,ms_to_ticks(60)},{0,4,ms_to_ticks(60)},
    {0,-4,ms_to_ticks(60)},{0,4,ms_to_ticks(60)},{0,-4,ms_to_ticks(60)}
};
inline const CalibSeg CAL_END_FAIL_SEQ[] = {
    {4,0,ms_to_ticks(60)},{-4,0,ms_to_ticks(60)},{4,0,ms_to_ticks(60)},
    {-4,0,ms_to_ticks(60)},{4,0,ms_to_ticks(60)},{-4,0,ms_to_ticks(60)}
};

struct CalibSample { std::chrono::steady_clock::time_point t; float dt_ms,sx,sy; };

// 标定回写的唯一 VAR 名 (调用方给出): 每个输出模式一个 — hid 用 L_EST, 手柄输出
//   将用自己的名字。标定只写延迟; 速度倍率是手动项, 固件永不写。
const char* const L_VAR_HID = "L_EST";

// s_fit = 拟合联立解出的灵敏度 (px/count, 只作诊断量打印, 不写回也不参与手感);
//   l_est 进 = 延迟扫描起点, 出 = 标定结果 (ms)
bool run_calibration(const std::deque<CalibSample>& hist, float& s_fit, float& l_est);
bool persist_calibration(const std::string& path, const std::string& var, float l);
std::string resolve_cam_device(const std::string& spec);
