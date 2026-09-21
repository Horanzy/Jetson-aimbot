// ============================================================================
//  calib.h — 灵敏度 s (px/count) 与环路延迟 L (ms) 的在线标定: 最小二乘估计 +
//    延迟粗/细双扫 (run_calibration), S_EST/L_EST 脚本原子回写
//    (persist_calibration), 采集卡设备名解析 (resolve_cam_device), 以及标定的
//    激励轨迹表 (CalibSeg — 由 core/control.cu 的状态机播放)。采样在
//    io/capture.cu (块相位相关, 不依赖 AI 检测)。
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
const float S_MIN = 0.05f, S_MAX = 20.0f;
const float L_MIN = 0.0f,  L_MAX = 200.0f;
const int   CALIB_WAIT_TIMEOUT     = ms_to_ticks(2000);   // 等待标定计算回执超时 2s

// 轨迹段 = 每拍位移 (counts) × 拍数; 段速度为设计量 (counts/拍, s=1 时即屏幕
//   px/拍), 段时长由墙钟毫秒导出 —— 激励段的速度与默认速度帽同量级
//   (2 counts/ms = 2000 px/s, 帽默认 1500–2000), 即被测的正是瞄准会用到的速度段。
struct CalibSeg { int dx, dy, ticks; };
// 起始方块: 纯视觉开始信号 (采样自激励段才开始, 本段不参与估计), 1500 px/s
inline const CalibSeg CAL_START_SEQ[] = {
    {3,0,ms_to_ticks(240)},{0,3,ms_to_ticks(240)},
    {-3,0,ms_to_ticks(240)},{0,-3,ms_to_ticks(240)},{0,0,ms_to_ticks(500)}};
// 激励方波单圈基元: 每边 2 counts/ms × 250ms = 500 counts (control.cu 重复
//   CAL_EXCITE_LOOPS 圈) — 5 圈 ≈ 5s 激励, 120fps 下 ≈600 帧, 同时盖过直方图
//   的 300 帧容量与 CALIB_WINDOW 的最小窗口
inline const CalibSeg CAL_EXCITE_SEQ[] = {
    {4,0,ms_to_ticks(250)},{0,4,ms_to_ticks(250)},
    {-4,0,ms_to_ticks(250)},{0,-4,ms_to_ticks(250)}};
constexpr int CAL_EXCITE_LOOPS = 5;
// 静置段: 激励结束到回执之间留出的干净窗口
inline const CalibSeg CAL_SETTLE_SEQ[] = {{0,0,ms_to_ticks(300)}};
// 回执: 成功 = 点头, 失败 = 摇头 (8 counts/拍 = 每程 240 counts 的短促甩动)
inline const CalibSeg CAL_END_OK_SEQ[] = {
    {0,8,ms_to_ticks(60)},{0,-8,ms_to_ticks(60)},{0,8,ms_to_ticks(60)},
    {0,-8,ms_to_ticks(60)},{0,8,ms_to_ticks(60)},{0,-8,ms_to_ticks(60)}
};
inline const CalibSeg CAL_END_FAIL_SEQ[] = {
    {8,0,ms_to_ticks(60)},{-8,0,ms_to_ticks(60)},{8,0,ms_to_ticks(60)},
    {-8,0,ms_to_ticks(60)},{8,0,ms_to_ticks(60)},{-8,0,ms_to_ticks(60)}
};

struct CalibSample { std::chrono::steady_clock::time_point t; float dt_ms,sx,sy; };

bool run_calibration(const std::deque<CalibSample>& hist, float& s_est, float& l_est);
bool persist_calibration(const std::string& path, float s, float l);
std::string resolve_cam_device(const std::string& spec);
