// ============================================================================
//  calib.h — 灵敏度 s (px/count) 与环路延迟 L (ms) 的在线标定: 最小二乘估计 +
//    延迟粗/细双扫 (run_calibration), S_EST/L_EST 脚本原子回写
//    (persist_calibration), 采集卡设备名解析 (resolve_cam_device), 以及
//    CalibSeg 激励轨迹表 — 轨迹由 core/control.cu 的标定状态机播放, 采样在
//    io/capture.cu (块相位相关, 不依赖 AI 检测)。
// ============================================================================

#pragma once

#include <chrono>
#include <deque>
#include <string>

#include "core/state.h"        // DEFAULT_FREQ / ms_to_ticks — 拍数一律按墙钟导出

// ========================= 标定 (以拍计的时长由墙钟导出, 换拍率不改语义) =========================
const int   CALIB_TRIGGER_TICKS    = ms_to_ticks(5000);   // 双侧键长按 5s
const int   CALIB_WINDOW           = 90;             // 最少样本帧数
const float CALIB_MIN_EXCITE       = 4000.0f;        // 最小 ΣC² 激发量
const float S_MIN = 0.05f, S_MAX = 20.0f;
const float L_MIN = 0.0f,  L_MAX = 200.0f;
const int   CALIB_WAIT_TIMEOUT     = ms_to_ticks(2000);   // 等待计算超时 2s

struct CalibSeg { int dx, dy, ticks; };
// 激励轨迹: 每拍位移 (counts) × 拍数; 拍数由段墙钟时长导出, 段速度为设计量 —
//   激励方波 2px/拍 = 2000 counts/s (s=1 时即速度帽量级), 收尾甩动 4px/拍 =
//   4000 counts/s。起始方块是纯视觉开始信号 (采样自激励段才开始): 3px→1.5px
//   非整数, 取 2px/拍 与激励同速。
inline const CalibSeg CAL_START_SEQ[] = {
    {2,0,ms_to_ticks(240)},{0,2,ms_to_ticks(240)},{-2,0,ms_to_ticks(240)},{0,-2,ms_to_ticks(240)},
    {0,0,ms_to_ticks(500)}};
// 激励方波单圈基元: 每边 2px × 250ms = 500 counts (control.cu 重复 5 圈)
inline const CalibSeg CAL_EXCITE_SEQ[] = {
    {2,0,ms_to_ticks(250)},{0,2,ms_to_ticks(250)},{-2,0,ms_to_ticks(250)},{0,-2,ms_to_ticks(250)}};
inline const CalibSeg CAL_SETTLE_SEQ[] = {{0,0,ms_to_ticks(300)}};
inline const CalibSeg CAL_END_OK_SEQ[] = {
    {0,4,ms_to_ticks(60)},{0,-4,ms_to_ticks(60)},{0,4,ms_to_ticks(60)},
    {0,-4,ms_to_ticks(60)},{0,4,ms_to_ticks(60)},{0,-4,ms_to_ticks(60)}};
inline const CalibSeg CAL_END_FAIL_SEQ[] = {
    {4,0,ms_to_ticks(60)},{-4,0,ms_to_ticks(60)},{4,0,ms_to_ticks(60)},
    {-4,0,ms_to_ticks(60)},{4,0,ms_to_ticks(60)},{-4,0,ms_to_ticks(60)}};

struct CalibSample { std::chrono::steady_clock::time_point t; float dt_ms,sx,sy; };

bool run_calibration(const std::deque<CalibSample>& hist, float& s_est, float& l_est);
bool persist_calibration(const std::string& path, float s, float l);
std::string resolve_cam_device(const std::string& spec);
