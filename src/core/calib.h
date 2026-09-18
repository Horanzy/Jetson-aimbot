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

// ========================= 标定 =========================
const int   CALIB_TRIGGER_TICKS    = 2500;           // 双侧键长按 5s @500Hz
const int   CALIB_WINDOW           = 90;             // 最少样本帧数
const float CALIB_MIN_EXCITE       = 4000.0f;        // 最小 ΣC² 激发量
const float S_MIN = 0.05f, S_MAX = 20.0f;
const float L_MIN = 0.0f,  L_MAX = 200.0f;
const int   CALIB_WAIT_TIMEOUT     = 1000;           // 等待计算超时 @500Hz

struct CalibSeg { int dx, dy, ticks; };
inline const CalibSeg CAL_START_SEQ[] = {{3,0,120},{0,3,120},{-3,0,120},{0,-3,120},{0,0,250}};
inline const CalibSeg CAL_SETTLE_SEQ[] = {{0,0,150}};
inline const CalibSeg CAL_END_OK_SEQ[] = {
    {0,8,30},{0,-8,30},{0,8,30},{0,-8,30},{0,8,30},{0,-8,30}
};
inline const CalibSeg CAL_END_FAIL_SEQ[] = {
    {8,0,30},{-8,0,30},{8,0,30},{-8,0,30},{8,0,30},{-8,0,30}
};

struct CalibSample { std::chrono::steady_clock::time_point t; float dt_ms,sx,sy; };

bool run_calibration(const std::deque<CalibSample>& hist, float& s_est, float& l_est);
bool persist_calibration(const std::string& path, float s, float l);
std::string resolve_cam_device(const std::string& spec);
