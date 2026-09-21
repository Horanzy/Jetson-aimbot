// ============================================================================
//  estimator.h — 目标状态估计: α-β 位置/速度滤波 (Smith 预测器的估计侧, 增益按
//    实测 dt 归一) + 方向矛盾 CUSUM 速度归零 + 创新均值 â 加速度通道。由采集
//    线程逐帧驱动 (estimator_step), 估计结果经 TargetState 发布给控制拍
//    (DEFAULT_FREQ) tick; 每帧返回实测 dt (标定采样复用)。
// ============================================================================

#pragma once

#include <chrono>

#include "core/control.h"

// 滤波器跨帧状态 (采集线程持有, 每帧经 estimator_step 推进)
struct EstimatorState {
    bool filt_init=false;
    float fx=0,fy=0,fvx=0,fvy=0;
    float sig2x=1,sig2y=1,csx=0,csy=0;   // CUSUM 状态 (σ 自标定, 归零重拉)
    float sig2rx=1,sig2ry=1;             // â 传感器稳健尺度 σ̂_r (清洗创新二阶矩)
    float ybar_x=0,ybar_y=0;             // 创新均值 EMA ȳ (â 传感器)
    float ax_e=0,ay_e=0;                 // 加速度估计 â (0 = 未通过显著性/重建/门控)
    bool reb_x=true,reb_y=true;          // 重建抑制旗标 (v̂ 自 0 重建期 â 无定义)
    std::chrono::steady_clock::time_point reb_until_x=std::chrono::steady_clock::time_point::min(),
                                          reb_until_y=std::chrono::steady_clock::time_point::min();
    float last_dt=PRED_DT0,last_alpha=PRED_ALPHA0,last_beta=PRED_BETA0;
    std::chrono::steady_clock::time_point t_prev=std::chrono::steady_clock::now();
};

// 推进一帧: dt 归一滤波更新 (含 CUSUM/â) 与 g_target 发布; 返回本帧实测 dt (ms,
// 钳制到 1–100)。found/best_dx/best_dy 为本帧目标筛选结果; s_x/s_y 为该轴的有效
// 灵敏度 (px/count, state.h 的 s_hid_now) — 自身运动换算逐轴, 与注入同一份值。
float estimator_step(EstimatorState& st, std::chrono::steady_clock::time_point now,
                     bool found, float best_dx, float best_dy,
                     float s_x, float s_y, float l_est, float max_v);
