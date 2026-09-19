// ============================================================================
//  capture.h — AI 采集/推理线程: GStreamer (NV12→BGRx→BGR appsink) 采集 →
//    CUDA 预处理 → TensorRT 推理 → 目标筛选 → estimator_step 估计并发布
//    g_target; 标定相位相关采样与标定计算/回写 (-S, 灵敏度单位制与回写 VAR
//    名随输出模式: hid = px/count → S_EST/L_EST, pad = px per 偏转·ms →
//    PAD_STICK_GAIN_X/_Y 与 L_EST_PAD, 见 io/pad_calib.h), 三源截图采集与预览绘制。
// ============================================================================

#pragma once

#include <string>

void ai_thread(std::string model_path, int target_cls,
               std::string cam_dev, int cam_fps, bool preview,
               float init_s, float init_l, std::string persist_path,
               std::string out_dir, int fire_ms, double auto_s,
               int cooldown_ms, int jpeg_quality, bool pad_mode);
