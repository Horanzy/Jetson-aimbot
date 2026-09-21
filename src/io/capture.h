// ============================================================================
//  capture.h — AI 采集/推理线程: GStreamer (NV12→BGRx→BGR appsink) 采集 →
//    CUDA 预处理 → TensorRT 推理 → 目标筛选 → estimator_step 估计并发布
//    g_target (自身运动换算的逐轴灵敏度按帧时刻的 ADS 状态取); 标定相位相关
//    采样与标定计算/延迟回写 (-S), 三源截图采集与预览绘制。
// ============================================================================

#pragma once

#include <string>

void ai_thread(std::string model_path, int target_cls,
               std::string cam_dev, int cam_fps, bool preview,
               float init_l, std::string persist_path,
               std::string out_dir, int fire_ms, double auto_s,
               int cooldown_ms, int jpeg_quality);
