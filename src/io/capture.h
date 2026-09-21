// ============================================================================
//  capture.h — AI 采集/推理线程: GStreamer (NV12→BGRx→BGR appsink) 采集 →
//    CUDA 预处理 → TensorRT 推理 → 目标筛选 → estimator_step 估计并发布
//    g_target (自身运动换算的账本来源与逐轴比例由估计器按输出模式取); 标定采样
//    (640 居中裁切 → 320 相关域 → 块一维投影相位相关) 与拟合/延迟回写 (-S, VAR 名由
//    调用方按输出模式给: HID_L_EST / PAD_L_EST / P5G_L_EST), 三源截图采集与预览绘制。
//    标定期整条推理链跳过 (标定不需要检测, 省下的算力付相关的账), 见 capture.cu。
// ============================================================================

#pragma once

#include <string>

#include "io/calib_run.h"      // CalMode (激励计划/注入单位) 与 CAL_VAR_* (回写 VAR 名)

void ai_thread(std::string model_path, int target_cls,
               std::string cam_dev, int cam_fps, bool preview,
               float init_l, std::string persist_path, const char* cal_var,
               CalMode cal_mode,
               std::string out_dir, int fire_ms, double auto_s,
               int cooldown_ms, int jpeg_quality);
