// ============================================================================
//  trt.h — TensorRT 推理辅助: Logger 与 CUDA 错误检查, 模型输入预处理
//    (BGR→RGB CHW, float/half 双版 CUDA kernel, 见 trt.cu), 输出张量布局解析
//    与 NMS。
// ============================================================================

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <vector>

#include <NvInfer.h>
#include <cuda_runtime_api.h>

class Logger : public nvinfer1::ILogger {
public:
    void log(Severity sev, const char* msg) noexcept override {
        if (sev <= Severity::kWARNING) std::cout << "[TensorRT]: " << msg << "\n";
    }
};
extern Logger gLogger;
struct TRTDestroy { template<class T> void operator()(T* p) const { delete p; } };
#define CHECK_CUDA(call) do {                                         \
    cudaError_t e = (call);                                           \
    if (e != cudaSuccess) {                                           \
        std::cerr << "CUDA Error: " << cudaGetErrorString(e)          \
                  << " at " << __FILE__ << ":" << __LINE__ << "\n";   \
        std::exit(EXIT_FAILURE);                                      \
    }                                                                 \
} while (0)

inline size_t elemSize(nvinfer1::DataType dt) {
    switch (dt) {
        case nvinfer1::DataType::kFLOAT: return 4; case nvinfer1::DataType::kHALF: return 2;
        case nvinfer1::DataType::kINT8: return 1;  case nvinfer1::DataType::kINT32: return 4;
        case nvinfer1::DataType::kBOOL: return 1;  default: return 0;
    }
}
inline size_t volume(const nvinfer1::Dims& d) {
    size_t v=1; for (int i=0;i<d.nbDims;++i) v*=d.d[i]; return v;
}
inline bool hasDynamicDim(const nvinfer1::Dims& d) {
    for (int i=0;i<d.nbDims;++i) if (d.d[i]<0) return true; return false;
}

struct OutputLayout { int attrs=0,num=0; bool attrs_first=true; };
inline bool parseOutputLayout(const nvinfer1::Dims& d, OutputLayout& l) {
    if (d.nbDims==3 && d.d[0]==1) { int a=d.d[1],b=d.d[2];
        l.attrs=std::min(a,b); l.num=std::max(a,b); l.attrs_first=(a<=b); return true; }
    if (d.nbDims==2) { int a=d.d[0],b=d.d[1];
        l.attrs=std::min(a,b); l.num=std::max(a,b); l.attrs_first=(a<=b); return true; }
    return false;
}
inline float outVal(const float* o, const OutputLayout& l, int a, int i) {
    return l.attrs_first ? o[a*l.num+i] : o[i*l.attrs+a];
}
struct Detection { float cx,cy,w,h,conf; int class_id; };
std::vector<Detection> nms(const std::vector<Detection>& dets, float thr);

// BGR (HWC) → RGB CHW 归一化预处理; is_float 选 float/half 输出布局
void launchPreprocess(const uint8_t* d_bgr, void* d_out, int w, int h,
                      bool is_float, cudaStream_t stream);
