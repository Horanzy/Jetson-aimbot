// ============================================================================
//  trt.cu — trt.h 的实现: Logger 实例, BGR→RGB CHW 预处理 CUDA kernel
//    (float/half 双版, 与其启动封装同编译单元, 免 -rdc) 与 NMS。
// ============================================================================

#include "core/trt.h"

#include <map>

#include <cuda_fp16.h>

Logger gLogger;

__global__ void preprocessFloatKernel(const uint8_t* __restrict__ bgr,
                                      float* __restrict__ out, int total) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= total) return;
    const float sc = 1.0f / 255.0f;
    int i3 = idx * 3;
    out[idx]           = bgr[i3+2] * sc;
    out[total + idx]   = bgr[i3+1] * sc;
    out[2*total + idx] = bgr[i3]   * sc;
}
__global__ void preprocessHalfKernel(const uint8_t* __restrict__ bgr,
                                     __half* __restrict__ out, int total) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= total) return;
    const float sc = 1.0f / 255.0f;
    int i3 = idx * 3;
    out[idx]           = __float2half(bgr[i3+2] * sc);
    out[total + idx]   = __float2half(bgr[i3+1] * sc);
    out[2*total + idx] = __float2half(bgr[i3]   * sc);
}
void launchPreprocess(const uint8_t* d_bgr, void* d_out, int w, int h,
                      bool is_float, cudaStream_t stream) {
    int total = w * h;
    int threads = 256, blocks = (total + threads - 1) / threads;
    if (is_float)
        preprocessFloatKernel<<<blocks, threads, 0, stream>>>(d_bgr, (float*)d_out, total);
    else
        preprocessHalfKernel<<<blocks, threads, 0, stream>>>(d_bgr, (__half*)d_out, total);
}

static float iou(const Detection& a, const Detection& b) {
    float ax1=a.cx-a.w*.5f,ay1=a.cy-a.h*.5f,ax2=a.cx+a.w*.5f,ay2=a.cy+a.h*.5f;
    float bx1=b.cx-b.w*.5f,by1=b.cy-b.h*.5f,bx2=b.cx+b.w*.5f,by2=b.cy+b.h*.5f;
    float ix1=std::max(ax1,bx1),iy1=std::max(ay1,by1),ix2=std::min(ax2,bx2),iy2=std::min(ay2,by2);
    if (ix2<=ix1||iy2<=iy1) return 0;
    return (ix2-ix1)*(iy2-iy1)/(a.w*a.h+b.w*b.h-(ix2-ix1)*(iy2-iy1));
}
std::vector<Detection> nms(const std::vector<Detection>& dets, float thr) {
    std::vector<Detection> out;
    std::map<int,std::vector<Detection>> per;
    for (auto& d:dets) per[d.class_id].push_back(d);
    for (auto& [cls,ds]:per) {
        std::sort(ds.begin(),ds.end(),[](auto&a,auto&b){return a.conf>b.conf;});
        std::vector<bool> sup(ds.size(),false);
        for (size_t i=0;i<ds.size();++i) { if (sup[i]) continue; out.push_back(ds[i]);
            for (size_t j=i+1;j<ds.size();++j) if (!sup[j]&&iou(ds[i],ds[j])>thr) sup[j]=true; } }
    return out;
}
