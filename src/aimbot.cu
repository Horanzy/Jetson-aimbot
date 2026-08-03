// ============================================================================
//  aimbot.cu — AI 视觉自瞄 (鼠标透传式) + 可选训练数据采集
//
//  链路: 采集卡 (UVC 1080p NV12, -d 按名字选择) → GStreamer nvvidconv
//        → CUDA 预处理 → TensorRT YOLO 检测 → alpha-beta 目标跟踪
//        → 控制律 (极点配置 PI + type-2 速度前馈) → USB Gadget 透传
//
//  控制律: 收敛带宽 wn 由标定延迟 L 自动导出 (wn=(90°−PM)π/180/L, PM=60°, 免手调),
//    ζ=1 临界阻尼; type-2 速度前馈 (ff_gain=1) 补匀速跟踪零拖尾。结构参数为头部常量,
//    详见 arena/laws/ff_pi.py 与 AGENTS.md。
//
//  采集 (可选): 传 -o 输出目录即开启, 按三源触发自动截图 (开火 / 检测 / 定时),
//    截图 = 模型输入同款中心裁剪, 按来源分子目录, 异步写盘不阻塞推理。不传 -o 则纯自瞄。
//
//  标定: 双侧键长按 5 秒, 程序自动生成激励轨迹 (画正方形), 块相位相关测背景位移,
//    最小二乘估计灵敏度 s (px/count) + 环路延迟 L (ms); 经 -S 传入脚本路径时自动回写。
// ============================================================================

#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <array>
#include <deque>
#include <functional>
#include <cstring>
#include <cctype>
#include <climits>
#include <cfloat>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <signal.h>
#include <stdlib.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <sys/stat.h>
#include <linux/input.h>
#include <chrono>
#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <map>
#include <random>
#include <fstream>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <cuda_fp16.h>

// ========================= 系统常量 =========================
constexpr size_t HID_REPORT_LEN  = 9;
constexpr int    DEFAULT_FREQ    = 500;
constexpr const char* DEFAULT_KEYWORD  = "Logitech";
constexpr const char* DEFAULT_VIRT_DEV = "/dev/hidg0";
constexpr const char* DEV_SEARCH_PATH  = "/dev/input/by-id/";

const float FOV_RADIUS     = 150.0f;
const int   KEEP_ALIVE_MS  = 200;
const int   CAP_SIZE       = 640;

const uint16_t LEFT_KEY  = 0x01;
const uint16_t RIGHT_KEY = 0x02;
const uint16_t SIDE_KEY  = 0x10;
const uint16_t SIDE_KEY2 = 0x08;
const uint16_t BOTH_SIDE_KEYS = SIDE_KEY | SIDE_KEY2;

// ========================= 跟踪滤波器 (Smith 预测器, dt 归一) =========================
const float PRED_ALPHA0   = 0.50f;                   // 位置增益 @120fps; 实际 α=ALPHA0·dt/DT0 (帧率无关)
const float PRED_BETA0    = 0.04f;                   // 速度增益 @120fps; 实际 β=BETA0·dt/DT0
const float PRED_L_COMP   = 1.10f;                   // Smith 过补偿系数 (>1 帮欠补偿侧 L真>L̂, 危险方向)
const float PRED_DT0      = 1000.0f / 120.0f;        // 增益归一参考帧周期
const float TRACK_JUMP_GATE = 100.0f;                // 创新超此值 → 重置滤波器
const float TARGET_STALE_MS = 200.0f;                // 目标超时 → 暂停自瞄

// ========================= 控制律: 极点配置 PI + type-2 速度前馈 =========================
//  带宽由标定延迟 L 导出 (免手调); 前馈补跟踪速度。详见 arena/laws/ff_pi.py 与 AGENTS.md。
const float FF_PM_DEG = 60.0f;                       // 相位裕度: wn=(90°−PM)π/180/L (无量纲设计选择)
const float FF_ZETA   = 1.0f;                        // 收敛阻尼比 (临界阻尼, 无过冲)
const float FF_GAIN_VAL   = 1.0f;                        // 速度前馈增益: =1 是匀速目标零拖尾的精确开环指令
const float FF_I_GATE = 8.0f;                        // 收敛区门控 / I 距离衰减尺度 (px)
const float FF_I_FRAC = 1.0f;                        // 积分限幅 = I_FRAC×max_v/Ki

// ========================= 标定 =========================
const int   CALIB_TRIGGER_TICKS    = 2500;
const int   CALIB_WINDOW           = 90;
const float CALIB_MIN_EXCITE       = 4000.0f;
const float S_MIN = 0.05f, S_MAX = 20.0f;
const float L_MIN = 0.0f,  L_MAX = 200.0f;
const int   CALIB_WAIT_TIMEOUT     = 1000;

const float TICK_MS = 1000.0f / DEFAULT_FREQ;

// ========================= 采集 =========================
const size_t SAVE_QUEUE_MAX = 8;

struct CalibSeg { int dx, dy, ticks; };
static const CalibSeg CAL_START_SEQ[] = {{3,0,120},{0,3,120},{-3,0,120},{0,-3,120},{0,0,250}};
static const CalibSeg CAL_SETTLE_SEQ[] = {{0,0,150}};
static const CalibSeg CAL_END_OK_SEQ[] = {
    {0,8,30},{0,-8,30},{0,8,30},{0,-8,30},{0,8,30},{0,-8,30}
};
static const CalibSeg CAL_END_FAIL_SEQ[] = {
    {8,0,30},{-8,0,30},{8,0,30},{-8,0,30},{8,0,30},{-8,0,30}
};

// ========================= 全局状态 =========================
std::atomic<bool> global_running{true};
void signal_handler(int) { global_running = false; }

std::atomic<bool> g_calib_collect{false};
std::atomic<bool> g_calib_request{false};
std::atomic<int>  g_calib_done{0};

std::atomic<bool> g_left_down{false};

// ---- 时间辅助 ----
static inline std::chrono::steady_clock::time_point shift_ms(
    std::chrono::steady_clock::time_point t, double ms) {
    return t + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
               std::chrono::duration<double, std::milli>(ms));
}
static inline double elapsed_ms(
    std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b) {
    return std::chrono::duration<double, std::milli>(a - b).count();
}

// ---- 目标状态 ----
struct TargetState {
    float px = 0, py = 0, vx = 0, vy = 0;
    bool  valid = false;
    std::chrono::steady_clock::time_point t_pub;
    float s_est = 1.0f, l_est_ms = 60.0f;
    std::mutex mtx;
};
TargetState g_target;

// ---- counts 历史 ----
class CountsHistory {
public:
    struct Sample { std::chrono::steady_clock::time_point t; long long cx, cy; };
    void add(std::chrono::steady_clock::time_point t, int dx, int dy) {
        std::lock_guard<std::mutex> lk(mtx);
        cum_x += dx; cum_y += dy;
        buf.push_back({t, cum_x, cum_y});
        while (buf.size() > 1500) buf.pop_front();
    }
    std::pair<double,double> at(std::chrono::steady_clock::time_point t) const {
        std::lock_guard<std::mutex> lk(mtx);
        if (buf.empty()) return {0,0};
        if (t <= buf.front().t) return {(double)buf.front().cx, (double)buf.front().cy};
        if (t >= buf.back().t)  return {(double)buf.back().cx,  (double)buf.back().cy};
        size_t lo = 0, hi = buf.size() - 1;
        while (hi - lo > 1) { size_t m=(lo+hi)/2; if (buf[m].t<=t) lo=m; else hi=m; }
        const Sample &a=buf[lo], &b=buf[hi];
        double span = elapsed_ms(b.t, a.t);
        double f = span > 0 ? elapsed_ms(t, a.t) / span : 0;
        return {a.cx + (b.cx-a.cx)*f, a.cy + (b.cy-a.cy)*f};
    }
    std::pair<long long,long long> cum() const {
        std::lock_guard<std::mutex> lk(mtx); return {cum_x, cum_y};
    }
private:
    mutable std::mutex mtx;
    std::deque<Sample> buf;
    long long cum_x = 0, cum_y = 0;
};
CountsHistory g_counts;

struct MouseState {
    int32_t rel_x=0, rel_y=0, rel_wheel=0, rel_hwheel=0;
    uint16_t buttons = 0;
    std::mutex mtx;
};

// ---- 异步写盘队列 ----
struct SaveTask { cv::Mat img; std::string path; };
std::queue<SaveTask> g_save_q;
std::mutex g_save_mtx;
std::condition_variable g_save_cv;
std::atomic<long> g_dropped{0};

void enqueue_save(const std::string& path, const cv::Mat& frame) {
    { std::lock_guard<std::mutex> lk(g_save_mtx);
      if (g_save_q.size() >= SAVE_QUEUE_MAX) { ++g_dropped; return; }
      g_save_q.push({frame.clone(), path}); }
    g_save_cv.notify_one();
}
void writer_thread(int quality) {
    while (true) {
        SaveTask task;
        { std::unique_lock<std::mutex> lk(g_save_mtx);
          g_save_cv.wait(lk, []{ return !g_save_q.empty() || !global_running.load(); });
          if (g_save_q.empty()) break;
          task = std::move(g_save_q.front()); g_save_q.pop(); }
        cv::imwrite(task.path, task.img, {cv::IMWRITE_JPEG_QUALITY, quality});
    }
}
std::string make_filepath(const std::string& dir) {
    static std::atomic<unsigned long> seq{0};
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{}; localtime_r(&tt, &tm);
    char name[96];
    snprintf(name, sizeof(name), "%04d%02d%02d_%02d%02d%02d_%03d_%06lu.jpg",
             tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec,
             (int)ms.count(), seq++);
    return dir + "/" + name;
}
void ensure_dir(const std::string& d) {
    std::string cmd = "mkdir -p '" + d + "'";
    system(cmd.c_str());
}

// ========================= 命令行交互 =========================
std::string get_input_with_default(const std::string& prompt, const std::string& def) {
    std::cout << prompt << " [" << def << "]: ";
    std::string line; std::getline(std::cin, line);
    return line.empty() ? def : line;
}

// ========================= TensorRT 辅助 =========================
class Logger : public nvinfer1::ILogger {
public:
    void log(Severity sev, const char* msg) noexcept override {
        if (sev <= Severity::kWARNING) std::cout << "[TensorRT]: " << msg << "\n";
    }
} gLogger;
struct TRTDestroy { template<class T> void operator()(T* p) const { delete p; } };
#define CHECK_CUDA(call) do {                                         \
    cudaError_t e = (call);                                           \
    if (e != cudaSuccess) {                                           \
        std::cerr << "CUDA Error: " << cudaGetErrorString(e)          \
                  << " at " << __FILE__ << ":" << __LINE__ << "\n";   \
        std::exit(EXIT_FAILURE);                                      \
    }                                                                 \
} while (0)

static size_t elemSize(nvinfer1::DataType dt) {
    switch (dt) {
        case nvinfer1::DataType::kFLOAT: return 4; case nvinfer1::DataType::kHALF: return 2;
        case nvinfer1::DataType::kINT8: return 1;  case nvinfer1::DataType::kINT32: return 4;
        case nvinfer1::DataType::kBOOL: return 1;  default: return 0;
    }
}
static size_t volume(const nvinfer1::Dims& d) {
    size_t v=1; for (int i=0;i<d.nbDims;++i) v*=d.d[i]; return v;
}
static bool hasDynamicDim(const nvinfer1::Dims& d) {
    for (int i=0;i<d.nbDims;++i) if (d.d[i]<0) return true; return false;
}

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
static void launchPreprocess(const uint8_t* d_bgr, void* d_out, int w, int h,
                             bool is_float, cudaStream_t stream) {
    int total = w * h;
    int threads = 256, blocks = (total + threads - 1) / threads;
    if (is_float)
        preprocessFloatKernel<<<blocks, threads, 0, stream>>>(d_bgr, (float*)d_out, total);
    else
        preprocessHalfKernel<<<blocks, threads, 0, stream>>>(d_bgr, (__half*)d_out, total);
}

struct OutputLayout { int attrs=0,num=0; bool attrs_first=true; };
static bool parseOutputLayout(const nvinfer1::Dims& d, OutputLayout& l) {
    if (d.nbDims==3 && d.d[0]==1) { int a=d.d[1],b=d.d[2];
        l.attrs=std::min(a,b); l.num=std::max(a,b); l.attrs_first=(a<=b); return true; }
    if (d.nbDims==2) { int a=d.d[0],b=d.d[1];
        l.attrs=std::min(a,b); l.num=std::max(a,b); l.attrs_first=(a<=b); return true; }
    return false;
}
static inline float outVal(const float* o, const OutputLayout& l, int a, int i) {
    return l.attrs_first ? o[a*l.num+i] : o[i*l.attrs+a];
}
struct Detection { float cx,cy,w,h,conf; int class_id; };
static float iou(const Detection& a, const Detection& b) {
    float ax1=a.cx-a.w*.5f,ay1=a.cy-a.h*.5f,ax2=a.cx+a.w*.5f,ay2=a.cy+a.h*.5f;
    float bx1=b.cx-b.w*.5f,by1=b.cy-b.h*.5f,bx2=b.cx+b.w*.5f,by2=b.cy+b.h*.5f;
    float ix1=std::max(ax1,bx1),iy1=std::max(ay1,by1),ix2=std::min(ax2,bx2),iy2=std::min(ay2,by2);
    if (ix2<=ix1||iy2<=iy1) return 0;
    return (ix2-ix1)*(iy2-iy1)/(a.w*a.h+b.w*b.h-(ix2-ix1)*(iy2-iy1));
}
static std::vector<Detection> nms(const std::vector<Detection>& dets, float thr) {
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

// ========================= 标定 =========================
struct CalibSample { std::chrono::steady_clock::time_point t; float dt_ms,sx,sy; };
static bool run_calibration(const std::deque<CalibSample>& hist, float& s_est, float& l_est) {
    const int n=(int)hist.size(); if (n<CALIB_WINDOW) return false;
    auto scan=[&](float lo,float hi,float step,float& out_s,float& out_dl)->float {
        float best=FLT_MAX;
        for (float dl=lo;dl<=hi;dl+=step) {
            double lag=l_est+dl, sum_cc=0, sum_sc=0;
            std::vector<std::pair<float,float>> cs(n);
            for (int i=0;i<n;++i) { auto&r=hist[i];
                auto c0=g_counts.at(shift_ms(r.t,-lag-r.dt_ms));
                auto c1=g_counts.at(shift_ms(r.t,-lag));
                float Cx=(float)(c1.first-c0.first), Cy=(float)(c1.second-c0.second);
                cs[i]={Cx,Cy}; sum_cc+=(double)Cx*Cx+(double)Cy*Cy;
                sum_sc+=(double)r.sx*Cx+(double)r.sy*Cy; }
            if (sum_cc<CALIB_MIN_EXCITE) continue;
            double s_hat=sum_sc/sum_cc, score=0;
            for (int i=0;i<n;++i) { double ex=hist[i].sx-s_hat*cs[i].first,
                                         ey=hist[i].sy-s_hat*cs[i].second; score+=ex*ex+ey*ey; }
            score/=sum_cc;
            if (score<best) { best=(float)score; out_s=(float)s_hat; out_dl=dl; } }
        return best;
    };
    float s1=0,dl1=0;
    if (scan(-40.0f,96.0f,8.0f,s1,dl1)==FLT_MAX) return false;
    float s2=s1,dl2=dl1;
    if (scan(dl1-8.0f,dl1+8.0f,2.0f,s2,dl2)==FLT_MAX) { s2=s1; dl2=dl1; }
    s_est=std::clamp(s2,S_MIN,S_MAX); l_est=std::clamp(l_est+dl2,L_MIN,L_MAX);
    return true;
}
static bool persist_calibration(const std::string& path, float s, float l) {
    std::ifstream in(path); if (!in.good()) return false;
    std::vector<std::string> lines; std::string line;
    while (std::getline(in,line)) lines.push_back(line); in.close();
    char sbuf[64],lbuf[64];
    snprintf(sbuf,sizeof(sbuf),"S_EST=%.4f",s); snprintf(lbuf,sizeof(lbuf),"L_EST=%.1f",l);
    bool fs=false,fl=false;
    for (auto& ln:lines) { if (ln.rfind("S_EST=",0)==0){ln=sbuf;fs=true;}
                           else if (ln.rfind("L_EST=",0)==0){ln=lbuf;fl=true;} }
    if (!fs) lines.push_back(sbuf); if (!fl) lines.push_back(lbuf);
    struct stat st{}; bool have=(stat(path.c_str(),&st)==0);
    std::string tmp=path+".tmp";
    { std::ofstream o(tmp,std::ios::trunc); if (!o.good()) return false;
      for (auto& ln:lines) o<<ln<<"\n"; }
    if (have) { chmod(tmp.c_str(),st.st_mode); chown(tmp.c_str(),st.st_uid,st.st_gid); }
    if (rename(tmp.c_str(),path.c_str())!=0) { unlink(tmp.c_str()); return false; }
    return true;
}

static std::string resolve_cam_device(const std::string& spec) {
    if (spec.rfind("/dev/",0)==0) return spec;
    std::string key=spec;
    std::transform(key.begin(),key.end(),key.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    std::vector<std::string> avail,hits;
    if (DIR* dp=opendir("/dev/v4l/by-id")) {
        while (dirent* e=readdir(dp)) {
            std::string nm=e->d_name;
            const std::string suf="-video-index0";
            if (nm.size()<=suf.size()
                || nm.compare(nm.size()-suf.size(),suf.size(),suf)!=0) continue;
            avail.push_back(nm);
            std::string low=nm;
            std::transform(low.begin(),low.end(),low.begin(),
                           [](unsigned char c){ return std::tolower(c); });
            if (low.find(key)!=std::string::npos) hits.push_back(nm);
        }
        closedir(dp);
    }
    if (hits.size()!=1) {
        std::cerr<<"❌ 采集卡 \""<<spec<<"\" "
                 <<(hits.empty()?"没有匹配":"匹配到多个")<<", 可选:\n";
        for (auto& a:avail) std::cerr<<"   "<<a<<"\n";
        return "";
    }
    std::string link="/dev/v4l/by-id/"+hits[0];
    char resolved[PATH_MAX];
    if (realpath(link.c_str(),resolved)) return resolved;
    return link;
}

// ========================= AI 推理 + 采集线程 =========================
void ai_thread(std::string model_path, float conf_thr, int target_cls,
               float y_off_pct, std::string cam_dev, int cam_fps, bool preview,
               float init_s, float init_l, std::string persist_path,
               std::string out_dir, int fire_ms, double auto_s,
               int cooldown_ms, int jpeg_quality) {
    const int cam_w=1920, cam_h=1080;
    const float nms_iou_thr=0.45f;
    const bool collecting_enabled = !out_dir.empty();

    std::ifstream ef(model_path, std::ios::binary);
    if (!ef.good()) { std::cerr<<"AI: 无法打开模型\n"; global_running=false; return; }
    ef.seekg(0,std::ios::end); size_t esz=ef.tellg(); ef.seekg(0,std::ios::beg);
    std::vector<char> edata(esz); ef.read(edata.data(),esz); ef.close();

    std::unique_ptr<nvinfer1::IRuntime,TRTDestroy> rt(nvinfer1::createInferRuntime(gLogger));
    std::unique_ptr<nvinfer1::ICudaEngine,TRTDestroy> eng(rt->deserializeCudaEngine(edata.data(),esz));
    std::unique_ptr<nvinfer1::IExecutionContext,TRTDestroy> ctx(eng->createExecutionContext());

    int nio=eng->getNbIOTensors(); std::string in_name,out_name;
    for (int i=0;i<nio;++i) { const char* nm=eng->getIOTensorName(i);
        auto m=eng->getTensorIOMode(nm);
        if (m==nvinfer1::TensorIOMode::kINPUT&&in_name.empty()) in_name=nm;
        if (m==nvinfer1::TensorIOMode::kOUTPUT&&out_name.empty()) out_name=nm; }

    auto in_dims=eng->getTensorShape(in_name.c_str());
    auto in_dt=eng->getTensorDataType(in_name.c_str());
    auto out_dt=eng->getTensorDataType(out_name.c_str());
    if (hasDynamicDim(in_dims)) {
        auto opt=eng->getProfileShape(in_name.c_str(),0,nvinfer1::OptProfileSelector::kOPT);
        ctx->setInputShape(in_name.c_str(),opt); }
    auto real_in=ctx->getTensorShape(in_name.c_str());
    int iw=real_in.d[3], ih=real_in.d[2];
    int cap_w=std::max(CAP_SIZE,iw), cap_h=std::max(CAP_SIZE,ih);
    float cx0=cap_w/2.0f, cy0=cap_h/2.0f;

    auto out_dims=ctx->getTensorShape(out_name.c_str());
    if (volume(out_dims)==0) out_dims=eng->getTensorShape(out_name.c_str());
    OutputLayout ol; parseOutputLayout(out_dims,ol);
    long G=(long)(iw/8)*(ih/8)+(long)(iw/16)*(ih/16)+(long)(iw/32)*(ih/32);
    bool end2end=(ol.attrs==6&&!ol.attrs_first&&ol.num==300);
    bool yolov5=(!end2end&&ol.num==3*G);
    int nclass=ol.attrs-(yolov5?5:4);
    std::cout<<"模型: "<<(end2end?"端到端":yolov5?"YOLOv5":"YOLOv8/11")
             <<" "<<iw<<"x"<<ih;
    if (!end2end) std::cout<<" "<<nclass<<"类";
    std::cout<<"\n";

    size_t ivol=volume(real_in),ovol=volume(out_dims);
    size_t ibytes=ivol*elemSize(in_dt),obytes=ovol*elemSize(out_dt);
    size_t bgr_bytes=(size_t)cap_w*cap_h*3;
    void *d_in=nullptr,*d_out=nullptr,*d_bgr=nullptr,*d_model_bgr=nullptr,*h_out=nullptr;
    CHECK_CUDA(cudaMalloc(&d_in,ibytes)); CHECK_CUDA(cudaMalloc(&d_out,obytes));
    CHECK_CUDA(cudaMalloc(&d_bgr,bgr_bytes));
    bool need_crop=(iw!=cap_w||ih!=cap_h);
    if (need_crop) CHECK_CUDA(cudaMalloc(&d_model_bgr,(size_t)iw*ih*3));
    else d_model_bgr=d_bgr;
    int m_off_x=(cap_w-iw)/2, m_off_y=(cap_h-ih)/2;
    CHECK_CUDA(cudaHostAlloc(&h_out,obytes,cudaHostAllocDefault));
    std::vector<float> ofbuf; if (out_dt==nvinfer1::DataType::kHALF) ofbuf.resize(ovol);
    cudaStream_t stream; CHECK_CUDA(cudaStreamCreate(&stream));
    ctx->setTensorAddress(in_name.c_str(),d_in);
    ctx->setTensorAddress(out_name.c_str(),d_out);

    int crop_x=(cam_w-cap_w)/2, crop_y=(cam_h-cap_h)/2;
    std::string base="v4l2src device="+cam_dev+" ! video/x-raw,format=NV12,width="
        +std::to_string(cam_w)+",height="+std::to_string(cam_h)+",framerate="
        +std::to_string(cam_fps)+"/1 ! ";
    std::string full_pipe=base+"nvvidconv ! video/x-raw,format=BGRx ! videoconvert "
        "! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false";
    std::string crop_pipe=base+"nvvidconv left="+std::to_string(crop_x)
        +" right="+std::to_string(crop_x+cap_w)+" top="+std::to_string(crop_y)
        +" bottom="+std::to_string(crop_y+cap_h)+" ! video/x-raw,width="+std::to_string(cap_w)
        +",height="+std::to_string(cap_h)+",format=BGRx ! videoconvert "
        "! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false";

    cv::VideoCapture cap;
    cap.open(preview?full_pipe:crop_pipe, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) { std::cerr<<"无法打开摄像头\n"; global_running=false; return; }
    if (preview) cv::namedWindow("Aimbot",cv::WINDOW_AUTOSIZE);
    std::cout<<"✅ AI 线程已启动 ("<<cam_fps<<" fps, "<<cam_dev<<")\n";

    bool filt_init=false; float fx=0,fy=0,fvx=0,fvy=0;
    auto t_prev=std::chrono::steady_clock::now();

    float s_est=init_s, l_est=init_l;
    std::deque<CalibSample> hist; bool was_collecting=false;
    int collect_frames=0; double collect_resp=0;
    int bs_w=cap_w/6, bs_h=cap_h/6;
    cv::Mat hann; cv::createHanningWindow(hann,cv::Size(bs_w,bs_h),CV_32F);
    cv::Mat prev_gray_f;

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> jitter(0.5,1.5);
    auto roll_auto=[&](){ return std::chrono::milliseconds((long)(auto_s*1000.0*(jitter(rng)))); };
    auto t_start=std::chrono::steady_clock::now();
    auto next_auto=t_start+roll_auto();
    auto last_save=t_start-std::chrono::hours(1);
    auto last_fire=t_start-std::chrono::hours(1);
    long n_fire=0, n_det=0, n_auto=0;

    cv::Mat frame, cap_img;
    int read_fails=0;
    long fps_cnt=0; auto fps_t0=std::chrono::steady_clock::now();

    while (global_running) {
        if (!cap.read(frame)) {
            if (++read_fails>30) { std::cerr<<"AI: 采集卡断开\n"; global_running=false; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); continue; }
        read_fails=0;

        ++fps_cnt;
        auto fps_now=std::chrono::steady_clock::now();
        double fps_dt=std::chrono::duration<double>(fps_now-fps_t0).count();
        if (fps_dt>=60.0) { std::cout<<"[AI FPS] "<<(int)(fps_cnt/fps_dt)<<" fps (上限 "
                            <<cam_fps<<")\n"; fps_cnt=0; fps_t0=fps_now; }

        cap_img=(preview&&frame.cols>cap_w)?frame(cv::Rect(crop_x,crop_y,cap_w,cap_h)):frame;

        CHECK_CUDA(cudaMemcpy2DAsync(d_bgr,(size_t)cap_w*3,
                     cap_img.data,cap_img.step,
                     (size_t)cap_w*3,cap_h,cudaMemcpyHostToDevice,stream));
        if (need_crop) {
            const uint8_t* src=(const uint8_t*)d_bgr+(size_t)m_off_y*cap_w*3+m_off_x*3;
            CHECK_CUDA(cudaMemcpy2DAsync(d_model_bgr,(size_t)iw*3,
                         src,(size_t)cap_w*3,
                         (size_t)iw*3,ih,cudaMemcpyDeviceToDevice,stream));
        }
        launchPreprocess((const uint8_t*)d_model_bgr,d_in,iw,ih,
                         in_dt==nvinfer1::DataType::kFLOAT,stream);
        ctx->enqueueV3(stream);
        CHECK_CUDA(cudaMemcpyAsync(h_out,d_out,obytes,cudaMemcpyDeviceToHost,stream));
        CHECK_CUDA(cudaStreamSynchronize(stream));

        const float* od=nullptr;
        if (out_dt==nvinfer1::DataType::kFLOAT) od=(const float*)h_out;
        else { auto* ho=(const __half*)h_out;
               for (size_t i=0;i<ovol;++i) ofbuf[i]=__half2float(ho[i]); od=ofbuf.data(); }

        std::vector<Detection> raw;
        for (int i=0;i<ol.num;++i) {
            Detection d;
            if (end2end) {
                d.conf=outVal(od,ol,4,i);
                d.class_id=(int)std::round(outVal(od,ol,5,i));
                if (d.conf<conf_thr||d.class_id!=target_cls) continue;
                float x1=outVal(od,ol,0,i),y1=outVal(od,ol,1,i),
                      x2=outVal(od,ol,2,i),y2=outVal(od,ol,3,i);
                d.cx=(x1+x2)*.5f;d.cy=(y1+y2)*.5f;d.w=x2-x1;d.h=y2-y1;
            } else if (yolov5) {
                float obj=outVal(od,ol,4,i),mc=0;int ci=-1;
                for(int c=0;c<nclass;++c){float s=obj*outVal(od,ol,5+c,i);if(s>mc){mc=s;ci=c;}}
                if(mc<conf_thr||ci!=target_cls)continue;
                d.cx=outVal(od,ol,0,i);d.cy=outVal(od,ol,1,i);
                d.w=outVal(od,ol,2,i);d.h=outVal(od,ol,3,i);d.conf=mc;d.class_id=ci;
            } else {
                float mc=0;int ci=-1;
                for(int c=0;c<nclass;++c){float s=outVal(od,ol,4+c,i);if(s>mc){mc=s;ci=c;}}
                if(mc<conf_thr||ci!=target_cls)continue;
                d.cx=outVal(od,ol,0,i);d.cy=outVal(od,ol,1,i);
                d.w=outVal(od,ol,2,i);d.h=outVal(od,ol,3,i);d.conf=mc;d.class_id=ci;
            }
            raw.push_back(d);
        }
        auto filtered=nms(raw,nms_iou_thr);
        if (need_crop) for (auto& d:filtered) {
            d.cx+=m_off_x; d.cy+=m_off_y; }

        bool cal_collecting=g_calib_collect.load();
        if (cal_collecting&&!was_collecting) { hist.clear(); prev_gray_f.release();
            collect_frames=0; collect_resp=0; }
        was_collecting=cal_collecting;

        float best_dist=1e9f,best_dx=0,best_dy=0; bool found=false;
        for (auto& d:filtered) {
            float ty=d.cy+d.h*(0.5f-y_off_pct/100.0f);
            float dx=d.cx-cx0, dy=ty-cy0, dist=std::sqrt(dx*dx+dy*dy);
            if (dist<best_dist&&dist<FOV_RADIUS) { best_dist=dist;best_dx=dx;best_dy=dy;found=true; } }

        auto now=std::chrono::steady_clock::now();
        float dt=(float)elapsed_ms(now,t_prev); t_prev=now;
        dt=std::clamp(dt,1.0f,100.0f);

        if (found) {
            if (!filt_init) { fx=best_dx;fy=best_dy;fvx=0;fvy=0;filt_init=true; }
            else {
                float Lc=l_est*PRED_L_COMP;
                auto c0=g_counts.at(shift_ms(now,-(double)Lc-dt));
                auto c1=g_counts.at(shift_ms(now,-(double)Lc));
                float cax=(float)(c1.first-c0.first), cay=(float)(c1.second-c0.second);
                float px_pred=fx+fvx*dt-s_est*cax, py_pred=fy+fvy*dt-s_est*cay;
                float inx=best_dx-px_pred, iny=best_dy-py_pred;
                if (std::hypot(inx,iny)>TRACK_JUMP_GATE) { fx=best_dx;fy=best_dy;fvx=0;fvy=0; }
                else { float rr=dt/PRED_DT0;
                       float alpha=std::min(0.90f,PRED_ALPHA0*rr);
                       float beta=std::min(0.60f,PRED_BETA0*rr);
                       fx=px_pred+alpha*inx; fy=py_pred+alpha*iny;
                       fvx+=(beta/dt)*inx; fvy+=(beta/dt)*iny; }
            }
            { std::lock_guard<std::mutex> lk(g_target.mtx);
              g_target.px=fx;g_target.py=fy;g_target.vx=fvx;g_target.vy=fvy;
              g_target.s_est=s_est;g_target.l_est_ms=l_est;
              g_target.t_pub=now;g_target.valid=true; }
        } else {
            std::lock_guard<std::mutex> lk(g_target.mtx); g_target.valid=false;
        }

        if (cal_collecting) {
            cv::Mat gray,small,sf;
            cv::cvtColor(cap_img,gray,cv::COLOR_BGR2GRAY);
            cv::resize(gray,small,cv::Size(cap_w/2,cap_h/2),0,0,cv::INTER_AREA);
            small.convertTo(sf,CV_32F);
            if (!prev_gray_f.empty()) {
                float shx[9],shy[9];int nv=0;double br=0;
                for(int by=0;by<3;++by)for(int bx=0;bx<3;++bx){
                    cv::Rect r(bx*bs_w,by*bs_h,bs_w,bs_h); double resp=0;
                    cv::Point2d sh=cv::phaseCorrelate(prev_gray_f(r),sf(r),hann,&resp);
                    if(resp>0.01){shx[nv]=(float)sh.x;shy[nv]=(float)sh.y;++nv;}
                    if(resp>br)br=resp; }
                ++collect_frames; collect_resp+=br;
                if (nv>=4) { std::nth_element(shx,shx+nv/2,shx+nv);
                             std::nth_element(shy,shy+nv/2,shy+nv);
                             hist.push_back({now,dt,-2.0f*shx[nv/2],-2.0f*shy[nv/2]});
                             if((int)hist.size()>300)hist.pop_front(); }
            }
            prev_gray_f=sf;
        }

        if (g_calib_request.exchange(false)) {
            bool ok=false;
            for(int it=0;it<8;++it) ok|=run_calibration(hist,s_est,l_est);
            g_calib_done=ok?1:2;
            if (ok) { std::cout<<"[标定] s="<<s_est<<" px/count, L="<<l_est<<" ms\n";
                if (!persist_path.empty()) {
                    if (persist_calibration(persist_path,s_est,l_est))
                        std::cout<<"[标定] 已回写 "<<persist_path<<"\n";
                    else std::cerr<<"[标定] 回写失败\n"; }
            } else { std::cout<<"[标定] 失败: 样本 "<<hist.size()<<"/"<<collect_frames<<"\n"; }
        }

        if (collecting_enabled) {
            bool cd_ok = elapsed_ms(now,last_save) >= cooldown_ms;
            std::string mode;

            if (g_left_down.load()) {
                if (elapsed_ms(now,last_fire) >= fire_ms) { mode="fire"; last_fire=now; }
                if (now>=next_auto) next_auto=now+roll_auto();
            } else {
                if (found && cd_ok) mode="det";
                if (mode.empty() && now>=next_auto && cd_ok) mode="auto";
            }

            if (!mode.empty()) {
                enqueue_save(make_filepath(out_dir+"/"+mode), cap_img);
                last_save=now; next_auto=now+roll_auto();
                if (mode=="fire") ++n_fire;
                else if (mode=="det") ++n_det;
                else ++n_auto;
                std::cout<<"[SAVE] "<<mode<<"  (fire="<<n_fire<<" det="<<n_det
                         <<" auto="<<n_auto<<" drop="<<g_dropped.load()<<")\n";
            }
        }

        if (preview) {
            for (auto& d:filtered) {
                cv::Rect box((int)(d.cx-d.w*.5f+crop_x),(int)(d.cy-d.h*.5f+crop_y),(int)d.w,(int)d.h);
                box&=cv::Rect(0,0,cam_w,cam_h);
                cv::rectangle(frame,box,cv::Scalar(0,255,0),2);
                float ty=d.cy+d.h*(0.5f-y_off_pct/100.0f)+crop_y;
                cv::drawMarker(frame,cv::Point((int)(d.cx+crop_x),(int)ty),
                               cv::Scalar(0,0,255),cv::MARKER_CROSS,10,2);
            }
            if (collecting_enabled) {
                char st[128];
                snprintf(st,sizeof(st),"fire:%ld det:%ld auto:%ld drop:%ld%s",
                         n_fire,n_det,n_auto,g_dropped.load(),g_left_down.load()?" [FIRE]":"");
                cv::putText(frame,st,cv::Point(10,30),cv::FONT_HERSHEY_SIMPLEX,0.6,
                            g_left_down.load()?cv::Scalar(0,0,255):cv::Scalar(0,255,0),2);
            }
            cv::imshow("Aimbot",frame);
            if (cv::waitKey(1)==27) break;
        }
    }

    cap.release();
    if (preview) cv::destroyAllWindows();
    CHECK_CUDA(cudaStreamDestroy(stream));
    CHECK_CUDA(cudaFreeHost(h_out));
    CHECK_CUDA(cudaFree(d_in)); CHECK_CUDA(cudaFree(d_out)); CHECK_CUDA(cudaFree(d_bgr));
    if (d_model_bgr!=d_bgr) CHECK_CUDA(cudaFree(d_model_bgr));
    if (collecting_enabled)
        std::cout<<"采集统计: fire="<<n_fire<<" det="<<n_det<<" auto="<<n_auto
                 <<" dropped="<<g_dropped.load()<<"\n";
    std::cout<<"AI 线程已退出\n";
}

// ========================= 鼠标 I/O =========================
void extract_and_clear(MouseState& s, int16_t& x, int16_t& y,
                       int8_t& w, int8_t& hw, uint16_t& btns) {
    std::lock_guard<std::mutex> lk(s.mtx);
    x=(int16_t)std::clamp(s.rel_x,-32768,32767);
    y=(int16_t)std::clamp(s.rel_y,-32768,32767);
    w=(int8_t)std::clamp(s.rel_wheel,-128,127);
    hw=(int8_t)std::clamp(s.rel_hwheel,-128,127);
    btns=s.buttons;
    s.rel_x=s.rel_y=s.rel_wheel=s.rel_hwheel=0;
}
std::string find_mouse_device(const std::string& kw) {
    if (kw.front()=='/') return access(kw.c_str(),R_OK)==0?kw:"";
    std::string cmd="find "+std::string(DEV_SEARCH_PATH)
                   +" -name '*"+kw+"*-event-mouse' -print -quit 2>/dev/null";
    FILE* fp=popen(cmd.c_str(),"r"); if (!fp) return {};
    char buf[512]; std::string r;
    if (fgets(buf,sizeof(buf),fp)) { r=buf; if(!r.empty()&&r.back()=='\n') r.pop_back(); }
    pclose(fp); return r;
}
void reader_thread(const std::string& dev, MouseState& st) {
    int fd=open(dev.c_str(),O_RDONLY);
    if (fd<0) { std::cerr<<"无法打开鼠标\n"; global_running=false; return; }
    if (ioctl(fd,EVIOCGRAB,1)<0) std::cerr<<"警告: 无法独占\n";
    else std::cout<<"✅ 已独占: "<<dev<<"\n";
    struct pollfd pfd{}; pfd.fd=fd; pfd.events=POLLIN;
    struct input_event ev; int errs=0;
    while (global_running) {
        int pr=poll(&pfd,1,100);
        if (pr<0) { if(errno==EINTR)continue; global_running=false; break; }
        if (pr==0) continue;
        if (!(pfd.revents&POLLIN)) { std::cerr<<"鼠标断开\n"; global_running=false; break; }
        ssize_t n=read(fd,&ev,sizeof(ev));
        if (n==(ssize_t)sizeof(ev)) { errs=0;
            std::lock_guard<std::mutex> lk(st.mtx);
            if (ev.type==EV_REL) {
                if(ev.code==REL_X)st.rel_x+=ev.value;
                else if(ev.code==REL_Y)st.rel_y+=ev.value;
                else if(ev.code==REL_WHEEL)st.rel_wheel+=ev.value;
                else if(ev.code==REL_HWHEEL)st.rel_hwheel+=ev.value;
            } else if (ev.type==EV_KEY&&ev.code>=BTN_LEFT&&ev.code<=BTN_TASK) {
                int idx=ev.code-BTN_LEFT;
                if(ev.value)st.buttons|=(1<<idx); else st.buttons&=~(1<<idx); }
        } else if (n<0&&errno!=EINTR&&errno!=EAGAIN) {
            if(++errs>10){std::cerr<<"鼠标读取失败\n";global_running=false;break;} usleep(1000); }
    }
    close(fd);
}
void send_report(int fd, int16_t rx, int16_t ry, int8_t w, int8_t hw, uint16_t btns,
                 const std::function<void(std::array<uint8_t,HID_REPORT_LEN>&,int16_t,int16_t)>& overlay) {
    std::array<uint8_t,HID_REPORT_LEN> rpt{};
    rpt[0]=0x02; rpt[1]=btns&0xFF; rpt[2]=btns>>8;
    rpt[3]=rx&0xFF; rpt[4]=rx>>8; rpt[5]=ry&0xFF; rpt[6]=ry>>8;
    rpt[7]=w; rpt[8]=hw;
    if (overlay) overlay(rpt,rx,ry);
    static int errs=0;
    ssize_t n;
    do { n=::write(fd,rpt.data(),HID_REPORT_LEN); } while (n<0&&errno==EINTR);  // 信号中断: 重试
    if (n==(ssize_t)HID_REPORT_LEN) { errs=0; return; }
    if (n<0&&(errno==EAGAIN||errno==EWOULDBLOCK)) return;        // 瞬时节流: 丢这一帧, 不停机
    if (++errs>10) { std::cerr<<"写入虚拟鼠标失败\n"; global_running=false; }  // 持续硬错误才停
}

// ========================= main =========================
int main(int argc, char* argv[]) {
    std::cout<<"========================================\n"
             <<"  AI 视觉自瞄 (ffpi 控制律)\n"
             <<"========================================\n";

    std::string a_m,a_c,a_t,a_y,a_d,a_f,a_x,a_s,a_l,a_S,a_k,a_v;
    std::string a_o; int fire_ms=300; double auto_s=10;
    int cooldown_ms=500; int jpeg_q=95;

    for (int i=1;i<argc;++i) {
        std::string arg=argv[i];
        if      (arg=="-m"&&i+1<argc) a_m=argv[++i];
        else if (arg=="-c"&&i+1<argc) a_c=argv[++i];
        else if (arg=="-t"&&i+1<argc) a_t=argv[++i];
        else if (arg=="-y"&&i+1<argc) a_y=argv[++i];
        else if (arg=="-d"&&i+1<argc) a_d=argv[++i];
        else if (arg=="-f"&&i+1<argc) a_f=argv[++i];
        else if (arg=="-x"&&i+1<argc) a_x=argv[++i];
        else if (arg=="-s"&&i+1<argc) a_s=argv[++i];
        else if (arg=="-l"&&i+1<argc) a_l=argv[++i];
        else if (arg=="-S"&&i+1<argc) a_S=argv[++i];
        else if (arg=="-k"&&i+1<argc) a_k=argv[++i];
        else if (arg=="-v"&&i+1<argc) a_v=argv[++i];
        else if (arg=="-o"&&i+1<argc) a_o=argv[++i];
        else if (arg=="-F"&&i+1<argc) fire_ms=std::stoi(argv[++i]);
        else if (arg=="-A"&&i+1<argc) auto_s=std::stod(argv[++i]);
        else if (arg=="-C"&&i+1<argc) cooldown_ms=std::stoi(argv[++i]);
        else if (arg=="-q"&&i+1<argc) jpeg_q=std::stoi(argv[++i]);
        else if (arg=="-h"||arg=="--help") {
            std::cout<<"用法: "<<argv[0]<<" [自瞄选项] [采集选项]\n"
                "\n自瞄选项:\n"
                "  -m <路径>  模型       -c <ID>   类别      -t <阈值> 置信度\n"
                "  -y <偏移>  部位       -d <采集卡> 名字或 /dev/videoN\n"
                "  -f <帧率>  120/60     -x <速度> 最大px/s\n"
                "  -s <s>     初始灵敏度 -l <L>    初始延迟\n"
                "  -S <脚本>  回写路径   -k <键>   fire/ads/both  -v <y/n> 预览\n"
                "\n采集选项 (不传 -o 则纯自瞄不截图):\n"
                "  -o <目录>  输出目录 (自动建 fire/ det/ auto/ 子目录)\n"
                "  -F <ms>    开火截图间隔 (默认 300)\n"
                "  -A <秒>    定时截图间隔 (默认 10, 随机 0.5x~1.5x)\n"
                "  -C <ms>    检测/定时截图冷却 (默认 500, 开火不受限)\n"
                "  -q <1-100> JPEG 质量 (默认 95)\n";
            return 0;
        }
    }

    std::string model_path;
    if (!a_m.empty()) { model_path=a_m;
        if (!std::ifstream(model_path).good()) { std::cerr<<"❌ 模型不存在\n"; return 1; }
    } else { while(true) { std::cout<<"模型路径: "; std::getline(std::cin,model_path);
             if (std::ifstream(model_path).good()) break; std::cerr<<"文件不存在\n"; } }

    int   cls     =std::stoi(!a_c.empty()?a_c:get_input_with_default("类别ID","0"));
    float conf    =std::stof(!a_t.empty()?a_t:get_input_with_default("置信度","0.4"));
    float y_off   =std::stof(!a_y.empty()?a_y:get_input_with_default("Y偏移","65"));
    int   cam_fps =std::stoi(!a_f.empty()?a_f:get_input_with_default("帧率","120"));
    cam_fps=std::clamp(cam_fps,1,240);
    float max_spd =std::stof(!a_x.empty()?a_x:get_input_with_default("最大速度","1500"));
    max_spd=std::clamp(max_spd,100.0f,20000.0f);
    const float max_v=max_spd/1000.0f;
    float init_s=std::clamp(std::stof(a_s.empty()?"1.0":a_s),S_MIN,S_MAX);
    float init_l=std::clamp(std::stof(a_l.empty()?"60":a_l),L_MIN,L_MAX);
    const std::string persist_path=a_S;
    std::string aim_key=!a_k.empty()?a_k:get_input_with_default("触发键","fire");
    int aim_mode=0;
    if(aim_key=="ads")aim_mode=1; else if(aim_key=="both")aim_mode=2;
    std::string pv=!a_v.empty()?a_v:get_input_with_default("预览(y/n)","n");
    bool preview=(pv=="y"||pv=="Y");
    if(!preview) unsetenv("DISPLAY");
    jpeg_q=std::clamp(jpeg_q,1,100);

    const bool do_collect=!a_o.empty();
    if (do_collect) { ensure_dir(a_o); ensure_dir(a_o+"/fire");
                      ensure_dir(a_o+"/det"); ensure_dir(a_o+"/auto"); }

    std::string cam_dev=resolve_cam_device(a_d.empty()?"/dev/video0":a_d);
    if (cam_dev.empty()) return 1;
    if (access(cam_dev.c_str(),F_OK)!=0) {
        std::cerr<<"❌ 采集卡设备不存在: "<<cam_dev<<"\n"; return 1; }
    std::cout<<"✅ 采集卡: "<<cam_dev<<"\n";

    std::string real_dev=find_mouse_device(DEFAULT_KEYWORD);
    if (real_dev.empty()) { std::cerr<<"❌ 未找到鼠标\n"; return 1; }
    std::cout<<"✅ 鼠标: "<<real_dev<<"\n";
    int virt_fd=open(DEFAULT_VIRT_DEV,O_WRONLY);
    if (virt_fd<0) { std::cerr<<"❌ 无法打开 "<<DEFAULT_VIRT_DEV<<"\n"; return 1; }

    signal(SIGINT,signal_handler); signal(SIGTERM,signal_handler);
    { std::lock_guard<std::mutex> lk(g_target.mtx);
      g_target.s_est=init_s; g_target.l_est_ms=init_l; }

    std::cout<<"初始: s="<<init_s<<" L="<<init_l<<"\n";
    if (do_collect)
        std::cout<<"采集: "<<a_o<<"  开火="<<fire_ms<<"ms  定时="<<auto_s
                 <<"s  冷却="<<cooldown_ms<<"ms\n";
    else
        std::cout<<"采集: 关闭 (未传 -o)\n";

    MouseState state;
    std::thread reader(reader_thread,real_dev,std::ref(state));
    std::thread writer; if (do_collect) writer=std::thread(writer_thread,jpeg_q);
    std::thread ai(ai_thread,model_path,conf,cls,y_off,cam_dev,cam_fps,preview,
                   init_s,init_l,persist_path,
                   a_o,fire_ms,auto_s,cooldown_ms,jpeg_q);

    int tfd=timerfd_create(CLOCK_MONOTONIC,0);
    struct itimerspec its{}; its.it_value.tv_nsec=1;
    its.it_interval.tv_nsec=1'000'000'000/DEFAULT_FREQ;
    timerfd_settime(tfd,0,&its,nullptr);
    int ep=epoll_create1(0);
    struct epoll_event evt{}; evt.events=EPOLLIN; evt.data.fd=tfd;
    epoll_ctl(ep,EPOLL_CTL_ADD,tfd,&evt);

    auto last_press=std::chrono::steady_clock::now()-std::chrono::hours(1);
    auto overlay=[=,&last_press](std::array<uint8_t,HID_REPORT_LEN>& rpt,
                                 int16_t real_x,int16_t real_y) mutable {
        static float rem_x=0,rem_y=0;
        static float int_x=0,int_y=0;
        static int cal=0,hold=0;
        static const CalibSeg* seq=nullptr;
        static int slen=0,si=0,st=0,wt=0;
        static std::vector<CalibSeg> excite;

        auto now=std::chrono::steady_clock::now();
        uint16_t btns=rpt[1]|(rpt[2]<<8);
        bool left=btns&LEFT_KEY, right=btns&RIGHT_KEY, side=btns&SIDE_KEY;
        g_left_down.store(left);

        int32_t fx=real_x, fy=real_y;

        if (cal==3) {
            fx=fy=0;
            int done=g_calib_done.load();
            if (done!=0||++wt>CALIB_WAIT_TIMEOUT) {
                seq=done==1?CAL_END_OK_SEQ:CAL_END_FAIL_SEQ;
                slen=done==1?(int)(sizeof(CAL_END_OK_SEQ)/sizeof(CalibSeg))
                           :(int)(sizeof(CAL_END_FAIL_SEQ)/sizeof(CalibSeg));
                cal=done==1?4:5; si=st=0; }
        } else if (cal!=0) {
            if (si<slen) { auto& sg=seq[si]; fx=sg.dx;fy=sg.dy;
                if(++st>=sg.ticks){st=0;++si;} }
            if (si>=slen) {
                if (cal==1) { excite.clear();
                    for(int i=0;i<5;++i){excite.push_back({4,0,125});excite.push_back({0,4,125});
                        excite.push_back({-4,0,125});excite.push_back({0,-4,125});}
                    seq=excite.data();slen=(int)excite.size();si=st=0;
                    g_calib_collect=true;cal=2;
                } else if (cal==2) { seq=CAL_SETTLE_SEQ;slen=1;si=st=0;cal=6;fx=fy=0;
                } else if (cal==6) { g_calib_collect=false;g_calib_done=0;
                    g_calib_request=true;wt=0;cal=3;fx=fy=0;
                } else { cal=0;fx=real_x;fy=real_y; } }
            rem_x=rem_y=0;
        } else {
            bool trig=(aim_mode==2)?(left||right):(aim_mode==1)?right:left;
            if(trig&&!side)last_press=now;
            bool aiming=std::chrono::duration_cast<std::chrono::milliseconds>(
                            now-last_press).count()<=KEEP_ALIVE_MS;
            if (aiming) {
                float px,py,vx,vy,se,le;bool valid;
                std::chrono::steady_clock::time_point tp;
                { std::lock_guard<std::mutex> lk(g_target.mtx);
                  px=g_target.px;py=g_target.py;vx=g_target.vx;vy=g_target.vy;
                  se=g_target.s_est;le=g_target.l_est_ms;
                  valid=g_target.valid;tp=g_target.t_pub; }
                double age=elapsed_ms(now,tp);
                if (valid&&age<TARGET_STALE_MS) {
                    float Lc=le*PRED_L_COMP;
                    auto cp=g_counts.at(shift_ms(tp,-(double)Lc));
                    auto cn=g_counts.cum();
                    float ifx=se*(float)(cn.first-cp.first);
                    float ify=se*(float)(cn.second-cp.second);
                    float ex=px+vx*(float)(age+Lc)-ifx;
                    float ey=py+vy*(float)(age+Lc)-ify;
                    float r=std::hypot(ex,ey);
                    // 收敛带宽由标定延迟导出: wn=(90°−PM)π/180/L (PM=60, 免手调, 随 L 自动缩放)
                    float L=std::max(1.0f,le);
                    float wn=(90.0f-FF_PM_DEG)*3.14159265358979f/180.0f/L;
                    float kp=2.0f*FF_ZETA*wn;
                    float ki=wn*wn;
                    float gate=FF_I_GATE/(FF_I_GATE+r);
                    float i_lim=FF_I_FRAC*max_v/std::max(ki,1e-9f);
                    float vx_u=kp*ex+ki*int_x;
                    float vy_u=kp*ey+ki*int_y;
                    if (ex*ex+ey*ey>FOV_RADIUS*FOV_RADIUS) { int_x=int_y=0; }
                    else {
                        bool wx=(vx_u>max_v&&ex>0)||(vx_u<-max_v&&ex<0);
                        bool wy=(vy_u>max_v&&ey>0)||(vy_u<-max_v&&ey<0);
                        if(!wx)int_x=std::clamp(int_x+ex*TICK_MS*gate,-i_lim,i_lim);
                        if(!wy)int_y=std::clamp(int_y+ey*TICK_MS*gate,-i_lim,i_lim);
                    }
                    // type-2 速度前馈: ff_gain=1 是匀速目标零拖尾的精确开环指令 (门控到收敛区)
                    vx_u+=FF_GAIN_VAL*gate*vx;
                    vy_u+=FF_GAIN_VAL*gate*vy;
                    float vcx=std::clamp(vx_u,-max_v,max_v);
                    float vcy=std::clamp(vy_u,-max_v,max_v);
                    float s=std::clamp(se,S_MIN,S_MAX);
                    rem_x+=vcx*TICK_MS/s; rem_y+=vcy*TICK_MS/s;
                    int sx=std::clamp((int)std::trunc(rem_x),-120,120);
                    int sy=std::clamp((int)std::trunc(rem_y),-120,120);
                    rem_x-=sx;rem_y-=sy; fx+=sx;fy+=sy;
                } else { rem_x=rem_y=0; int_x=int_y=0; }
            } else { rem_x=rem_y=0; int_x=int_y=0; }

            if ((btns&BOTH_SIDE_KEYS)==BOTH_SIDE_KEYS) {
                if(++hold>=CALIB_TRIGGER_TICKS){hold=0;cal=1;
                    seq=CAL_START_SEQ;slen=(int)(sizeof(CAL_START_SEQ)/sizeof(CalibSeg));
                    si=st=0;std::cout<<"[标定] 触发\n";}
            } else hold=0;
        }

        fx=std::clamp(fx,-32768,32767); fy=std::clamp(fy,-32768,32767);
        rpt[3]=fx&0xFF;rpt[4]=fx>>8; rpt[5]=fy&0xFF;rpt[6]=fy>>8;
        g_counts.add(now,(int)fx,(int)fy);
    };

    std::cout<<"✅ 500Hz 运行中, Ctrl+C 停止\n";
    while (global_running) {
        struct epoll_event evs[1];
        int nf=epoll_wait(ep,evs,1,500);
        if(nf<0&&errno==EINTR)continue; if(nf<=0)continue;
        uint64_t exp; read(tfd,&exp,sizeof(exp));
        int16_t x,y;int8_t w,hw;uint16_t b;
        extract_and_clear(state,x,y,w,hw,b);
        send_report(virt_fd,x,y,w,hw,b,overlay);
    }

    global_running=false;
    g_save_cv.notify_all();
    reader.join(); ai.join();
    if (do_collect) writer.join();
    close(virt_fd);close(tfd);close(ep);
    std::cout<<"已停止\n";
    return 0;
}
