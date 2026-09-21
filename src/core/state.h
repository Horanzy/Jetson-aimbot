// ============================================================================
//  state.h — 进程级共享状态: 系统常量与拍率换算, 热参/标定原子, 时间辅助,
//    目标/counts/鼠标共享结构, 异步写盘队列, 信号。跨线程数据通道的唯一定义
//    点: g_target (mutex, 采集线程→控制拍) 与 g_counts (mutex, 双向: 采集读
//    自身指令补偿, 控制写实际发出的轨迹)。
// ============================================================================

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <queue>
#include <string>
#include <utility>

#include <opencv2/opencv.hpp>

#include "core/control.h"

// ========================= 系统常量 =========================
constexpr size_t HID_REPORT_LEN  = 9;
constexpr int    DEFAULT_FREQ    = 500;              // 控制拍频率 Hz (透传与标定状态机同一节拍)
constexpr const char* DEFAULT_KEYWORD  = "";         // 空 = 匹配任意 *-event-mouse 设备
constexpr const char* DEFAULT_VIRT_DEV = "/dev/hidg0";
constexpr const char* DEV_SEARCH_PATH  = "/dev/input/by-id/";

const float FOV_RADIUS   = 150.0f;                   // FOV 半径默认值 (px): 目标筛选圈兼积分器边界; 经 -r 或热参 fov 覆盖 (运行时 g_fov_radius)
const int   HOT_CTL_PORT = 47700;                    // 热参数通道端口 (UDP, 仅绑 127.0.0.1)
const int   CAP_SIZE     = 640;                      // 最小采集边长 (px): 模型输入更小时也按此尺寸采集, 再居中裁到模型输入
const float TICK_MS      = 1000.0f / DEFAULT_FREQ;   // 控制拍周期 (ms)

// 时长一律以墙钟毫秒为准, 拍数是它的换算结果: 拍数写死会在换拍率时改变物理
//   时长 (标定的激励段/停顿、触发与回执窗口都是设计量), 故拍数只由 ms 导出。
constexpr int ms_to_ticks(int ms) { return ms * DEFAULT_FREQ / 1000; }

// ========================= 全局状态 =========================
extern std::atomic<bool> global_running;
void signal_handler(int);

extern std::atomic<bool> g_calib_collect;
extern std::atomic<bool> g_calib_request;
extern std::atomic<int>  g_calib_done;               // 0=计算中 1=成功 2=失败

extern std::atomic<bool> g_left_down;

// ---- 热参数 (webui 经 UDP 127.0.0.1 下发; 白名单外忽略, 数值在固件侧强制钳制) ----
extern std::atomic<float> g_conf_thr;                // 置信度阈值 (-t)
extern std::atomic<float> g_y_off_pct;               // 瞄准高度偏移 % (-y)
extern std::atomic<float> g_max_v;                   // 速度上限 px/ms (= -x / 1000)
extern std::atomic<int>   g_aim_mode;                // 触发键模式 (-k): 0=fire 1=ads 2=both
extern std::atomic<float> g_fov_radius;              // FOV 半径 px (-r / 热参 fov)
extern std::atomic<bool>  g_aim_enabled;             // 鼠标接管 (-a / 热参 aim): false=纯透传不注入
extern std::atomic<bool>  g_cap_fire;                // 采集源开关 (-e / 热参): 开火截图
extern std::atomic<bool>  g_cap_det;                 //   检测截图
extern std::atomic<bool>  g_cap_auto;                //   定时截图

// ---- 时间辅助 ----
inline std::chrono::steady_clock::time_point shift_ms(
    std::chrono::steady_clock::time_point t, double ms) {
    return t + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
               std::chrono::duration<double, std::milli>(ms));
}
inline double elapsed_ms(
    std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b) {
    return std::chrono::duration<double, std::milli>(a - b).count();
}

// ---- 目标状态 ----
struct TargetState {
    float px = 0, py = 0, vx = 0, vy = 0;
    float ax_e = 0, ay_e = 0;                    // â 加速度估计 (px/ms², 0=未通过显著性/重建/门控)
    float last_dt = 1000.0f / 120.0f;            // 最近一帧的滤波增益 (供 â 反演与 ε 修正)
    float last_alpha = PRED_ALPHA0, last_beta = PRED_BETA0;
    float cs = 0;                                // CUSUM 告警电平 (σ 倍数归一, 信任度来源)
    bool  valid = false;
    std::chrono::steady_clock::time_point t_pub;
    float s_est = 1.0f, l_est_ms = 60.0f;
    std::mutex mtx;
};
extern TargetState g_target;

// ---- counts 历史 ----
// 深度 = 3s 墙钟的拍数: run_calibration 按 g_counts.at(t−lag) 回溯, 最深的查询
//   是标定直方图最早一帧再退一个 lag(max(L)+dl+dt ≈ 0.3s), 而直方图最多 300
//   帧 (120fps ≈ 2.5s) — 3s 整体覆盖它。深度以拍计会随拍率缩水, 故按墙钟表达。
const size_t COUNTS_HIST_TICKS = (size_t)3 * DEFAULT_FREQ;
class CountsHistory {
public:
    struct Sample { std::chrono::steady_clock::time_point t; long long cx, cy; };
    void add(std::chrono::steady_clock::time_point t, int dx, int dy) {
        std::lock_guard<std::mutex> lk(mtx);
        cum_x += dx; cum_y += dy;
        buf.push_back({t, cum_x, cum_y});
        while (buf.size() > COUNTS_HIST_TICKS) buf.pop_front();
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
extern CountsHistory g_counts;

struct MouseState {
    int32_t rel_x=0, rel_y=0, rel_wheel=0, rel_hwheel=0;
    uint16_t buttons = 0;
    std::mutex mtx;
};

// ---- 异步写盘队列 ----
const size_t SAVE_QUEUE_MAX = 8;
struct SaveTask { cv::Mat img; std::string path; };
extern std::queue<SaveTask> g_save_q;
extern std::mutex g_save_mtx;
extern std::condition_variable g_save_cv;
extern std::atomic<long> g_dropped;

void enqueue_save(const std::string& path, const cv::Mat& frame);
void writer_thread(int quality);
std::string make_filepath(const std::string& dir);
void ensure_dir(const std::string& d);
