// ============================================================================
//  state.h — 进程级共享状态: 系统常量与拍率换算, 热参/标定原子, 时间辅助,
//    目标/counts/鼠标共享结构, 异步写盘队列, 信号。跨线程数据通道的唯一定义
//    点: g_target (mutex, 采集线程→控制拍) 与 g_counts (mutex, 双向: 采集读
//    自身指令补偿, 控制写实际发出的轨迹)。
// ============================================================================

#pragma once

#include <algorithm>
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
// 控制拍频率 Hz: 透传、控制律与标定状态机共用一个节拍。1kHz 是"拍率不再是精度
//   瓶颈"的选择 — 控制律按实测 dt 归一 (不变量 1), 提高拍率只把同一连续律采样
//   得更细, 不改变手感; 报告率 = min(拍率, 主机服务率), 端点 bInterval=1 (高速
//   125µs 微帧 → 服务能力 8kHz) 高于拍率, 故实际报告率 = 拍率。
constexpr int    DEFAULT_FREQ    = 1000;
constexpr const char* DEFAULT_KEYWORD  = "";         // 空 = 任一 *-event-mouse 中字典序首个
constexpr const char* DEV_SEARCH_PATH  = "/dev/input/by-id/";

const float FOV_RADIUS   = 150.0f;                   // FOV 半径默认值 (px): 目标筛选圈兼积分器边界; 经 -r 或热参 fov 覆盖 (运行时 g_fov_radius)
const int   HOT_CTL_PORT = 47700;                    // 热参数通道端口 (UDP, 仅绑 127.0.0.1)
const int   CAP_SIZE     = 640;                      // 最小采集边长 (px): 模型输入更小时也按此尺寸采集, 再居中裁到模型输入
const float TICK_MS      = 1000.0f / DEFAULT_FREQ;   // 控制拍周期 (ms)

// 时长一律以墙钟毫秒为准, 拍数是它的换算结果: 拍数写死会在换拍率时改变物理
//   时长 (标定的激励段/停顿、触发与回执窗口都是设计量), 故拍数只由 ms 导出。
constexpr int ms_to_ticks(int ms) { return ms * DEFAULT_FREQ / 1000; }

// ========================= 拉枪速度倍率 (spd) =========================
// spd 是**拉枪速度倍率**, 与有效灵敏度成反比: 有效灵敏度 = 基线 / k(spd),
//   k(spd) = spd/100。spd 大 → 假定的游戏灵敏度低 → 同样的期望屏幕速度发出更多
//   counts / 更大偏转 → 屏幕跟得更快 = 拉枪更快。整数步进 (105/109), **100 = 基线**
//   (出厂手感), 与 spd 成反比的落点使"调大 = 更快"在四个值上一致。
// 基线是有出处的, 不是为凑 100 造出来的: hid 的基线就是本库的灵敏度占位 1.0
//   px/count, pad 的基线就是满偏屏幕速度 3000 px/s (COD 实测 30–70% 档
//   193/556/1159/1651/1804 px/s → 满偏外推 ≈2600, 取 3000; 与速度帽推导
//   2000 px/s = 960·v/d 同量级)。
// 夹取带 [1, 10000] 是防误输入 (0/负数/离谱放大值); 有意义的带是 5..2000 ——
//   有效灵敏度 s_hid_eff = 100/spd 落在 0.05–20 px/count: 基线 (1 px/count 与
//   3000 px/s) 两侧各约 1.3 个数量级的修正范围。这是选择规则不是实测带: 游戏侧的
//   灵敏度是未知量, 倍率只需能把基线拉进游戏的工作区间, 再往外不产生新的分辨率,
//   只会在第一次拉枪时把准星甩飞。逐轴取值: 同一灵敏度下垂直与水平的屏速比是
//   游戏属性 (俯仰灵敏度常更低), 一个总倍率会把两轴绑死; ADS 键按住期间整套换成
//   adsspd 那一对。速度帽 (-x) 不随 spd 变 — 它约束"准星能否追上目标的屏幕速度",
//   是游戏量, 与转换刻度无关。
const int   SPD_BASE = 100;                          // 基线显示值 (整数步进 1 = 快 1%)
const int   SPD_MIN  = 1, SPD_MAX = 10000;           // 防误输入夹取带 (见上)
const float S_HID_BASE    = 1.0f;                    // hid 基线灵敏度 px/count (s_eff = 基线/k)
const float GAIN_PAD_BASE = 3000.0f;                 // pad 满偏屏幕速度基线 px/s (A_eff = 基线/k;
                                                     //   手柄输出是其第一个消费者)
inline float spd_k(int spd) { return (float)spd / (float)SPD_BASE; }
inline float s_hid_eff(int spd) { return S_HID_BASE / spd_k(spd); }
inline float gain_pad_eff(int spd) { return GAIN_PAD_BASE / spd_k(spd); }
// 夹取: 唯一定义点 — CLI (--spd/--ads-spd) 与热参 (spdx/...) 都经它
inline int spd_clamp(long v) {
    return (int)std::clamp<long>(v, (long)SPD_MIN, (long)SPD_MAX);
}

// ========================= 全局状态 =========================
extern std::atomic<bool> global_running;
void signal_handler(int);

extern std::atomic<bool> g_calib_collect;
extern std::atomic<bool> g_calib_request;
extern std::atomic<int>  g_calib_done;               // 0=计算中 1=成功 2=失败
extern std::atomic<bool> g_padcalib_request;         // pad 标定请求 (热参 padcalib=1; 一次消费即清)

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
// ---- 拉枪速度倍率 (CLI --spd/--ads-spd, 热参 spdx/spdy/adsspdx/adsspdy) ----
// 四个值唯一, 逐轴: 腰射一对, ADS 键按住期间一对 (整套换, 不逐位混搭)。刻度与基线
//   的定义见上方 "拉枪速度倍率 (spd)"。
extern std::atomic<int>   g_spd_x, g_spd_y;          // 腰射: X / Y (显示值, 100 = 基线)
extern std::atomic<int>   g_ads_spd_x, g_ads_spd_y;  // ADS 键按住时: X / Y (同刻度)
// ADS 键状态 (右键位; 控制拍每拍写): 估计器/在飞补偿按帧时刻的该状态取倍率 —
//   飞行窗只有 L 毫秒, 窗内切态的误差是二阶量 (接受的取舍)
extern std::atomic<bool>  g_ads_down;
inline bool ads_down() { return g_ads_down.load(); }
// 逐轴取当前显示值 (axis: 0 = X / 1 = Y)
inline int spd_axis(bool ads, int axis) {
    return ads ? (axis ? g_ads_spd_y.load() : g_ads_spd_x.load())
               : (axis ? g_spd_y.load() : g_spd_x.load());
}
// 逐轴 hid 有效灵敏度 (px/count) = 基线 / k(spd): 注入换算、在飞补偿、估计器自身
//   运动补偿三处消费同一份值 — 分裂会让命令放大多少、补偿就错多少
inline float s_hid_now(bool ads, int axis) { return s_hid_eff(spd_axis(ads, axis)); }

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
    float l_est_ms = 60.0f;                      // 标定量: 环路延迟 (ms), 唯一被回写的量
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
    // 清空: at() 按时间二分, 混入不同时间基的记录会让查找失效 — 单测在换时间基
    //   时先清空 (真机上账本由单一线程按真实钟写入, 天然单调, 无需清空)
    void clear() {
        std::lock_guard<std::mutex> lk(mtx);
        buf.clear(); cum_x = 0; cum_y = 0;
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
