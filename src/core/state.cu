// ============================================================================
//  state.cu — state.h 中共享全局的定义与异步写盘队列的实现。
// ============================================================================

#include "core/state.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>

std::atomic<bool> global_running{true};
void signal_handler(int) { global_running = false; }

std::atomic<bool> g_calib_collect{false};
std::atomic<bool> g_calib_request{false};
std::atomic<int>  g_calib_done{0};                   // 0=计算中 1=成功 2=失败
std::atomic<bool> g_padcalib_request{false};         // pad 标定请求 (由 pad 拍消费)

std::atomic<bool> g_left_down{false};

std::atomic<float> g_conf_thr{0.4f};                 // 置信度阈值 (-t)
std::atomic<float> g_y_off_pct{65.0f};               // 瞄准高度偏移 % (-y)
std::atomic<float> g_max_v{1.5f};                    // 速度上限 px/ms (= -x / 1000)
std::atomic<int>   g_aim_mode{0};                    // 触发键模式 (-k): 0=fire 1=ads 2=both
std::atomic<float> g_fov_radius{FOV_RADIUS};         // FOV 半径 px (-r / 热参 fov)
std::atomic<bool>  g_aim_enabled{true};              // 鼠标接管 (-a / 热参 aim): false=纯透传不注入
std::atomic<bool>  g_cap_fire{true};                 // 采集源开关 (-e / 热参): 开火截图
std::atomic<bool>  g_cap_det{true};                  //   检测截图
std::atomic<bool>  g_cap_auto{true};                 //   定时截图

TargetState g_target;

CountsHistory g_counts;

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
