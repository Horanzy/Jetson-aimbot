// ============================================================================
//  calib.h — 标定的采样底座 (与 AI 检测无关): 采样几何 (640 居中裁切 → 320 半分辨率
//    相关域 → 3×3 块), 单轴一维投影相位相关 (calib_pc1d), 单帧块统计 (静止簇剔除 +
//    中位), 标定值的脚本原子回写 (persist_calibration), 采集卡设备名解析
//    (resolve_cam_device)。测量方法 (停顿 + 三读数) 与激励计划在 io/calib_run.h。
//
//  采样几何 (与模型输入尺寸解耦; 推理链路的裁剪逐位不变):
//    标定固定取采集帧的 640×640 居中裁剪, 灰度后 INTER_AREA 降到 320×320 的
//    半分辨率相关域, 3×3 块 (域内块宽 106px)。相关域 1px = 2 屏幕像素
//    (CALIB_SAMPLE_SCALE, 由裁剪/采样两边导出), 位移乘尺度还原成屏幕像素。
//    跟模型走是不行的: 标定测的是屏幕物理位移, 换一个网络输入尺寸就换一个标定场
//    (块宽变 → 量程与噪声底都变), 而标定值必须只依赖机器与游戏; 640 = 采集管道
//    最小边长 (CAP_SIZE), 即与模型解耦后仍能共用的最小裁剪。
//    半分辨率是采样率决策: 相关的代价随域面积走, 而可靠量程与域分辨率无关 (它由
//    屏幕域的块宽定), 所以降采样只买采样率。真机实测 (Orin, cv 4.8) 见下表。
//
//  沿轴的一维投影而不是二维块相关 —— 两种都实现并实测过 (同一批合成注入 + 真实
//  采集卡纹理; "投影" = 块沿轴求和成一维再相关, 二维 = cv::phaseCorrelate):
//
//    | 量测                                   | 二维 3×3 域 320 | 一维投影 域 320 |
//    |----------------------------------------|-----------------|-----------------|
//    | 每帧相关耗时 (Orin CPU, 9 块)           | 5.19 ms (62%)   | 0.90 ms (11%)   |
//    | 实时采集下每帧耗时 (含 1080p→BGR)        | 10.56 ms        | 2.34 ms         |
//    | 亚像素复原误差 (真实纹理, 屏幕 px)       | −0.04 … −0.15   | ±0.01           |
//    | 相关峰 @ 域内 60px 位移                  | 0.31            | 0.54            |
//    | 静止 HUD 占 6/9 块时的剔除前后 (屏幕 px) | 0.00 → 89.87    | 0.00 → 89.90    |
//    | 旁轴 60px/帧 时本轴读数偏移 (屏幕 px)    | −0.04           | −0.05           |
//    | 域内精确范围 (扫描到 52px 仍精确)        | 52px = 屏幕104  | 52px = 屏幕104  |
//
//  一维投影在三处胜出: 贵 5.8 倍的相关 (62% → 11% 的 120fps 预算, 60fps 下 5.5%),
// 亚像素精度高一个量级 (直接进尾迹 → 延迟的换算), 峰随位移衰减更慢 (量程更宽)。
// 旁轴污染 (投影把另一个轴的位移当噪声收进来) 实测 −0.05px: 一维投影把块内 106 行
// 平均掉, 另一轴的运动只改变这个平均的随机起伏, 不产生系统性位移。故采样侧是
// **一维投影**: 每块每轴一条 1-D 相位相关 (2 轴 × 9 块 = 18 次长度 106 的 DFT)。
//  上半分辨率域仍留着 (要求与实测一致): 全分辨率 640 域的一维投影是 2.47 ms/帧
// (30%) — 也在预算内, 但屏幕域噪声只有 2 倍之差 (σ 0.05 vs 0.1 px/帧), 而读数自身
// 的量化是 ±帧长/2 (4–8ms), 这点噪声差不出现在任何判定里。
//
//  块统计与静止内容剔除 (采样端唯一的统计步骤):
//    块相位相关峰低于纳入门的块先剔除 (该块无纹理)。游戏 HUD 在屏幕上静止: 这些块
// 报"位移≈0 且峰高" → 若它们占纳入块多数, 块间**中位数**被静止簇劫持 → 测出的位移
// 趋近 0 (实测: HUD 占 6/9 块、真位移 90px 时朴素中位读 0.00 — 不是低读而是全丢),
// 且真实速度越大丢得越狠。故先按 |位移| 把块分两簇, 只对运动簇取中位:
// 门限 = CALIB_STATIC_FRAC×最大块位移。0.5 的出处: 同一运动簇内各块的差异只来自
// 视差 (块心相对裁剪中心的差分旋转/平移比, 交战距离上 ≤1.15×), 与 0.5 之间还有
// ≈1.7 倍余量; 而静止簇的位移是检测噪声 (零点几 px), 与运动簇 (每帧几 px 到几十 px)
// 差一个数量级以上 — 门限落在两簇之间且无量纲。兜底: 运动簇不足 3 块 (单块异常把
// 门限顶高) 时退回全体中位, 多数块仍在全体中位里。
// ============================================================================

#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <string>

#include <opencv2/opencv.hpp>          // 采样器 (投影 + 一维 DFT) 就地实现

#include "core/state.h"                // DEFAULT_FREQ / ms_to_ticks / CAP_SIZE

// ========================= 时长与标定带 (以拍计的时长一律由墙钟导出) =========================
const int CALIB_TRIGGER_TICKS = ms_to_ticks(5000);   // 触发长按 (hid 双侧键 / pad L3+R3)
const int CALIB_WAIT_TIMEOUT  = ms_to_ticks(2000);   // 等采样侧拟合回执超时 2s

// 环路延迟的物理带 (ms): 上界 = 最慢假定采集率 (60fps) 下 7 帧 — 游戏侧输入→显示
//   4 帧 + 采集/推理/拍 3 帧 = 116.7 → 取整 120。停顿长度与静止参考窗都由这个上界
//   导出 (见 io/calib_run.h), 所以它不是一个松钳制: 超出它的实测值意味着测量被
//   停顿截断, 判失败而不是硬钳制。
const float L_MIN = 0.0f, L_MAX = 120.0f;

// ---- 采样几何 ----
constexpr int CALIB_CROP_PX   = CAP_SIZE;                       // 标定裁剪边长 (1:1 取自帧中心)
constexpr int CALIB_SAMPLE_PX = CALIB_CROP_PX / 2;              // 相关域边长 320 (半分辨率)
constexpr int CALIB_GRID_N    = 3;                              // 3×3 块
constexpr int CALIB_BLOCK_PX  = CALIB_SAMPLE_PX / CALIB_GRID_N; // 相关域块宽 ≈106px
constexpr int CALIB_BLOCKS_N  = CALIB_GRID_N * CALIB_GRID_N;
// 相关域 → 屏幕像素: 640 裁剪压到 320 相关 → 2.0 (由裁剪/采样两边导出, 改分辨率
//   不会漏改系数)
constexpr float CALIB_SAMPLE_SCALE = (float)CALIB_CROP_PX / (float)CALIB_SAMPLE_PX;

// 标定裁剪矩形 (居中正方形, 1:1): 纯函数 — 单测直接断言"居中且无缩放"
struct CalCropRect { int x, y, w, h; };
inline constexpr CalCropRect calib_crop_rect(int frame_w, int frame_h) {
    return {(frame_w - CALIB_CROP_PX) / 2, (frame_h - CALIB_CROP_PX) / 2,
            CALIB_CROP_PX, CALIB_CROP_PX};
}

// 可靠每帧位移上界 (屏幕 px) = 域内块宽的一半 × 尺度 = 106px。出处: 循环相关
//   (DFT 相位相关) 的峰只在 |位移| < 长度一半时唯一, 越过即回卷 (读数跳到 s−B,
//   符号翻转)。实测: 域内注到 52px (屏幕 104px) 仍精确复原, 域边界 53px 与之吻合。
//   120fps 下 106px/帧 ≈ 12700px/s; 设计激励速度 1.5px/ms 时为 12.5px/帧, 8.5 倍余量
//   (等效可容忍连续掉 8 帧)。越过界的段不是错值: 回卷使累计行程倒退 → 到不了行程
//   目标 → 按超时判不可测。
inline constexpr float calib_shift_max_px() {
    return 0.5f * CALIB_SAMPLE_SCALE * (float)CALIB_BLOCK_PX;
}

// L 边沿读数与"还在动"判据的量化底 (屏幕 px): 相位相关的亚峰内插分辨率约 1/4 相关域
//   像素 × 尺度 = 0.5px。低于它的位移读数只是内插噪声 — 纯 3σ 门 (数字采集 σ≈0.03px
//   → 0.09px) 会掉到这个底下, 停顿里的噪声尖峰被误判成"还在动", 边沿读数系统性虚抬。
//   两界并用: max(3σ, 本底)。
inline constexpr float calib_edge_floor_px() {
    return 0.25f * CALIB_SAMPLE_SCALE;
}

// ---- 块统计的门 ----
// 块相关峰的纳入门: 只是"该块有没有信号"的取舍 (峰在无纹理块上落在数值噪声量级,
//   在有纹理块上高出一个数量级), 不承载测量语义。
const float CALIB_BLOCK_RESP      = 0.01f;
// 单帧最少纳入块数: 4/9 = 3×3 网格的严格多数 (静止簇剔除与运动簇都要足够的块才有意义)
constexpr int CALIB_BLOCK_MIN     = 4;
// 运动簇最少块数 (少于它退回全体中位): 3 = 中位不被单个异常块翻过去的最小数
constexpr int CALIB_CLUSTER_MIN   = 3;
// 静止簇剔除门 (×最大块位移; 出处见文件头)
constexpr float CALIB_STATIC_FRAC = 0.5f;

// 单轴的单帧块统计 (纯函数, 与相机/模型无关 — 单测直接喂合成块):
//   n = 纳入块数 (相关峰已过门, 上界 CALIB_BLOCKS_N), shift = 各块沿该轴的位移
//   (**相关域** px, 正 = 内容朝 +轴 移动), resp = 各块相关峰。返回屏幕 px 的帧位移
//   (已含尺度与方向: 画面位移 = −内容位移 = 准星/命令方向) 与两个质量量。
struct CalAxisStats {
    bool  ok = false;              // 纳入块数 ≥ CALIB_BLOCK_MIN
    int   n_used = 0;              // 纳入块数 (峰过门)
    int   n_static = 0;            // 其中被静止簇剔除的块数
    bool  fallback = false;        // 运动簇 < 3 块 → 退回全体中位 (兜底)
    float shift = 0;               // 帧位移 (屏幕 px, 沿该轴)
    float shift_all = 0;           // 静止簇剔除前的全体中位 (诊断对照)
    float resp = 0;                // 运动簇相关峰中位
    float spread = 0;              // 运动簇内相对中位的最大偏离 (屏幕 px)
};
CalAxisStats calib_axis_stats(int n, const float* shift, const float* resp);

// 一维相位相关: 两条等长 CV_32F 行向量 + 同长汉宁窗 → 亚像素位移 (相关域 px, 循环;
//   正 = 内容朝 +轴 移动, 与 cv::phaseCorrelate 同口径) 与峰 (0–1; 白化互谱的逆变换
//   峰, 完全一致时 ≈1)。长度为域内块宽 (106)。
double calib_pc1d(const cv::Mat& a, const cv::Mat& b, const cv::Mat& win, double* resp);

// 采样器: 逐帧调用, 缓存汉宁窗与投影缓冲 (逐帧分配会拖慢标定期)。入参是相邻两帧的
//   相关域图 (CV_32F, 320×320)。输出两轴各自的单帧统计 — 两条轴各一次投影相关;
//   只被激励的轴进判定, 另一轴是"画面自己走了多少"的现场读数。
class CalibSampler {
public:
    struct Frame {
        bool   ok[2] = {false, false};   // 该轴本帧是否出样本
        float  shift[2] = {0, 0};        // 帧位移 (屏幕 px, 沿轴)
        float  shift_all[2] = {0, 0};    // 剔除前对照
        float  resp[2] = {0, 0};
        float  spread[2] = {0, 0};
        int    n_static[2] = {0, 0};
    };
    CalibSampler();
    Frame measure(const cv::Mat& prev_f32, const cv::Mat& cur_f32);
private:
    cv::Mat win_row_, win_col_;          // 1×B / B×1 汉宁窗 (逐帧重建窗会拖慢标定期)
};

// 单帧样本: 相邻两帧的一维投影位移 (sx/sy 屏幕 px, 沿命令方向) 与质量量 —
//   ok[axis] = 该轴本帧出了样本 (另一轴可以是空的), resp/spread = 该轴运动簇的相关峰
//   中位 / 簇内最大偏离, sx_all/sy_all = 静止簇剔除前的全体中位 (诊断对照),
//   n_static = 被剔除的块数。slot = 采样这一刻正在播放的激励段槽位 (−1 = 停顿/未标定),
//   由采样侧与在线累计同一次判定给出 (见 io/calib_run.h)。
struct CalibSample {
    std::chrono::steady_clock::time_point t;
    float dt_ms = 0;                     // 距上一有效样本的实际间隔 (掉帧即变大)
    float sx = 0, sy = 0;
    float sx_all = 0, sy_all = 0;
    bool  ok[2] = {false, false};
    float resp[2] = {0, 0}, spread[2] = {0, 0};
    int   n_static[2] = {0, 0};
    int   slot = -1;
};

// 标定回写: 只替换以 var 开头的行为值 (缺行追加到末尾), 临时文件 + rename 原子替换,
//   原文件权限/属主继承。VAR 名由调用方给 (hid: L_EST / pad: L_EST_PAD — 两套互不覆盖),
//   机制与名无关。临时名带 pid: 同一脚本可能同时被另一写者改写 (webui 保存用它自己的
//   临时文件), 共用一个 ".tmp" 会让两份内容在重命名前互相穿插。
bool persist_calibration(const std::string& path, const std::string& var, float l);
std::string resolve_cam_device(const std::string& spec);
