// ============================================================================
//  calib.h — 灵敏度与环路延迟 L (ms) 的在线标定: 最小二乘估计 + 延迟粗/细双扫
//    (run_calibration), 标定值脚本原子回写 (persist_calibration), 采集卡设备名
//    解析 (resolve_cam_device), 以及 CalibSeg 激励轨迹表 — 轨迹由标定状态机
//    播放 (hid: core/control.cu 的 law_tick, pad: io/pad_calib.cu), 采样在
//    io/capture.cu (块相位相关, 不依赖 AI 检测)。
//
//  灵敏度单位制随输出模式, 拟合数学与回写机制两模式共用:
//    hid: px/count, 账本 = g_counts (鼠标实际 counts)
//    pad: px per (偏转·ms), 账本 = 摇杆账本 (合并偏转 × 拍时长), 满偏屏速 =
//         灵敏度 × PAD_AXIS_MAX × 1000 (io/pad_calib.h)
//  账本来源经 own_motion_ledger() 取 — 模式路由, 消费端数学逐字符一致;
//  唯一随模式变的是灵敏度钳制带 (CalibBand)。
// ============================================================================

#pragma once

#include <chrono>
#include <deque>
#include <string>

#include "core/state.h"        // DEFAULT_FREQ / ms_to_ticks — 拍数一律按墙钟导出

// ========================= 标定 (以拍计的时长由墙钟导出, 换拍率不改语义) =========================
const int   CALIB_TRIGGER_TICKS    = ms_to_ticks(5000);   // 双侧键长按 5s
const int   CALIB_WINDOW           = 90;             // 最少样本帧数
const int   CALIB_HIST_FRAMES      = 300;            // hid 采样窗深度 (帧): ~2.5s @120fps — 周期方波只需尾窗 (pad 的
                                                     //   分级激励不周期, 窗深由 io/pad_calib.h 按计划时长导出)
// 块相位相关的采样门: 单块响应的纳入下限 (归一化相关峰 — 低于它的块"无信号",
//   不参与中位) 与单帧最少纳入块数。两者是既有采样语义 (hid 标定同样走这条),
//   pad 的"该级能否作测量依据"另有一道更高的级响应门 (io/pad_calib.h)。
const float CALIB_BLOCK_RESP       = 0.01f;
const int   CALIB_BLOCK_MIN        = 4;
const float CALIB_MIN_EXCITE       = 4000.0f;        // 最小 ΣC² 激发量
const float S_MIN = 0.05f, S_MAX = 20.0f;
const float L_MIN = 0.0f,  L_MAX = 200.0f;
const int   CALIB_WAIT_TIMEOUT     = ms_to_ticks(2000);   // 等待计算超时 2s

struct CalibSeg { int dx, dy, ticks; };
// 激励轨迹: 每拍位移 (counts) × 拍数; 拍数由段墙钟时长导出, 段速度为设计量 —
//   激励腿 2px/拍 = 2000 counts/s (s=1 时即速度帽量级), 收尾甩动 4px/拍 =
//   4000 counts/s。
// **以起点为中心的十字**: 腿序 +x → −x → −x → +x → +y → −y → −y → +y, 每条腿从静止
//   出发、回到静止 (激励单圈每腿后接零指令停顿, 起始十字不接停顿只作视觉信号)。
//   为什么每轴要 4 条腿: 行程 = 各腿位移的**累积和**, 一条腿只把准星从起点推出去 A;
//   要让它落到起点两侧必须再有反向腿把准星拉回来 —— 部分和因此是
//   0 → +A → 0 → −A → 0, 包络 = 起点 ±A。老方波 (+x,+y,−x,−y) 的部分和是
//   0 → +A → (+A,+A) → (0,+A) → 0, 行程单侧落在起点右下方 → 操作者必须"故意从
//   左上角起"才不出屏, 而系统光标一旦夹边相机就不再响应 → 该段测量被污染。
//   其余两条设计后果: 每圈回到起点 → 画面内容保持相似, 块相位相关更稳;
//   每条腿的启停都是干净边沿 → 每圈 16 个边沿 (老方波 4 个) → lag 对齐回归的辨识更强。
//   单腿位移与老方波同量级 (2px/拍 × 250ms = 500 counts), 拟合统计量可比。
// 腿间零指令停顿 = 与 pad 的 PAUSE 同一规则: 用户保证的 L 上界 100ms + 余量 (≥2 帧
//   @120fps) = 116.7ms → 上取 50ms 的整值 150ms。
// **停顿的收益 (量化的; 不是"防偏小")**: run_calibration 是 lag 对齐回归, 折返处取的
//   是区间聚合量 (窗口内指令的混合值与画面位移同口径), 故折返本身不产生系统性偏小 —
//   这正是正方形一直能标对 s 的原因。停顿的真实收益是三项: (1) **稀释折返邻域样本** —
//   只有折返附近的样本对 lag 的量化误差敏感, 腿内稳态样本对平移不敏感, 停顿把腿内样本
//   占比提上去, 那份误差的权重随之下降; (2) **锐化 lag 辨识** — 启停边沿更干净, 相关峰
//   更尖; (3) 停顿段静止 → 顺带得到噪声底 σ (与 pad 的 pause 同源, 只进日志诊断 —
//   hid 的验收判据 S_MIN/S_MAX 与样本数判据一动不改)。合成对照见 io/pad_test.cu [20]。
inline const CalibSeg CAL_START_SEQ[] = {          // 起始十字 (纯视觉信号, 不采样)
    {2,0,ms_to_ticks(240)},{-2,0,ms_to_ticks(240)},
    {-2,0,ms_to_ticks(240)},{2,0,ms_to_ticks(240)},
    {0,2,ms_to_ticks(240)},{0,-2,ms_to_ticks(240)},
    {0,-2,ms_to_ticks(240)},{0,2,ms_to_ticks(240)},
    {0,0,ms_to_ticks(500)}};
// 激励单圈基元: 8 腿 × (腿 250ms + 停顿 150ms) = 3200ms
constexpr int CAL_EXCITE_LOOPS = 3;                // 3 圈 × 8 腿 = 24 条腿 (老方波 5 圈 × 4 = 20)
inline const CalibSeg CAL_EXCITE_SEQ[] = {
    {2,0,ms_to_ticks(250)},{0,0,ms_to_ticks(150)},
    {-2,0,ms_to_ticks(250)},{0,0,ms_to_ticks(150)},
    {-2,0,ms_to_ticks(250)},{0,0,ms_to_ticks(150)},
    {2,0,ms_to_ticks(250)},{0,0,ms_to_ticks(150)},
    {0,2,ms_to_ticks(250)},{0,0,ms_to_ticks(150)},
    {0,-2,ms_to_ticks(250)},{0,0,ms_to_ticks(150)},
    {0,-2,ms_to_ticks(250)},{0,0,ms_to_ticks(150)},
    {0,2,ms_to_ticks(250)},{0,0,ms_to_ticks(150)}};
inline const CalibSeg CAL_SETTLE_SEQ[] = {{0,0,ms_to_ticks(300)}};
inline const CalibSeg CAL_END_OK_SEQ[] = {
    {0,4,ms_to_ticks(60)},{0,-4,ms_to_ticks(60)},{0,4,ms_to_ticks(60)},
    {0,-4,ms_to_ticks(60)},{0,4,ms_to_ticks(60)},{0,-4,ms_to_ticks(60)}};
inline const CalibSeg CAL_END_FAIL_SEQ[] = {
    {4,0,ms_to_ticks(60)},{-4,0,ms_to_ticks(60)},{4,0,ms_to_ticks(60)},
    {-4,0,ms_to_ticks(60)},{4,0,ms_to_ticks(60)},{-4,0,ms_to_ticks(60)}};

// 单帧样本: 相邻帧的块相位相关中位位移 (sx/sy, 全分辨率 px, 已含报告位移的 ×2 还原)
//   与两个采样质量量 — resp = 参与中位的各块相关峰中位 (归一化峰高), spread =
//   各块位移相对中位的最大偏离 (全分辨率 px)。resp/spread 供 pad 的级有效性判定
//   (io/pad_calib.h), hid 的拟合不读它们 (既有语义不变)。
struct CalibSample {
    std::chrono::steady_clock::time_point t;
    float dt_ms, sx, sy;
    float resp = 0, spread = 0;
};

// 灵敏度钳制带 (单位制随模式: hid = px/count, pad = px per 偏转·ms): 拟合结果
//   落带外说明激励/背景不可信, 钳到带边是兜底。hid 侧就是既有的 S_MIN/S_MAX;
//   pad 侧的带由满偏屏速设计带换算 (io/pad_calib.h)。
struct CalibBand { float s_min, s_max; };
inline const CalibBand CALIB_BAND_COUNTS{S_MIN, S_MAX};

bool run_calibration(const std::deque<CalibSample>& hist, float& s_est, float& l_est,
                     const CalibBand& band);

// 标定回写: VAR 名与书写格式由调用方给出 (hid: S_EST/L_EST 两条, pad: 双轴增益 +
//   L_EST_PAD 三条, 见 io/pad_calib.h) — 只替换以该名开头的行为值 (缺行则追加到
//   文件末尾), 临时文件 + rename 原子替换, 原文件权限/属主继承; 机制与名无关,
//   两套互不覆盖。
struct CalibVar { const char* name; float value; const char* fmt; };
bool persist_calibration(const std::string& path, const CalibVar* vars, int n);
std::string resolve_cam_device(const std::string& spec);
