// ============================================================================
//  pad_calib.h — 手柄标定 (pad 模式) 的触发、分轴分级激励计划 (段间停顿 + 中段
//    取样)、逐级增益测量与幂律外推、以及量纲换算。与 hid 的双侧键标定完全独立:
//    分开触发 (L3+R3 长按 / 热参 padcalib), 分开存放 (PAD_STICK_GAIN_X/_Y +
//    L_EST_PAD), 分开回写 (同一原子回写机制, VAR 名不同), 自成一态 (状态机由
//    1kHz pad 拍驱动, 见 io/pad_output.cu 的 pad_tick)。hid 的激励表/拟合调用
//    与本模块无任何共享 (只共用 persist_calibration 与 CalibSeg 类型)。
//
//  标定量:
//    L     (ms)        与 hid 同一物理含义: 注入 → 屏幕 → 采集 的环路延迟 — 单值,
//                      与轴无关 (同一链路)
//    gain_x/gain_y     满偏转屏速 px/s: 各轴右摇杆满偏 (±32767) 的准星屏速 —
//                      注入换算 v·1000/gain_axis·满偏 与 pad 有效速度帽
//                      min(-x, gain_axis/1000) 的来源 (垂直灵敏度常低于水平,
//                      故逐轴标定、逐轴存放)
//
//  测量法 = 段间停顿 + 中段取样 (两条独立的量各自直接可测, 不需要把增益与延迟
//    耦合在一起回归):
//      每轴逐级 d_k 激发, 每级 ± 各一段, **每段后插入零偏转停顿 P**。
//      L: 命令边沿 (偏转归零) → 画面真正停下 的时延。观测模型是"t 时刻拿到的
//        位移 = 世界在 [t−L−dt, t−L] 的位移", 故完全静止的首帧出现在
//        t = p0 + L + dt (p0 = 边沿时刻) → 逐停顿取该时延, 全体取中位 (离散度
//        一并记录)。停顿内的样本必须静止 —— 它们同时是本标的噪声底来源。
//      gain: 每段只取**中段**样本 [段起+P, 段末−帧长] 做 Σ(位移·账本)/Σ|账本|²。
//        段首 P 覆盖"命令换向的观测瞬态"(窗宽 = 帧长 + L ≤ 帧长 + P), 段尾留一帧
//        避免窗跨到停顿 → 中段样本的账本窗完全落在本段内, 与 disp = g·C 逐位
//        自洽, **无需任何延迟对齐**。中段样本数 = (T−P−帧长)·帧率 ≥ 采样下限。
//      **为什么不做"逐级增益 + lag 联合回归"**: 分级响应 (幂律) 下各级位移尺度
//        不同, 单一灵敏度的 lag 扫描会被曲线形状偏置; 而按"段内直接平均"取增益
//        又会把段首 L 的瞬态与段尾跨界样本算进均值 → 增益被 1−2L/T 系统性低估
//        (合成用例 io/pad_test.cu [14] 把这个低估钉住)。停顿法让 L 与 gain 各自
//        直接可测, 两者都不需要对方。
//
//  级集 {10, 25, 50, 70}% (两轴一致): 三点以上才能同时定幂律的两参数并留自校验
//    余量; 低端 10% 是"最灵敏游戏下每帧位移仍在相关量程内"的起点 (设计带上限
//    30000px/s × 10% ÷ 120fps = 25px/帧 ≪ 70px 量程); 高端 70% 一般仍在控制范围内
//    且自带自校验 — 越出量程/被夹紧时该级整级丢弃, 其余级照常拟合。70%→满偏的
//    外推跨度 1.43 是级集内最小的一档 (上界取 50% 则跨度 2, 外推误差同比放大)。
//
//  为什么不能满偏直接测: 测量链是块相位相关 — 半分辨率 cap/2 宽、3×3 块 (半分辨率
//    块宽 B = cap_w/6 ≈ 106px) 取中位位移 ×2 还原 (还原后即 1080p 屏幕像素: 相关在
//    1080p 帧的中心裁剪场上做, 1 px = 1 屏幕 px)。圆相关在 |位移| > B/2 时回卷,
//    峰在逼近它之前已与旁瓣混叠, 可靠每帧位移上界取 B/3 半分辨率 = 2B/3 ≈ 70px
//    全分辨率 (120fps 下 ≈8400px/s)。游戏里满偏转的准星屏速通常高于它 → 每帧位移
//    越界 → 相关峰跳变 → 拟合越界、标定必失败。故改为部分偏转多点测量后外推:
//    对数域最小二乘拟合 gain(d) = A·d^(p−1) → 外推满偏得 A = 满偏转屏速。
//    **已知近似**: 注入仍用满偏单增益 (全行程按线性反解偏转), 不按曲线局部斜率
//    反解 — 控制律是闭环 PI + 前馈, 曲线非线性由环路吸收 (作者定案)。
//
//  自适应段时长 (低挡位先测完就已知增益量级): T = 行程目标/V̂(该级), 夹在
//    [中段统计下限, 流程预算上限] 之间:
//      行程目标 = 屏高的 1/4 — 方向逐段交替, 故一段行程即该级相对起点的最大偏移;
//        1/4 屏既留在俯仰夹紧窗内, 也留在相位相关的每帧量程内 (对不夹紧的 X 轴
//        同样约束量程, 故两轴一致)。
//      V̂ 由已测级按最陡设计曲线外推 (见 pad_cal_seg_ticks): 只高估屏速 → 段长取短
//        → 行程偏小 (安全方向), 反向低估会超行程冲进夹紧。
//      下限 = P + 帧长·(1 + PAD_CAL_SEG_MIN_N): 中段窗 (T−P−帧长) 至少要放得下
//        PAD_CAL_SEG_MIN_N 个样本 — 行程目标要求更短的段时取下限, 该级行程因而可能
//        高于目标; 越界/夹紧由级有效性判据兜底 (整级丢弃), 不静默取值。
//    方向交替 (+,−) 使俯仰围绕水平位往复, 不累积漂移。
//
//  级有效性 (任一条不满足 → 整级丢弃, 原因进日志):
//    - 段中段样本数 ≥ PAD_CAL_SEG_MIN_N。
//    - 激励充分性 (替代 hid 的 ΣC² 判据, pad 单位制下它才真正有效): 以停顿样本
//      估噪声底 σ (1.4826×中位|位移|, 稳健尺度), 段的中段位移须显著正于噪声:
//      Σ(位移沿激励方向) ≥ PAD_CAL_MIN_SNR·σ·√n。低于门 = 屏幕不响应注入 —
//      俯仰夹紧正是这种形态 (位移≈0), 故"夹紧 → 整级丢弃"由本判据实现;
//      符号为负同样被拒 (屏幕动反了 = 激励/背景不可信)。
//    - 相位相关响应: 中段样本响应的中位数 ≥ PAD_CAL_RESP_MIN (归一化相关峰; 采样端
//      0.01 是"该块有信号"的纳入下限, 本门取 5 倍 = 峰高出旁瓣一个量级才认为位移
//      唯一)。
//    - 几何量程: 段内最大每帧位移与 3×3 块间最大离散度均 ≤ 可靠量程
//      pad_cal_shift_max_px(块宽)。回卷时报告位移反而变小, 故越界的主要证据是
//      块间离散度 (各块回卷到不同别名 → 块间不一致) 与中段位移的符号/显著性。
//    - 同级两段一致性: |g₊−g₋| ≤ PAD_CAL_SEG_Z·√(SE₊²+SE₋²), SE = σ/√(Σ|C|²)
//      (由相干统计直接导出的一致性判据) — 方向不该改变响应, 不一致即测量不可信。
//  级数降级: 有效级 ≥2 → 幂律拟合; 恰好 1 级 → 线性回退 (p=1, A = 该级屏速/d,
//    日志标注"单级线性外推, 指数未测"); 0 级 → 失败收尾。
//  延迟自校验: 实测 L 中位数 > P 时停顿留不住瞬态 → 标定失败 (不硬算)。
// ============================================================================

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include "core/calib.h"                  // CalibSeg / CalibSample / L_MIN/L_MAX
#include "core/state.h"                  // ms_to_ticks / DEFAULT_FREQ / g_target
#include "io/pad_input.h"                // PAD_AXIS_MAX
#include "io/pad_output.h"               // PAD_GAIN_MIN/MAX, PAD_LEDGER_TICKS

// L3+R3 长按触发 (与 hid 的 CALIB_TRIGGER_TICKS 同一纪律: 拍数按墙钟导出)
const int PAD_CAL_TRIGGER_TICKS = ms_to_ticks(5000);

// 回写 VAR 名 (脚本 VAR 块; hid 侧为 S_EST/L_EST, 两套互不覆盖)
constexpr const char* PAD_CAL_VAR_GAIN_X = "PAD_STICK_GAIN_X";
constexpr const char* PAD_CAL_VAR_GAIN_Y = "PAD_STICK_GAIN_Y";
constexpr const char* PAD_CAL_VAR_L      = "L_EST_PAD";

// ---- 级集与激励计划 ----
constexpr int PAD_CAL_AXES_N   = 2;      // 0 = X (水平) / 1 = Y (垂直)
constexpr int PAD_CAL_LEVELS_N = 4;      // 级集大小 (两轴一致, 依据见文件头)
inline constexpr float PAD_CAL_LEVELS[PAD_CAL_LEVELS_N] = {0.10f, 0.25f, 0.50f, 0.70f};
constexpr int PAD_CAL_SEGS_PER_LEVEL = 2;      // 每级 ± 各一段 (同级两段即方向一致性对)
constexpr int PAD_CAL_EXCITE_N = PAD_CAL_AXES_N * PAD_CAL_LEVELS_N * PAD_CAL_SEGS_PER_LEVEL;
constexpr int PAD_CAL_SEGS_N   = 2 * PAD_CAL_EXCITE_N;    // 激励段 + 段后停顿

// 段间零偏转停顿 (ms): L 上界 (用户保证 ≤100ms) + 余量 (≥2 帧周期 @120fps = 16.7ms)
//   = 116.7ms → 上取到 50ms 的整值 150ms。停顿要同时装下 (a) 命令换向的观测瞬态
//   (宽 = 帧长+L ≤ 帧长+100ms), (b) 静止参考样本 (噪声底与 L 边沿检测的来源),
//   余量不足就会把"还在动"的样本当成静止参考。
constexpr float PAD_CAL_L_GUARANTEE_MS = 100.0f;
constexpr int   PAD_CAL_PAUSE_MS = 150;

// 段时长上限 (ms) = 计划表的名义段长 = 自适应段时长的上界: 全程最坏 =
//   16 激磁段 × (250+150)ms = 6.4s, 与 10–15s 的标定流程预算同尺度 (另计起始方块
//   1.5s / 静置 0.3s / 收尾 0.7s 与触发前长按 5s)。更长的段只对迟钝到落在设计带
//   下沿之外的游戏有意义, 而那样的游戏本就无法注入。
constexpr int PAD_CAL_SEG_MS_MAX = 250;
// 屏幕高 (px): 采集卡输入格式固定 1920×1080 (NV12 是两种卡唯一同时支持 120fps 的
//   格式), 相位相关位移经 ×2 还原后就是 1080p 屏幕像素 — 行程判据据此按屏高表达。
constexpr int PAD_CAL_SCREEN_H = 1080;
// 单段行程目标 = 屏高的 1/4 (依据见文件头)
constexpr float PAD_CAL_TRAVEL_PX = PAD_CAL_SCREEN_H / 4.0f;
// 段时长预测用的最陡曲线指数: 响应屏速 ∝ 偏转² 是最陡的常见加速曲线, 按它外推
//   只会**高估**屏速 → 段时长取短 → 行程偏小 (安全方向); 反向低估会超行程冲进
//   夹紧。仅用于定段时长, 拟合出的 p 不受此约束 (它只被测量约束)。
constexpr float PAD_CAL_CURVE_P_MAX = 2.0f;

// 计划名义总时长 (ms): 全部段取上限 = 自适应段时长的最坏情形 (账本/采样窗深度
//   按它导出)
constexpr int PAD_CAL_PLAN_MS = PAD_CAL_EXCITE_N * (PAD_CAL_SEG_MS_MAX + PAD_CAL_PAUSE_MS);

// 级偏转 (满偏比例 → counts)
constexpr int16_t pad_level_defl(float d) {
    return (int16_t)(d * (float)PAD_AXIS_MAX + 0.5f);
}
// 视觉信号与激励同为小幅慢速 (25% 偏转), 不再满偏甩动 (用户实测反馈):
//   起始方块 (纯视觉开始信号, 不采样) / 成功点头 / 失败摇头
constexpr int16_t PAD_CAL_ANIM_DEFL = pad_level_defl(PAD_CAL_LEVELS[1]);   // 25%
inline const CalibSeg PAD_CAL_START_SEQ[] = {
    {PAD_CAL_ANIM_DEFL,0,ms_to_ticks(240)},{0,PAD_CAL_ANIM_DEFL,ms_to_ticks(240)},
    {-PAD_CAL_ANIM_DEFL,0,ms_to_ticks(240)},{0,-PAD_CAL_ANIM_DEFL,ms_to_ticks(240)},
    {0,0,ms_to_ticks(500)}};
inline const CalibSeg PAD_CAL_SETTLE_SEQ[] = {{0,0,ms_to_ticks(300)}};
inline const CalibSeg PAD_CAL_END_OK_SEQ[] = {         // 成功 = 纵向点头 2 次
    {0,PAD_CAL_ANIM_DEFL,ms_to_ticks(120)},{0,-PAD_CAL_ANIM_DEFL,ms_to_ticks(120)},
    {0,PAD_CAL_ANIM_DEFL,ms_to_ticks(120)},{0,-PAD_CAL_ANIM_DEFL,ms_to_ticks(120)},
    {0,PAD_CAL_ANIM_DEFL,ms_to_ticks(120)},{0,-PAD_CAL_ANIM_DEFL,ms_to_ticks(120)}};
inline const CalibSeg PAD_CAL_END_FAIL_SEQ[] = {       // 失败 = 横向摇头
    {PAD_CAL_ANIM_DEFL,0,ms_to_ticks(120)},{-PAD_CAL_ANIM_DEFL,0,ms_to_ticks(120)},
    {PAD_CAL_ANIM_DEFL,0,ms_to_ticks(120)},{-PAD_CAL_ANIM_DEFL,0,ms_to_ticks(120)},
    {PAD_CAL_ANIM_DEFL,0,ms_to_ticks(120)},{-PAD_CAL_ANIM_DEFL,0,ms_to_ticks(120)}};

// 账本深度必须覆盖整段激励 + 每样本的延迟/帧长回溯 (L_MAX 200ms + 帧长 ≤100ms),
//   否则早段的账本查询会落到缓冲首样本 (at() 的钳制) 而静默给出 C=0。
static_assert((long)PAD_LEDGER_TICKS * 1000 / DEFAULT_FREQ >= (long)PAD_CAL_PLAN_MS + 300,
              "pad 摇杆账本深度必须覆盖整段激励计划 + 延迟回溯");

// 采样窗深度 (帧, 由采集端按帧率导出): 分级激励不周期, 样本须覆盖整段计划
constexpr int pad_cal_hist_frames(int cam_fps) {
    return (PAD_CAL_PLAN_MS + 1000) * cam_fps / 1000;
}

// ---- 判据常量 (依据见文件头) ----
constexpr int   PAD_CAL_SEG_MIN_N = 4;      // 中段最少样本 (相干统计下限)
// 噪声底 σ̂ 的样本数下限: σ̂ = 1.4826×中位|·| (稳健尺度), 中位估计的标准误差
//   ≈ 1.25/√n, 要相对误差 ≤30% 需 n ≥ 18 → 取 20。
constexpr int   PAD_CAL_SIGMA_MIN_N = 20;
constexpr float PAD_CAL_MIN_SNR   = 5.0f;   // 激励充分性门限 (噪声底 σ 倍数)
constexpr float PAD_CAL_EDGE_SNR  = 3.0f;   // 停顿边沿检测门限 (σ 倍数)
constexpr float PAD_CAL_SEG_Z     = 3.0f;   // 同级两段一致性 (合并标准误倍数)
constexpr float PAD_CAL_RESP_MIN  = 0.05f;  // 中段响应下限 (相关峰中位数)
constexpr float PAD_CAL_PAUSE_TAIL = 2.0f / 3.0f;   // 停顿的静止参考窗起点 (×P)

// 可靠每帧位移上界 (全分辨率 px): 半分辨率块宽 B 的 1/3、报告位移 ×2 还原 —
//   圆相关的回卷边界是 B/2, 峰在逼近它之前已与旁瓣混叠, 可靠上界取 B/3。
inline constexpr float pad_cal_shift_max_px(int bs_half) {
    return 2.0f * (float)bs_half / 3.0f;
}

// 中段统计下限 (ms): 中段窗长 T−P−帧长 至少要放得下 PAD_CAL_SEG_MIN_N 个样本 →
//   T ≥ P + 帧长·(1 + PAD_CAL_SEG_MIN_N)。下限大于上限时取上限 (上限是流程预算的
//   硬约束; 60fps 下 234ms 仍在 250ms 内, 只是自适应余量很窄)。
inline int pad_cal_seg_ms_min(int cam_fps) {
    double frame = 1000.0 / (double)cam_fps;
    int ms = PAD_CAL_PAUSE_MS + (int)(frame * (1 + PAD_CAL_SEG_MIN_N) + 0.5);
    return ms > PAD_CAL_SEG_MS_MAX ? PAD_CAL_SEG_MS_MAX : ms;
}

// 自适应段时长 (拍): 探针级 (尚无实测 v) 取下限; 否则按行程目标与已测屏速导出 —
//   V̂(下一级) = v_meas·(d_next/d_meas)^PAD_CAL_CURVE_P_MAX (安全方向的高估),
//   T = 行程目标/V̂, 再夹在 [下限, 上限]。纯函数: 单测直接断言 (io/pad_test.cu [10])。
int pad_cal_seg_ticks(float v_meas_px_s, float d_meas, float d_next, int cam_fps);

// ---- 自适应段时长的实测输入 (AI 线程写, 状态机读) ----
// "已测级的屏速": 状态机在每级开头读它给该级定段时长。证据不足时保持上次值;
//   每次激励开始时由状态机清零 (探针级 → 段时长取统计下限)。
extern std::atomic<float> g_pad_probe_px_s;   // px/s (0 = 尚无测量)
extern std::atomic<float> g_pad_probe_d;      // 该测量所在的满偏比例

// pad 灵敏度钳制带: 逐轴带内判定 (带外 = 激励/背景不可信 → 标定按失败收尾,
//   带边垃圾值绝不回写)。hid 侧的带内钳制是既有行为 (S_MIN/S_MAX), 此处只约束 pad。
inline bool pad_calib_accept(float gain) {
    return gain > PAD_GAIN_MIN && gain < PAD_GAIN_MAX;
}

// ---- 激励计划与段窗口 ----
// 计划里的一段 (状态机播放; 采样端按本表归段): level = 级序号;
//   pause = true 时是段后的零偏转停顿 (defl = 0, d = 0)
struct PadPlanSeg { int axis; int level; float d; int16_t defl; int ticks; bool pause; };

// 计划表 = 先 X 后 Y, 每轴逐级, 每级 ± 各一段, 每段后接停顿。
//   ticks 为名义段长 (激励段 = 上限, 停顿 = P); 激励段的实际播放时长由状态机按已测
//   增益自适应 (见 pad_cal_seg_ticks), 拟合只依赖段窗口时间戳。
const std::vector<PadPlanSeg>& pad_cal_plan();

// 段窗口 (状态机播放时逐段记录实际墙钟; ai_thread 收到 g_calib_request 后取快照)
struct PadExcSeg {
    int axis = 0, level = 0;
    float d = 0;
    int16_t defl = 0;
    bool pause = false;
    std::chrono::steady_clock::time_point t0{}, t1{};
};

// 段窗口表 (mutex 保护): 播放前建表, 逐段写入 t0/t1; 中断即清空
struct PadExcPlan {
    void reset();                                   // 开播: 按计划建表, 时间戳待写入
    void begin_seg(int i, std::chrono::steady_clock::time_point t);
    void end_seg(int i, std::chrono::steady_clock::time_point t);
    void clear();
    std::vector<PadExcSeg> snapshot() const;
private:
    mutable std::mutex mtx_;
    std::vector<PadExcSeg> segs_;
};
extern PadExcPlan g_pad_exc_plan;

// ---- 拟合结果 ----
// 逐级诊断 (日志与单测用): 实测屏速 px/s (无效 = 0)、有效/总段数、中段响应中位、
//   无效原因 (人可读 — 便于现场判断是不是"游戏太快/太慢/夹紧")
struct PadCalLevelDiag {
    float d = 0;
    float px_s = 0;
    bool  valid = false;
    const char* why = "";      // 丢弃原因
    int   seg_ok = 0, seg_all = 0;
    float resp = 0;            // 中段响应中位
    float val = 0, lim = 0;    // 触发量程/响应/显著性门时的实测值与门限 (日志用)
};
struct PadCalibResult {
    bool  ok = false;                                 // 两轴各有 ≥1 个有效级
    const char* err = "";                             // 整体失败原因 (无有效级/σ/L 不可测)
    bool  linear[PAD_CAL_AXES_N] = {false, false};    // 单级线性回退 (p=1, 指数未测)
    float l_est = 60.0f;                              // 环路延迟 (单值, 停顿实测主值)
    float l_mad = 0.0f;                               // 主值的离散度 (中位绝对偏差)
    int   l_n = 0;                                    // 主值的读数条数
    float l_edge_ms = 0.0f, l_edge_mad = 0.0f;        // 边沿读数 (增益无关的独立交叉检查)
    int   l_edge_n = 0;
    float sigma[PAD_CAL_AXES_N] = {0, 0};             // 停顿样本估出的噪声底 (px/帧)
    float gain[PAD_CAL_AXES_N] = {0, 0};              // 满偏转屏速 px/s (外推 / 回退)
    float p[PAD_CAL_AXES_N] = {1, 1};                 // 幂律指数 (屏速 ∝ d^p)
    float res[PAD_CAL_AXES_N] = {0, 0};               // 对数域拟合残差 (rms)
    int   nlv[PAD_CAL_AXES_N] = {0, 0};               // 有效级数
    PadCalLevelDiag lv[PAD_CAL_AXES_N][PAD_CAL_LEVELS_N];
};

// 拟合: 历史样本 (t/dt/sx/sy/resp/spread) + 段窗口表 + 可靠每帧位移上界
//   (shift_max_px = pad_cal_shift_max_px(cap_w/6), 由采集端按实际块宽给出) → 结果。
//   账本来源 = own_motion_ledger (pad 路由 = 摇杆账本), 由调用方保证模式已设。
PadCalibResult pad_calib_fit(const std::deque<CalibSample>& hist,
                             const std::vector<PadExcSeg>& plan, float shift_max_px);

// 逐帧更新"已测级屏速"探针 (AI 线程在采样期调用): 取最后一段所属的级, 用中段样本
//   池化最小二乘 → 发布 (屏速 px/s, 该级满偏比例)。证据不足则不改。
void pad_calib_update_probe(const std::deque<CalibSample>& hist,
                            const std::vector<PadExcSeg>& plan);

// 标定拍的一步: btns 为人类逻辑按键位表 (PadBtn), 返回值给出本拍是否处于标定中
//   及其右摇杆激励偏转 — active 时 pad_tick 以 pad_excite 独占右摇杆并跳过律。
struct PadCalibStep { bool active; int16_t dx, dy; };

// 状态机一步 (由 1kHz pad 拍每拍调用一次):
//   触发 = 人类 L3+R3 长按 PAD_CAL_TRIGGER_TICKS 拍, 或热参 padcalib=1 (一次
//   消费即清, 与 g_calib_request 同一 exchange 语义); 接管关闭 (-a n / aim=0)
//   时标定不可达且进行中的标定复位 (激励是程序注入的移动, 与纯透传互斥 —
//   与 hid 标定分支同一纪律)。cam_fps 用于中段统计下限与段时长换算。
//   相位/计时器是本模块私有 static, 与 hid 的状态机 (core/control.cu 的
//   law_tick) 无共享。
PadCalibStep pad_calib_step(uint16_t btns, int cam_fps);
