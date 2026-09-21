// ============================================================================
//  calib_run.h — 标定执行侧 (hid 与 pad 共用同一个引擎: 同一套测量法、同一份状态机、
//    同一份拟合; 两模式只给"激励计划 + 注入单位 + 聚合/回写 VAR 名")。
//
//  标定只出**环路延迟 L**。速度是四个逐轴倍率的手动项 (core/state.h 的 spd), 标定
//  既不测速度也不回写速度 —— 激励段里顺带量到的屏速只进日志, 供现场手算 spd 起点。
//
//  --- 观测模型 (一切读数的出发点) ---
//  命令在 t₀ 变 (+激励), 画面在 t₀+L 才动: 第 k 帧 (时刻 tₖ) 的位移 = 画面在
//  [tₖ−dt−L, tₖ−L] 上真正走过的距离, 其中 dt 是该样本距上一帧的实测间隔。
//  于是"命令沿 → 画面沿"的时距就是 L —— 三种彼此独立的读数都从同一条观测流上取:
//    [尾迹和] 停止命令之后画面还会走 v·L (v = 该段实测屏速)。停顿窗内的位移和
//      = v·(t₁ − t_first + dt_first + L) → **L = Σ/v + (t_first − t₁) − dt_first**。
//      亚帧精确 (位移是精确积分), 且对掉帧免疫: 和式在首末两个世界时刻之间望远镜式
//      相消, 中间缺几帧不改变它。主读数。
//    [停止沿] 画面在观测里最后"还在动"的样本 t_j 满足 L ∈ (t_j−dt_j−t₁, t_j−t₁]
//      → L = t_j − t₁ − dt_j/2。增益无关, 量化到帧格 (中位跨段相消)。
//    [起始沿] 画面开始动的第一个样本 t_k 满足 L ∈ [t_k−dt_k−t₀, t_k−t₀)
//      → L = t_k − t₀ − dt_k/2。同样量化到帧格。它同时是"这一段确实响应了"的证据。
//  两条边沿读数用同一条中位括号规则 (±dt/2, 对称), 尾迹读数没有量化 — 三者互相
//  独立 (增益/噪声来源都不同), 并列报出; 尾迹与边沿的系统性分歧是现场最有用的诊断
//  (命令→画面这条通道里还有别的滞后时才会出现)。
//
//  --- 激励形状 (为延迟重推: 只要"命令沿 → 画面沿"一对边沿, 不要速度梯度) ---
//  每轴: 静默 → 保持 +d 直到行程到位 → 停 → 停顿 (静默参考) → 反向重复, 符号交替
//  使画面回到起点。振幅由**屏幕速度目标** v* 导出, 三条约束都给算式:
//    (1) 段跨 ≥ 6 帧 (实测采样率下): 段长 T = A/v* ⇒ **v* ≤ A/(6·dt_slow)**。取设计
//        地板 dt_slow = 1/60s (要求"不假设 120fps"的落点: 60fps 下 6 帧、120fps 下
//        12 帧都成立), 于是 v*[H] = 150px/100ms = 1.5px/ms, v*[Y] = 100px/100ms
//        = 1.0px/ms (A 见行程目标)。采样率实测后照进日志; 段跨再短也只是读数更粗。
//    (2) 每帧位移落在相关域可靠范围内: v*·dt ≤ calib_shift_max_px() (106px@120fps
//        = 12700px/s, 设计值 12.5px/帧) — 6 个数量级余量, 只对"游戏比假定快很多"
//        起兜底作用 (越界段按不可测丢弃, 不是错值)。
//    (3) 尾迹 v·L 远大于噪声: 尾迹读数的时间分辨率为 σ√n/v (n = 停顿窗样本数),
//        要求它 ≤ 边沿读数的量化 dt/2 ⇒ v ≥ 2σ√n/dt。实测 σ 0.03–0.1px/帧 (数字采集,
//        半分辨率域), n≈20, dt=8.33ms → v ≥ 0.11px/ms; 设计值 1.5px/ms, 13 倍余量。
//    换算成注入量:
//      hid: 速率 r = v*/s_hid_now(ads, axis) (counts/ms; s 是 spd 口径的有效灵敏度,
//        没有 S_EST 那一层), 按每拍整数量化 + 余量累加 (平均率精确)。
//      pad: 偏转 d = clamp(v*·1000/gain_pad_eff(spd_axis(ads,axis)), CAL_PAD_DEFL_MIN, 1)
//        — 上界 1 = 满偏 (物理行程), 下界 0.30 = 实测死区地板 (10% 偏转几乎不动
//        画面的现场读数; 该轴有效增益被 spd 调得很大时 d 会撞上 1, 段随之变长,
//        段长是测量不是参数)。
//    轴覆盖: hid 两轴 (鼠标游戏的逐轴平滑策略可能不同; 两轴结果并列报出), pad 只扫
//    水平轴 (垂直通道被俯仰夹紧与旋转辅助瞄准污染 — 实测标定期间旁轴持续被牵引)。
//
//  --- 行程目标 ---
//  水平 150px / 垂直 100px。出处: 操作者可接受的行程上限 (150px 在 24" 2K 屏上约
//  4cm, 且落在自瞄自己的交战圈 FOV 半径 150px 内 — 增益正是在瞄准真正发生的屏幕区域
//  里测的), 同时满足上界 = 投影保真 (屏幕平移与真实相机旋转的差异在离中心 x 处 ∝
//  (x/f)², f≈960px@1080p → 端点 ≤150px 时 ≤2.4%) 与下界 = 显著高于噪声底
//  (150px 高出 σ√n ≈ 0.5px 两个数量级)。垂直取 100 更保守, 避开俯仰夹紧。
//  段长不预设: "行程到位即停" (采样侧在线累计, 状态机读到累计 ≥ 目标即停段) —— 段长
//  是测量出来的, 没有可预测的量。
//  超时 (ms) = 2000: 150px/2000ms = 75px/s 是"还算响应"的最低屏速; 到不了就是死区/
//  太慢/夹紧/无游戏, 判不可测并报出实测行程与这条速度上界。
//
//  --- 停顿长度与 L 上界是同一个决定 ---
//  停顿 (300ms) 要同时装下三件事: (a) 尾迹本体 (v·L) — 要求停顿长于 L 上界;
//  (b) 静止参考窗 (σ 的来源) 必须完全静止 — 它得在"最后一个还含运动的样本"之后;
//  (c) 下一段的段首留出 L + 帧长的余量 (否则前段尾动被算进下一段)。
//  于是静止参考窗起点 S = L_MAX + 2·dt_60 = 120 + 33.3 → 取 160ms (两个帧格的余量:
//  一个是被测样本自身窗口跨过停止时刻, 一个是采样网格相位), 静止窗 = 停顿的最后
//  140ms ≥ σ 的样本量要求 (池化 20 个样本 = 中位估计的相对误差 ≤30%; 140ms 在 60fps
//  下 8.4 帧、120fps 下 16.8 帧, 每轴 4 个停顿 → 34/67 个样本, 达标)。
//  停顿 P = S + 静止窗 = 160 + 140 = 300ms。
//  自校验与此同界: 实测 L 超过 **S** 即判失败 —— "窗内已静止"这一假设与拒绝条件必须
//  是同一条线, 否则两者之间那一段会被静默采信; 尾迹被截断时读数**恰好饱和在停顿长度**
//  上, 所以这条检查既拦"停顿太短"也拦"窗口里还有运动的尾部"。L > L_MAX 同样失败
//  (测量超出设计带 = 测量无效, 不硬钳制)。
//
//  --- 门 (只留与延迟有关的; 阈值与出处) ---
//   行程到位        | 状态机"到位即停"; 超时 = 不可测 (75px/s 上界)
//   每帧位移量程     | calib_shift_max_px() = 域内块宽一半 × 尺度 = 106px (循环相关的
//                     | 峰唯一界, 实测域内 52px 内精确)
//   块间离散         | 段内**逐帧中位** ≤ 量程 (单帧歧义不废整段 — 与块统计"单块异常
//                     | 顶不动中位"同一纪律); 帧内最大离散只记不判
//   纹理地板         | 相关峰中位 ≥ 0.05 (纳入门 0.01 的 5 倍: 峰高出旁瓣一个量级)
//   测量窗           | 两条穿越之间 ≥ 一个实测帧长 (夹不住就只能外推, 分母无界)
//   停顿长于 L 上界  | 尾迹读数 ≤ S(=160ms) 且 |读数| ≤ 停顿; L > L_MAX 失败并说明
//   散度             | 每个读数族的 MAD ≤ 一个**实测**采样间隔 dt (读数的量化单位;
//                     | 超一格说明它们不是同一次物理测量的重复)
//   两读数一致性     | |中位(尾迹) − 中位(停止沿)| ≤ 3·√(SE² 之和), SE 用族内 MAD 与
//                     | 量化底 dt/√12 的较大者 (3σ 统计门; 量化底必须进 SE — 边沿读数
//                     | 的量化是物理的, 不像相位相关噪声那样可以忽略)
//   L 物理带         | [L_MIN, L_MAX] = [0, 120] (上界出处见 core/calib.h)
//   运动前置         | 至少一段测到运动, 否则整轮失败 ("屏幕未响应")
//   读数数量         | 每轴 2 对 (4 段 × 3 读数 = 12 个读数/轴) ≥ 5 (中位数 + MAD
//                     | 的最小样本量; 每段给 3 个读数, 2 对就有 2.4 倍余量)
//  聚合: 每个读数族取**中位 + MAD**; 最终 L = 尾迹族的中位 (主读数; 尾迹族全不可用时
//  退回停止沿族并在日志里说明)。hid 的逐轴中位并列报出: 两轴一致 (在 3·SE 内) 则
//  合并成一个数, 不一致就把两侧数字都打出来并记下结论 (两轴各是一个独立读数).
//
//  --- 采样率不假设 120fps ---
//  所有量都用真实时间戳与实测 dt: 读数分辨率 = 边沿的 ±dt/2, 散度门 = dt, 段跨 =
//  T/dt 帧。低帧率/掉帧只让读数更粗 (如实反映在 MAD 上), 不改变方法; 每轮日志给出
//  实测采样率、采集率与无样本帧占比 (见 io/capture.cu)。采样率低于采集率 (推理被
//  跳过之外的原因) 同样只是一条日志事实。
//
//  --- 状态机 (该模式的 1kHz 拍驱动) ---
//  空闲 → 触发 → 起始十字 (视觉开始信号 + 触发后的准备期: 操作者需要时间把手拿开)
//  → 逐段激励 (每段后停顿; 到位即停/超时即段止; 一个轴连续两段超时 = 该轴不响应,
//  跳过它剩下的段) → 收尾静置 → 等采样侧拟合回执 → 点头 (成功) / 摇头 (失败)。
//  激励期整条注入通道归激励独占 (hid: 报文的位移字节; pad: pad_excite 把整只手柄
//  归中并把激励写在右摇杆上 — 人类摇杆/扳机归中、按键照旧透传, L3/R3 才到得了游戏)。
//  接管关闭 (-a n / aim=0) 时标定不可达并复位 (激励是程序注入的移动, 与纯透传互斥)。
//  上一轮的回执在每轮开跑时清零 —— 不清则第二轮起会立刻读到旧值 (点头/摇头报的是
//  上一轮的结论)。热参请求在标定进行中到达: 一次消费即清并记一行丢弃, 绝不重入。
//
//  --- 回写 ---
//  成功只写该模式的一条 VAR: hid → L_EST, pad → L_EST_PAD (persist_calibration 原子
//  替换), 运行进程内的 l_est 由采集侧接着更新。失败什么都不写。
// ============================================================================

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include "core/calib.h"                  // CalibSample / 采样几何 / L_MIN,L_MAX
#include "core/state.h"                  // ms_to_ticks / DEFAULT_FREQ
#include "io/pad_input.h"                // PAD_AXIS_MAX (pad 偏转单位)

// ---- 模式 (各一份状态; 一次运行只居其一) ----
enum CalMode { CAL_MODE_HID = 0, CAL_MODE_PAD = 1 };
constexpr int CAL_MODES_N = 2;

// ---- 计划参数 (推导见文件头) ----
// 回写的 VAR 名: 一个输出模式一个, 两套互不覆盖 (速度 VAR 是手动项, 固件永不写)
constexpr const char* CAL_VAR_HID = "L_EST";
constexpr const char* CAL_VAR_PAD = "L_EST_PAD";
// 每轴对 (方向 +/−) 数: 读数族要有 ≥ CAL_MIN_READINGS 条才谈得上"中位 + MAD".
//   pad 只扫一轴 → 该轴的对数就是全轮的对数 → 3 对 = 6 个尾迹读数 (>5);
//   hid 两轴合并 → 2 对 = 8 条 (>5), 逐轴 4 条只作并列报出的诊断。
constexpr int   CAL_PAIRS_HID = 2, CAL_PAIRS_PAD = 3;
inline int cal_pairs(CalMode m) { return m == CAL_MODE_PAD ? CAL_PAIRS_PAD : CAL_PAIRS_HID; }
constexpr float CAL_TRAVEL_PX[2]   = {150.0f, 100.0f};   // [0]=X / [1]=Y 行程目标
constexpr int   CAL_SEG_TIMEOUT_MS = 2000;               // 不可测判据 (75px/s 屏速上界)
constexpr int   CAL_PAUSE_MS       = 300;                // 段后停顿 (含静止参考窗)
constexpr int   CAL_STATIC_FROM_MS = 160;                // 静止参考窗起点 (= L_MAX + 2 帧@60fps)
constexpr int   CAL_SEG_MIN_MS     = 100;                // 段跨设计地板 = 6 帧 @60fps
constexpr float CAL_PAD_DEFL_MIN   = 0.30f;              // pad 激励偏转下限 (实测死区地板)
constexpr int   CAL_ABORT_N        = 2;                  // 一个轴连续 N 段超时 → 该轴不响应
constexpr int   CAL_AXES_MAX       = 2;                  // hid 两轴 / pad 一轴
// 计划表上限 (轴 × 方向 × 对 × (激励+停顿)) — 诊断数组按它定长
constexpr int   CAL_SEGS_MAX       = CAL_AXES_MAX * 2 * CAL_PAIRS_PAD * 2;

constexpr float CAL_EDGE_SNR       = 3.0f;   // 边沿/响应检出 (σ 倍数)
constexpr float CAL_RESP_MIN       = 0.05f;  // 相关峰中位下限 (纹理地板; 纳入门的 5 倍)
constexpr int   CAL_SIGMA_MIN_N    = 20;     // 静止窗池化样本数下限 (中位 SE ≤30%)
constexpr int   CAL_MIN_READINGS   = 5;      // 每个读数族的最少读数 (中位+MAD 的最小样本量)
constexpr float CAL_DISP_FRAMES    = 1.0f;   // 散度门 (× 实测采样间隔 dt)
constexpr float CAL_CONSIST_Z      = 3.0f;   // 两读数一致性 (3 倍合并标准误)
constexpr float CAL_EDGE_TOL_REL   = 0.30f;  // 一致性容差的相对项 (× L; 出处见 cal_fit)

// ---- 激励计划 ----
// 一条计划项 = 一个激励段 + 紧跟的停顿段。amp 的单位随模式 (hid: counts/ms; pad: 满偏比例)。
struct CalPlanSeg {
    int    axis = 0;        // 0=X / 1=Y
    int8_t dir  = 0;        // 激励方向 +1/−1 (停顿段为 0)
    bool   pause = false;
    float  amp = 0.0f;      // 激励幅度 (暂停段无意义)
};
// 计划表 (每轴 cal_pairs 对, 符号交替 [+,−,+,−]: 画面回到起点, 每方向各半)
std::vector<CalPlanSeg> cal_plan(CalMode m);
// 一轮的名义时长 (ms): 全部段撞超时的最坏情形 (操作者据此知道手要离开多久)
int cal_plan_worst_ms(CalMode m);
// 采样窗深度 (帧): 覆盖整轮的最坏时长 (超时 + 停顿) + 收尾
int cal_hist_frames(CalMode m, int cam_fps);

// ---- 段窗口表 (状态机写, 采样侧与拟合读) ----
struct CalSegWin {
    int    axis = 0;
    int8_t dir  = 0;
    bool   pause = false;
    float  amp = 0.0f;
    bool   begun = false;        // 已开播 (未触碰的槽位与已播放的区分)
    bool   skipped = false;      // 该轴不响应, 被跳过 (不出现在拟合里)
    bool   timeout = false;      // 因超时结束 (行程未达目标)
    bool   moved = false;        // 状态机读到过非零累计行程 (播放期的现场记录)
    std::chrono::steady_clock::time_point t0{}, t1{};
};
class CalWinTable {
public:
    void reset(CalMode m);
    void begin_seg(int i, std::chrono::steady_clock::time_point t);
    void end_seg(int i, std::chrono::steady_clock::time_point t, bool timeout, bool moved);
    void mark_skipped(int i);
    void clear();
    std::vector<CalSegWin> snapshot() const;
    size_t size() const;
private:
    mutable std::mutex mtx_;
    std::vector<CalSegWin> segs_;
};
extern CalWinTable g_cal_win;

// ---- 采样端在线累计行程 (采集线程逐帧) ----
// 找到当前激励段的槽位, 把该帧位移沿激励方向的投影累加进 g_cal_live_travel (屏幕 px);
// 槽位变化即清零重计。状态机每拍读它判"到位即停"。返回该样本所属的槽位 (−1 = 停顿
// 或未标定), 采集线程把它记进样本 —— 于是状态机的累计与拟合重算的是同一批样本。
int cal_note_sample(const CalibSample& s);
extern std::atomic<float> g_cal_live_travel;   // px (沿激励方向, 从段首累计)
extern std::atomic<int>   g_cal_live_slot;     // 对应计划槽位 (-1 = 无激励段)

// ---- 拟合结果 ----
// 逐段诊断 (日志与单测用): 一个激励段一条, 三个读数各自是否成立
struct CalSegDiag {
    bool  played = false;          // 该槽位确实播放过 (未被跳过)
    int   axis = 0;
    int8_t dir = 0;
    float amp = 0;                 // 激励幅度 (hid: counts/ms; pad: 满偏比例)
    bool  ok = false;              // 该段读出了屏速 (尾迹读数的前提)
    const char* why = "";          // 不可测原因
    float val = 0, lim = 0;        // 触发门限时的实测值与门限
    float travel = 0;              // 检出点→目标穿越之间的实测行程 (px)
    float t_ms = 0;                // 同上耗时 (ms)
    float v = 0;                   // 该段实测屏速 (px/s; 只作尾迹的除数与现场读数)
    float resp = 0;                // 相关峰中位
    float disp_med = 0, disp_max = 0;   // 每帧位移中位/最大 (px)
    float spread = 0, spread_max = 0;   // 块间离散 帧中位/帧最大 (px)
    float raw_med = 0;             // 静止块剔除前的每帧位移中位 (对照)
    float l_onset = -1, l_stop = -1, l_tail = -1;  // 三个读数 (−1 = 该读数不可用)
    float travel_end = 0;          // 段末累计行程 (px; 现场核对窗口是否被截断)
};
struct CalResult {
    bool  ok = false;
    const char* err = "";          // 整体失败原因
    float sigma[2] = {0, 0};       // 停顿静止窗估出的噪声底 (px/帧, 逐轴)
    int   sigma_n[2] = {0, 0};
    float l_est = 0;               // 回写值 (仅 ok 时有效)
    // 读数族 (逐轴与全轮): 中位 + MAD + 条数
    float l_tail = 0, l_tail_mad = 0;     int l_tail_n = 0;
    float l_stop = 0, l_stop_mad = 0;     int l_stop_n = 0;
    float l_onset = 0, l_onset_mad = 0;   int l_onset_n = 0;
    float l_axis[2] = {0, 0};             int l_axis_n[2] = {0, 0};   // 逐轴主读数 (尾迹族)
    float consist_diff = 0, consist_tol = 0;   // 尾迹−停止沿 的差与其容差 (诊断行)
    bool  axes_agree = false;             // 两轴一致 (hid; 单轴模式恒 true)
    float dt_ms = 0;                      // 实测采样间隔中位 (读数分辨率的出处)
    bool  used_edges = false;             // 尾迹族不可用, 退回停止沿族
    int   n_seg_ok = 0, n_seg_all = 0;
    CalSegDiag diag[CAL_SEGS_MAX];             // 按计划槽位索引 (只有激励段被填)
};

// 拟合: 历史样本 + 段窗口快照 → 结果 (账本不参与: 测量完全来自屏幕位移)。
CalResult cal_fit(CalMode mode, const std::deque<CalibSample>& hist,
                  const std::vector<CalSegWin>& plan);

// 回执码: 1 = 成功 (延迟实测出来且落在物理带内), 2 = 失败。成功判定唯一下游 = L。
int cal_done_code(const CalResult& r);

// 诊断行 (无论成败都打 — 失败时它就是现场证据): 逐段原始行 + 读数族汇总 + 模式结论
void cal_print_diag(CalMode mode, const CalResult& r, size_t hist_n);

// 回写 (成功路径): 只写该模式的一条延迟 VAR (hid: L_EST / pad: L_EST_PAD; 原子替换)。
// persist_path 为空则跳过。返回是否写入成功。
bool cal_writeback(CalMode mode, const CalResult& r, const std::string& persist_path);

// ---- 状态机 ----
// btns: 人类逻辑键位字 (hid: HID 按钮位; pad: PadBtn 位表)。返回本拍是否处于标定中及
//   本拍的注入命令 (hid: counts; pad: 摇杆偏转 −32767..32767)。active 时调用方让位。
struct CalStep { bool active = false; int16_t cx = 0, cy = 0; };
// now = 本拍的时刻 (拍线程传入自己的时钟; 单测传合成时间轴, 于是整轮可离线重放)
CalStep cal_step(CalMode mode, uint16_t btns, int cam_fps, std::chrono::steady_clock::time_point now);
