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
//      每轴逐级 d_k 激发, 每级按**对称段序** [+d,−d,−d,+d] 各一段, **每段后插入
//      零偏转停顿 P**。
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
//  级集 {30,40,50,60,70}% (两轴一致): 三点以上才能同时定幂律的两参数并留自校验
//    余量, 五级把拟合点铺在**可用区间**内。低端 30%: 实机反馈 10% 偏转下画面几乎
//    不动 — 该挡落在游戏摇杆的死区/响应曲线起始段, 实测屏速与噪声底同量级
//    (既不表征曲线, 又把"低挡位权重"带进外推, 使 A 系统性偏低); 25% 同样偏冒险,
//    故最低挡取 30%。高端 70%: 仍是"每帧位移通常落在相关量程内"的上界, 到满偏
//    (100%) 的外推跨度 1/0.7 = 1.43 是级集内最小的外推跨度 — 满偏不测 (见下),
//    故外推跨度只能靠"最高挡尽量高"来压缩。
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
//  自适应段时长 = **等行程** (低挡位先测完就已知增益量级):
//    T = 行程目标/V̂(该级), 夹在 [中段统计下限, 段长上限] 之间。
//      行程目标 = 屏高的 1/8 = 135px: **上界**由投影保真给出 — 块相位相关测的是屏幕
//        平移, 而真实链路是相机旋转, 屏幕上离中心 x 处的比例误差 = tan²θ = (x/f)²,
//        f ≈ 960px @1080p, x = 150px 时 2.4% (AGENTS.md 的既测量, ±150px FOV 圈);
//        取屏高/8 使整段偏移区间落在自瞄自身的交战圈 (FOV 半径 150px) 内 — 增益正是
//        在瞄准真正发生的屏幕区域里测的 — 并离 Y 轴俯仰夹紧窗留有量。**下界**由
//        实测散度给出 (可复现规则): 段内累积位移 (≈行程) 须显著高于噪声底, 门限
//        MIN_SNR·σ√n 在竞技噪声 0.5px/帧、下限窗标称 12 个样本下 = 8.7px, 实机噪声
//        0.016px/帧 下 = 0.3px — 135px 分别高出 15× / 460×, 故下界只排除"行程小到与
//        散度同尺"的设计 (实机日志里低挡位零点几 px 的行程正属此类, 它们被显著性门
//        与两段一致性门丢掉)。上下界之间取 135px = 屏高的整数分之一且 < 保真圆。
//      **为什么是等行程而不是等时长**: 测量质量由"累积位移 / 每帧噪声"决定 —
//        中段累积位移 ≈ v·T/1000, 噪声按样本数平均 σ√n = σ√(T/Δt), 故
//        Σdir·disp 的信噪比 ∝ v·√T。要固定信噪比就得 T ∝ 1/v² (慢游戏等平方级
//        加长), 要固定行程则 T ∝ 1/v, 信噪比只按 √(1/v) 缓降; 折中点取等行程 —
//        段长对屏速线性反比, 画面每段走的距离几乎一样, 行程有界可预测 (避免慢
//        游戏把标定甩得过久过远), 慢游戏损失的信噪比由**重播阶梯**补 (§重播)。
//      V̂ 由已测级按最陡设计曲线外推 (见 pad_cal_seg_ticks): 只高估屏速 → 段长取短
//        → 行程偏小 (安全方向), 反向低估会超行程冲进夹紧。
//      行程闭环校正: 段长还按上一级**实测行程/目标行程**的比值做一次乘性修正 —
//        该比值来自位移积分 (与最小二乘斜率独立的量), 吸收外推偏差与观测侧的 L 滞后
//        (段内样本看到的是 [t0−L, t1−L] 的世界, 少走了 v·L)。校正一次一级、无记忆、
//        不进回路: 比值只在上一级"取样充足且未被下限/上限夹住"时使用 (夹住时行程由
//        夹子决定, 比值不含增益信息), 且 T 本身被 [下限, 上限] 夹住 → 极端比值退化为
//        夹子, 不会自激。
//      下限 = P + 帧长·(1 + PAD_CAL_SEG_MIN_N + 余量): 中段窗 (T−P−帧长) 至少要
//        放得下 PAD_CAL_SEG_MIN_N 个样本 **加 4 帧抖动/丢帧余量** — 行程目标要求
//        更短的段时取下限, 该级行程因而可能高于目标 (快游戏); 越界/夹紧由级有效性
//        判据兜底 (整级丢弃), 不静默取值。
//      上限 = PAD_CAL_SEG_MS_MAX: 覆盖"实测采样周期为标称 4 倍"的极端; 撞上限后
//        不再加长 (再长也只对量程外的高速目标有意义), 改由重播阶梯重试。
//      每级进日志的是"目标行程 / 预期行程"与约束来源 (下限/上限/按目标), 现场据此
//        判断该级是被样本数夹住还是被预算夹住。
//    方向交替 (+,−) 使俯仰围绕水平位往复, 不累积漂移。
//
//  重播阶梯 (取样不足时超过行程目标主动多走): 某级播完后, 若该级**已完成的激励段**
//    里中段样本数最小值 < PAD_CAL_SEG_MIN_N, 且段长未达上限、流程预算仍有余, 则该级
//    以 T ← min(2T, 上限) 重播一次 (可连续重播直到上限); 日志标出尝试序号与新段长。
//    重播是"等行程不够可信"的唯一例外路径, 每一步仍受上限与量程门约束。次数由上下限
//    比值导出: 从下限起每次 ×2, 到上限为止 — @120fps 3 次尝试 (258/516/600ms),
//    @60fps 2 次 (367/600ms), 无需另设拍数上限。
//  整轮重跑: 整轮跑完后若"有效级 <2"且**丢级原因只有中段样本不足** (不是显著性/
//    量程/一致性等可信度问题), 允许以**抬高一档的下限** (min(2×下限, 上限)) 重跑整轮
//    一次 (仅一次)。它覆盖阶梯够不到的瞬态: 采样一度中断/处理线程停顿横跨整轮时,
//    级内重播同样落在中断里, 只有重跑整轮才能重新捕获; 由 AI 线程按拟合结果判定
//    (g_calib_done=3), 状态机执行并记日志。
//  流程预算 (PAD_CAL_BUDGET_MS): 重播与重跑都要求"已用时长 + 本次名义时长 ≤ 预算",
//    故激励全程的墙钟有硬上界 = 预算 + 一段在飞段; 摇杆账本与采样窗深度都按它导出。
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
//    - **无纹理与夹紧分开判** (两条互补的门, 物理含义不同):
//        · 响应门失败 (相关峰 < PAD_CAL_RESP_MIN) = 该方向画面没有可相关的内容
//          (抬头看天空), 与"注入是否可信"无关;
//        · 位移门失败 (相关正常但 Σ位移 ≈ 0/符号不符) = 画面不动 = 夹紧/死区,
//          只污染它自己那个方向。
//      据此: 两方向都通过 → 方向组一致性判据; 只有一个方向通过且另一方向**仅因响应门
//      失败** → 采用该方向并在日志写明 (如"仅 +X 向: 另一向无纹理"); 另一方向因位移门
//      或其他任何判据失败 → 整级丢弃 (现状不变 — 夹紧/量程/样本数问题不是"换个方向"
//      能回答的, 放宽只会让带污染的级进来)。
//    - 方向组一致性: 两个方向各自的池化增益 |g₊−g₋| ≤ PAD_CAL_SEG_Z·√(SE₊²+SE₋²),
//      SE = σ/√(Σ|C|²) (由相干统计直接导出的一致性判据) — 方向不该改变响应, 不一致即
//      测量不可信。每方向 2 段 → 先在本方向池化再比, 组内样本翻倍。
//  级数降级: 有效级 ≥2 → 幂律拟合; 恰好 1 级 → 线性回退 (p=1, A = 该级屏速/d,
//    日志标注"单级线性外推, 指数未测"); 0 级 → 失败收尾。
//  延迟自校验: 实测 L 中位数 > P 时停顿留不住瞬态 → 标定失败 (不硬算)。
//
//  采样探针 (给下一级定段长的实测输入, 与"级有效性"是两套判据): 探针只需给出
//    "量级正确的屏速"用于定段长, 故判据刻意宽松 — 显著性达 PAD_CAL_PROBE_SNR·σ̂·√n
//    即可 (不套用级的 MIN_N 样本数、两段一致性、量程判定), 并对结果做量级钳制
//    (0 < v̂ ≤ PAD_GAIN_MAX: 部分偏转的屏速不可能超过满偏转屏速, 对任何 p ≥ 0 的
//    幂律成立)。**为什么必须宽松**: 探针若套用级判据, 级无效 → 探针恒缺 → 段长恒
//    取下限 → 级仍无效, 自我强化; 而探针错向的后果不对称 — 高估 → 段短 → 行程偏小
//    (安全), 低估 → 段长 → 行程偏大 (由级判据丢弃该级)。来源与更新时机: 由 AI 线程
//    逐帧在采样期调用 pad_calib_update_probe — 取最后一段所属的级, 用该级中段样本
//    池化最小二乘; 证据不足 (σ̂ 样本不足/非正增益/超量级) 则保持上次值。每轴开头
//    清零 (两轴灵敏度不同)。
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
constexpr int PAD_CAL_LEVELS_N = 5;      // 级集大小 (两轴一致, 依据见文件头)
inline constexpr float PAD_CAL_LEVELS[PAD_CAL_LEVELS_N] = {0.30f, 0.40f, 0.50f, 0.60f, 0.70f};
// 每级段序 [+d, −d, −d, +d] (**对称段序**, 每方向 2 段 = 2 次独立测量):
//   行程以该级起点为中心 ±A (老的单侧 [+d, −d] 只往一个方向走 → 操作者必须故意
//   把准星放在偏离中心的一侧, 否则撞上俯仰夹紧/屏幕边缘; 对称段序使准星围绕起点
//   往复, 从屏幕中心起即可), 且方向成对 → 同级一致性判据 (方向不该改响应) 有 2 组
//   样本可用, 数据量翻倍。方向序列 +,−,−,+ 使相邻两段方向相反 (换向瞬态被停顿
//   吸收, 与段序无关), 同向两段不相邻 (不连续同向累积偏移)。
constexpr int PAD_CAL_SEGS_PER_LEVEL = 4;      // 每级 ± 各 2 段 (对称段序)
constexpr int PAD_CAL_EXCITE_N = PAD_CAL_AXES_N * PAD_CAL_LEVELS_N * PAD_CAL_SEGS_PER_LEVEL;
constexpr int PAD_CAL_SEGS_N   = 2 * PAD_CAL_EXCITE_N;    // 激励段 + 段后停顿

// 段间零偏转停顿 (ms): L 上界 (用户保证 ≤100ms) + 余量 (≥2 帧周期 @120fps = 16.7ms)
//   = 116.7ms → 上取到 50ms 的整值 150ms。停顿要同时装下 (a) 命令换向的观测瞬态
//   (宽 = 帧长+L ≤ 帧长+100ms), (b) 静止参考样本 (噪声底与 L 边沿检测的来源),
//   余量不足就会把"还在动"的样本当成静止参考。
constexpr float PAD_CAL_L_GUARANTEE_MS = 100.0f;
constexpr int   PAD_CAL_PAUSE_MS = 150;
// 同一停顿的**拍数**表达: 激励由 1kHz 拍驱动, 拟合窗一律按拍算 (1kHz 下与 ms 等价,
//   见 TICK_MS); 换拍率时按拍算的窗仍与激励的边界对齐, 按 ms 算则不会。
constexpr int   PAD_CAL_PAUSE_TICKS = ms_to_ticks(PAD_CAL_PAUSE_MS);

// 段时长上限 (ms): 覆盖"实测采样周期达标称 4 倍"(帧率被高估/丢帧严重) 的中段窗 —
//   该情形下要放满样本下限 + 余量, 需 T ≥ P + 4×帧长×(1+MIN_N+余量)
//   = 150 + 4×8.33×13 ≈ 583ms → 上取 600。上限同时是流程预算的一项。
constexpr int PAD_CAL_SEG_MS_MAX = 600;
// 屏幕高 (px): 采集卡输入格式固定 1920×1080 (NV12 是两种卡唯一同时支持 120fps 的
//   格式), 相位相关位移经 ×2 还原后就是 1080p 屏幕像素 — 行程判据据此按屏高表达。
constexpr int PAD_CAL_SCREEN_H = 1080;
// 单段行程目标 = 屏高的 1/8 = 135px (上下界推导见文件头)
constexpr float PAD_CAL_TRAVEL_PX = PAD_CAL_SCREEN_H / 8.0f;
// 段时长预测用的最陡曲线指数: 响应屏速 ∝ 偏转² 是最陡的常见加速曲线, 按它外推
//   只会**高估**屏速 → 段时长取短 → 行程偏小 (安全方向); 反向低估会超行程冲进
//   夹紧。仅用于定段时长, 拟合出的 p 不受此约束 (它只被测量约束)。
constexpr float PAD_CAL_CURVE_P_MAX = 2.0f;

// 计划名义总时长 (ms): 全部段取上限 = 一轮 (无重播) 的最坏情形 =
//   2 轴 × 5 级 × 4 段 × (600+150)ms = 30s (对称段序使段数翻倍; 用户明确"只标一次,
//   时间不是约束", 故以数据量与可靠性优先)
constexpr int PAD_CAL_PLAN_MS = PAD_CAL_EXCITE_N * (PAD_CAL_SEG_MS_MAX + PAD_CAL_PAUSE_MS);
// 流程预算 (ms): 激励全程 (重播阶梯 + 一次整轮重跑) 的墙钟上界 = 2 × 名义计划。
//   重播与重跑只在"已用 + 本次名义 ≤ 预算"时放行, 故实际时长 ≤ 预算 + 一段在飞段;
//   摇杆账本深度与采样窗深度都按本值导出 (见下面的 static_assert 与 pad_cal_hist_frames)。
constexpr int PAD_CAL_BUDGET_MS = 2 * PAD_CAL_PLAN_MS;

// 级偏转 (满偏比例 → counts)
constexpr int16_t pad_level_defl(float d) {
    return (int16_t)(d * (float)PAD_AXIS_MAX + 0.5f);
}
// 视觉信号与激励同为小幅慢速 (不另立常量: 取级集最低挡, 即最小激励幅度 — 用户实测
//   反馈满偏甩动刺眼), 且与激励同为**以起点为中心的十字** (老的单侧方块会把准星推离
//   操作者放好的屏幕中心; 十字只围绕起点往复, 净位移为 0): 起始十字 (纯视觉开始信号,
//   不采样) / 成功点头 (纵向 ±) / 失败摇头 (横向 ±)
constexpr int16_t PAD_CAL_ANIM_DEFL = pad_level_defl(PAD_CAL_LEVELS[0]);   // 30%
inline const CalibSeg PAD_CAL_START_SEQ[] = {
    {PAD_CAL_ANIM_DEFL,0,ms_to_ticks(240)},{-PAD_CAL_ANIM_DEFL,0,ms_to_ticks(240)},
    {0,PAD_CAL_ANIM_DEFL,ms_to_ticks(240)},{0,-PAD_CAL_ANIM_DEFL,ms_to_ticks(240)},
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

// 账本深度必须覆盖预算全程 (重播/重跑把激励拖长) 加一段在飞段与每样本的延迟/帧长
//   回溯 (L_MAX 200ms + 帧长 ≤100ms), 否则早段的账本查询会落到缓冲首样本 (at() 的
//   钳制) 而静默给出 C=0。
static_assert((long)PAD_LEDGER_TICKS * 1000 / DEFAULT_FREQ
                  >= (long)PAD_CAL_BUDGET_MS + 2L * (PAD_CAL_SEG_MS_MAX + PAD_CAL_PAUSE_MS) + 300L,
              "pad 摇杆账本深度必须覆盖标定预算全程 + 一段在飞段 + 延迟回溯");

// 采样窗深度 (帧, 由采集端按帧率导出): 分级激励不周期, 样本须覆盖预算全程
//   (重播/重跑后, 早段窗口仍是拟合的输入)
constexpr int pad_cal_hist_frames(int cam_fps) {
    return (PAD_CAL_BUDGET_MS + 1000) * cam_fps / 1000;
}

// ---- 判据常量 (依据见文件头) ----
// 中段最少样本: 中段增益是 Σ|C|² 加权的相干最小二乘, 标准误 = σ/√(Σ|C|²) = σ/(√n·|C|)
//   — 对 n 的边际收益按 1/√n 递减, 4→8 把单级增益的随机误差减半 (√2), 8→16 只再减
//   1/√2 的三分之一。取 8 = 在"相干统计下限"上再要一次减半, 代价是段长线性增长。
constexpr int   PAD_CAL_SEG_MIN_N = 8;
// 抖动/丢帧余量 (帧): 中段窗的标称样本数 = MIN_N + 余量 = 12@120fps。采样每帧一次,
//   但处理线程慢于采集/单帧块响应不足时该帧不出样本 (实测 630/630 说明采样率本身
//   正常, 而设计点上窗长只比 4 帧多 0.04 帧 = 零余量, 一次边界抖动就 <4 → 复现
//   实机的"每级中段样本不足")。余量 4 帧 = 在标称 12 个样本里容许丢 4 帧 (33%),
//   同时吸收"实测采样周期高于标称 (cam_fps 由 -f 给出而处理更慢)"的那一档偏差。
constexpr int   PAD_CAL_SEG_MARGIN_FRAMES = 4;
// 噪声底 σ̂ 的样本数下限: σ̂ = 1.4826×中位|·| (稳健尺度), 中位估计的标准误差
//   ≈ 1.25/√n, 要相对误差 ≤30% 需 n ≥ 18 → 取 20。探针的 σ̂ 与级判据同源同规矩
//   (同一个下限), 故探针自第 2 个停顿 (≈12 个尾部样本) 起可用, 自第 3 级起生效。
constexpr int   PAD_CAL_SIGMA_MIN_N = 20;
constexpr float PAD_CAL_MIN_SNR   = 5.0f;   // 激励充分性门限 (噪声底 σ 倍数)
constexpr float PAD_CAL_EDGE_SNR  = 3.0f;   // 停顿边沿检测门限 (σ 倍数)
constexpr float PAD_CAL_SEG_Z     = 3.0f;   // 同级两段一致性 (合并标准误倍数)
constexpr float PAD_CAL_RESP_MIN  = 0.05f;  // 中段响应下限 (相关峰中位数)
constexpr float PAD_CAL_PAUSE_TAIL = 2.0f / 3.0f;   // 停顿的静止参考窗起点 (×P)
// 探针显著性门限 (σ̂ 倍数): 只做"该级中段位移为正且非噪声"的池化符号检验, 不判级 —
//   2σ 池化门在 n ≥ 8 时虚警率 <5%, 且探针错向的后果不对称 (高估安全/低估由级判据
//   丢弃), 故取比级判据 (5σ) 低一档的 2σ。
constexpr float PAD_CAL_PROBE_SNR = 2.0f;
// 死区/响应曲线诊断的报告门限 (仅日志): 以该轴**最高有效挡位**为锚点作 p=1 直线,
//   低挡位实测低于该直线的一半 → 该挡位比线性外推低一倍以上 (幂律族里对应
//   比"直线"更陡的 p ≥ 1.5 一档), 疑似游戏摇杆死区/响应曲线。不参与任何判定。
constexpr float PAD_CAL_DEADZONE_FRAC = 0.5f;

// 可靠每帧位移上界 (全分辨率 px): 半分辨率块宽 B 的 1/3、报告位移 ×2 还原 —
//   圆相关的回卷边界是 B/2, 峰在逼近它之前已与旁瓣混叠, 可靠上界取 B/3。
inline constexpr float pad_cal_shift_max_px(int bs_half) {
    return 2.0f * (float)bs_half / 3.0f;
}

// 中段统计下限 (ms): 中段窗长 W = T−P−帧长 至少要放得下 PAD_CAL_SEG_MIN_N 个样本
//   再加 PAD_CAL_SEG_MARGIN_FRAMES 帧抖动余量 → T ≥ P + 帧长×(1+MIN_N+余量)。
//   @120fps = 258ms (标称 12 个样本), @60fps = 367ms (同样 12 个)。下限大于上限时取
//   上限 (上限是预算的硬约束), 该情形只可能出现在 -f 配置远低于 60fps 时。
inline int pad_cal_seg_ms_min(int cam_fps) {
    double frame = 1000.0 / (double)cam_fps;
    int ms = PAD_CAL_PAUSE_MS
           + (int)(frame * (1 + PAD_CAL_SEG_MIN_N + PAD_CAL_SEG_MARGIN_FRAMES) + 0.5);
    return ms > PAD_CAL_SEG_MS_MAX ? PAD_CAL_SEG_MS_MAX : ms;
}

// 本轮的下限 (ms): 首轮 = 统计下限; 整轮重跑 (rerun=true) = 抬高一档
//   min(2×下限, 上限) — 重跑把"每级首段取统计下限"整体抬到阶梯的下一级。
inline int pad_cal_seg_floor_ms(int cam_fps, bool rerun) {
    int lo = pad_cal_seg_ms_min(cam_fps);
    if (!rerun) return lo;
    int hi = 2 * lo;
    return hi > PAD_CAL_SEG_MS_MAX ? PAD_CAL_SEG_MS_MAX : hi;
}

// 自适应段时长 (拍): 等行程 — T = 行程目标/V̂(下一级), 夹在 [floor_ms, PAD_CAL_SEG_MS_MAX]。
//   V̂(d_next) = v_meas·(d_next/d_meas)^PAD_CAL_CURVE_P_MAX (按最陡曲线高估屏速 =
//   安全方向); 尚无实测 (探针级) 取 floor_ms。floor_ms 由调用方给 (首轮/重跑不同,
//   见 pad_cal_seg_floor_ms)。纯函数: 单测直接断言 (io/pad_test.cu [10])。
int pad_cal_seg_ticks(float v_meas_px_s, float d_meas, float d_next, int floor_ms);

// ---- 自适应段时长的实测输入 (AI 线程写, 状态机读) ----
// "已测级的屏速": 状态机在每级开头读它给该级定段时长。证据不足时保持上次值;
//   每轮激励开始时由状态机清零 (探针级 → 段时长取统计下限)。
extern std::atomic<float> g_pad_probe_px_s;   // px/s (0 = 尚无测量)
extern std::atomic<float> g_pad_probe_d;      // 该测量所在的满偏比例

// 级采样状态 (AI 线程逐帧更新, 状态机在级边界读): 该 (轴,级) 的**已完成激励段**里
//   中段窗样本数的最小值 (无已完成段 = PAD_CAL_MID_N_NONE) — 状态机据此判"取样不足"
//   并决定重播该级; 级重播与整轮重开时由状态机清回 NONE。
constexpr int PAD_CAL_MID_N_NONE = 1 << 30;
extern std::atomic<int> g_pad_mid_n[PAD_CAL_AXES_N][PAD_CAL_LEVELS_N];

// 上一级的实测行程 (px): 该级各激励段在命令区间 [t0, t1+一拍] 内的位移积分 (沿激励
//   方向取绝对值后取平均) — 含观测模型的 L 滞后, 即"准星真正走过的距离"。状态机在级
//   边界读它做段长的闭环校正 (见 pad_cal_seg_ticks 的调用点与文件头)。
extern std::atomic<float> g_pad_probe_travel_px;

// pad 灵敏度钳制带: 逐轴带内判定 (带外 = 激励/背景不可信 → 标定按失败收尾,
//   带边垃圾值绝不回写)。hid 侧的带内钳制是既有行为 (S_MIN/S_MAX), 此处只约束 pad。
inline bool pad_calib_accept(float gain) {
    return gain > PAD_GAIN_MIN && gain < PAD_GAIN_MAX;
}

// ---- 激励计划与段窗口 ----
// 计划里的一段 (状态机播放; 采样端按本表归段): level = 级序号;
//   pause = true 时是段后的零偏转停顿 (defl = 0, d = 0)
struct PadPlanSeg { int axis; int level; float d; int16_t defl; int ticks; bool pause; };

// 计划表 = 先 X 后 Y, 每轴逐级, 每级按对称段序 [+d,−d,−d,+d], 每段后接停顿。
//   ticks 为名义段长 (激励段 = 上限, 停顿 = P); 激励段的实际播放时长由状态机按已测
//   增益自适应 (见 pad_cal_seg_ticks), 拟合只依赖段窗口时间戳。
const std::vector<PadPlanSeg>& pad_cal_plan();

// 段窗口 (状态机播放时逐段记录实际墙钟; ai_thread 收到 g_calib_request 后取快照)
struct PadExcSeg {
    int axis = 0, level = 0;
    float d = 0;
    int16_t defl = 0;
    bool pause = false;
    bool begun = false;      // 已开播 (未触碰的槽 t0==t1==默认值, 靠本标志区分)
    std::chrono::steady_clock::time_point t0{}, t1{};
};

// 段窗口表 (mutex 保护): 播放前建表, 逐段写入 t0/t1; 中断即清空。
//   重播/重跑复用同一批槽位 (begin_seg 覆写时间戳) — 拟合只看最后一次尝试的窗口。
struct PadExcPlan {
    void reset();                                   // 开播: 按计划建表, 时间戳待写入
    void begin_seg(int i, std::chrono::steady_clock::time_point t);
    void end_seg(int i, std::chrono::steady_clock::time_point t);
    void clear();
    // 该 (轴,级) 的槽位作废 (重播前调用): 清 begun 与时间戳 — 否则上一轮尝试的窗口
    //   仍被当作"已完成的段", 取样不足的最小值会把重播也拖成不足 (无休止重播);
    //   拟合只看最后一次尝试的窗口, 与"槽位复用"是同一语义。
    void restart_level(int axis, int level);
    std::vector<PadExcSeg> snapshot() const;
    // 最后一个已开播的段序号 (-1 = 尚无)
    int last_begun() const;
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
    bool  why_samples = false; // 原因 = 中段样本不足 (整轮重跑的判据, 见文件头)
    const char* note = "";     // 采纳标注 (如 "仅 +X 向: 另一向无纹理")
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

// 拟合结果的**回执码** (状态机按它分支; ai_thread 与单测共用同一判定):
//   1 = 成功 (结果可用且逐轴落设计带); 2 = 失败; 3 = 失败但丢级**仅因中段样本不足**,
//   且某轴有效级 <2 → 建议整轮重跑 (见文件头: 只有这一类失败是"加长段长/重跑"能救的,
//   显著性/量程/一致性属可信度问题, 重跑不解决)。
inline int pad_cal_done_code(const PadCalibResult& r) {
    if (r.ok && r.gain[0] > 0 && r.gain[1] > 0
        && pad_calib_accept(r.gain[0]) && pad_calib_accept(r.gain[1])) return 1;
    if (r.err[0] == 0) {                       // 有逐级数据, 才谈得上"丢级原因"
        int nbad = 0;
        for (int ax = 0; ax < PAD_CAL_AXES_N; ++ax)
            for (int li = 0; li < PAD_CAL_LEVELS_N; ++li)
                if (!r.lv[ax][li].valid) {
                    if (!r.lv[ax][li].why_samples) return 2;   // 其它不可信原因 → 直接失败
                    ++nbad;
                }
        if (nbad > 0 && (r.nlv[0] < 2 || r.nlv[1] < 2)) return 3;
    }
    return 2;
}

// 拟合: 历史样本 (t/dt/sx/sy/resp/spread) + 段窗口表 + 可靠每帧位移上界
//   (shift_max_px = pad_cal_shift_max_px(cap_w/6), 由采集端按实际块宽给出) → 结果。
//   账本来源 = own_motion_ledger (pad 路由 = 摇杆账本), 由调用方保证模式已设。
PadCalibResult pad_calib_fit(const std::deque<CalibSample>& hist,
                             const std::vector<PadExcSeg>& plan, float shift_max_px);

// 逐帧更新采样状态 (AI 线程在采样期调用):
//   [1] 探针 (g_pad_probe_*): 取最后一段所属的级, 该级中段样本池化最小二乘, 判据见
//       文件头 (宽松 + 量级钳制); 证据不足则不改。
//   [2] 级采样状态 (g_pad_mid_n): 同一级的**已完成**激励段的中段窗样本数最小值。
void pad_calib_update_probe(const std::deque<CalibSample>& hist,
                            const std::vector<PadExcSeg>& plan);

// 死区/响应曲线诊断 (仅日志, 见文件头): 以该轴最高有效挡位为锚点作 p=1 直线,
//   低挡位实测低于 PAD_CAL_DEADZONE_FRAC×直线值 → 判为命中 (取最低的命中挡位)。
struct PadCalDeadzone {
    bool  hit = false;
    int   lo_li = 0, hi_li = 0;      // 最低命中挡位 / 锚点 (最高有效挡位)
    float v_lo = 0, v_line = 0;      // 该挡实测屏速 / 锚点直线的预估值 px/s
};
PadCalDeadzone pad_cal_deadzone(const PadCalibResult& r, int axis);

// 标定拍的一步: btns 为人类逻辑按键位表 (PadBtn), 返回值给出本拍是否处于标定中
//   及其右摇杆激励偏转 — active 时 pad_tick 以 pad_excite 独占右摇杆并跳过律。
struct PadCalibStep { bool active; int16_t dx, dy; };

// 状态机一步 (由 1kHz pad 拍每拍调用一次):
//   触发 = 人类 L3+R3 长按 PAD_CAL_TRIGGER_TICKS 拍, 或热参 padcalib=1 (一次
//   消费即清, 与 g_calib_request 同一 exchange 语义); 接管关闭 (-a n / aim=0)
//   时标定不可达且进行中的标定复位 (激励是程序注入的移动, 与纯透传互斥 —
//   与 hid 标定分支同一纪律)。cam_fps 用于中段统计下限与段时长换算。
//   g_calib_done: 1 = 成功收尾; 2 = 失败收尾; 3 = "仅因中段样本不足而有效级 <2"
//   (AI 线程按拟合结果判定) → 未重跑过且预算允许时整轮重跑, 否则按失败收尾。
//   相位/计时器是本模块私有 static, 与 hid 的状态机 (core/control.cu 的
//   law_tick) 无共享。
PadCalibStep pad_calib_step(uint16_t btns, int cam_fps);
