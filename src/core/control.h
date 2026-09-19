// ============================================================================
//  control.h — ff_pi_acc 控制律头常数区 (结构参数, 编译期, 无手调参数):
//    收敛带宽 wn 由标定延迟 L 自动导出 (wn=(90°−PM)π/180/L), 阻尼比/前馈增益/
//    CUSUM 与 â 通道参数均给出物理出处; 触发键位与接管保持窗。各常量的推导
//    与调整指引见 AGENTS.md "Tuning" 与 arena/laws/ff_pi_acc.py; 控制律按拍
//    执行 (control_apply / control_apply_pad 两执行后端共用同一 law_tick,
//    拍率 = DEFAULT_FREQ) 在 control.cu。
// ============================================================================

#pragma once

#include <cstdint>

// ========================= 跟踪滤波器 (Smith 预测器, dt 归一) =========================
const float PRED_ALPHA0   = 0.50f;                   // 位置增益 @120fps; 实际 α=ALPHA0·dt/DT0 (帧率无关)
const float PRED_BETA0    = 0.03f;                   // 速度增益 @120fps; 实际 β=BETA0·dt/DT0 (0.03 = 失配带边缘余量)
const float PRED_L_COMP   = 1.10f;                   // Smith 过补偿系数 (>1 帮欠补偿侧 L真>L̂, 危险方向)
const float PRED_DT0      = 1000.0f / 120.0f;        // 增益归一参考帧周期
const float TRACK_JUMP_GATE = 100.0f;                // 创新超此值 → 重置滤波器
const float TARGET_STALE_MS = 200.0f;                // 目标超时 → 暂停自瞄

// ========================= 控制律: 极点配置 PI + type-2 速度前馈 (ff_pi) =========================
//  带宽由标定延迟 L 导出 (免手调); 前馈补跟踪速度; 方向矛盾 CUSUM 告警时
//  该轴 v̂ 归零重拉 (变向/急停复用阶跃响应)。详见 arena/laws/ff_pi.py 与 AGENTS.md。
const float FF_PM_DEG   = 50.0f;                     // 相位裕度: wn=(90°−PM)π/180/L (失配带 L20–80 全过的最快设计点)
const float FF_ZETA     = 1.0f;                      // 收敛阻尼比 (临界阻尼, 无过冲)
const float FF_GAIN_VAL = 1.0f;                      // 速度前馈增益: =1 是匀速目标零拖尾的精确开环指令
const float FF_I_GATE   = 8.0f;                      // 收敛区门控 / I 距离衰减尺度 (px)
const float FF_I_FRAC   = 1.0f;                      // 积分限幅 = I_FRAC×max_v/Ki
// CUSUM 参数 (均为 σ 倍数, 无量纲 — Page 序贯变化检测): K 漂移 0.5σ,
// C 单帧增量上限 3σ (拒后坐力式单帧踢脚), H 告警 9σ (同号持续 ~2 帧触发)
const float CUSUM_K = 0.5f;
const float CUSUM_C = 3.0f;
const float CUSUM_H = 9.0f;
// â 加速度偏差补偿通道 (创新均值反演, 修 α-β 对匀加速目标的结构滞后; 依据详见
//   arena/laws/ff_pi_acc.py — 无加速时 â≡0, 指令流与纯 ff_pi 逐位一致):
const float ACC_SIG_CLIP_K = 2.0f;   // 清洗创新 Huber 截断 (σ̂_r 倍数, M-estimator 标准断点)
const float ACC_TAU_L      = 4.0f;   // ȳ EMA 记忆 = ACC_TAU_L·L̂ (须 >> CUSUM 告警延迟, << 加速段时长)
const float ACC_RB_HOLD_N  = 1.5f;   // 重建旗标最短保持 = N·(T/β) (α-β 速度估计自身时间常数)
const float ACC_OW_ACTIV_K = 2.0f;   // 自身活动门限 θ_a = max_v/(K·L̂) (K 个延迟窗走完速度帽算剧烈)
const float ACC_SNR        = 10.0f;  // â 显著性地板倍数 (盖过失配带相关 dither 噪声驱动的均值游走)

// ========================= 触发键位 (HID 按钮位掩码) =========================
const uint16_t LEFT_KEY  = 0x01;
const uint16_t RIGHT_KEY = 0x02;
const uint16_t SIDE_KEY  = 0x10;                     // 侧键2: 抑制自瞄
const uint16_t SIDE_KEY2 = 0x08;                     // 侧键1
const uint16_t BOTH_SIDE_KEYS = SIDE_KEY | SIDE_KEY2; // 双侧键: 标定触发

const int KEEP_ALIVE_MS = 200;                       // 松开触发键后保持自瞄的时间

// 控制拍 (周期 TICK_MS): 把自瞄指令 (counts) 与标定激励序列合成进 HID 报文的位移字节;
//  纯透传 (-a n / 热参 aim=0) 时不注入。cam_fps 用于丢帧期前馈衰减的时间尺度,
//  rpt 为 HID_REPORT_LEN 字节报文缓冲 (只改写位移字节)。
void control_apply(int cam_fps, uint8_t* rpt, int16_t real_x, int16_t real_y);

// pad 模式的律入口 (与 control_apply 同一 law_tick, 律数学逐句一致): btns 为
//   触发键位字 (fire→LEFT_KEY, ads→RIGHT_KEY, 由 pad 侧 RT/LT 门控生成 — 侧键
//   抑制与双侧键标定是 hid 键位语义, pad 键位字恒无那些位, 分支自然惰性)。
//   输出期望速度 (px/ms) 与注入门 (接管开 × 触发保持窗内); 速度帽逐轴取
//   min(热参 x, 该轴满偏转屏速) — 注入打到该轴满偏即该轴行程上限 (hid 两轴同为
//   热参 x, 算式与单帽逐位相同)。量化/报文/counts 尾巴是 hid 专属, pad 路径不含。
bool control_apply_pad(int cam_fps, uint16_t btns, float& out_vx, float& out_vy);
