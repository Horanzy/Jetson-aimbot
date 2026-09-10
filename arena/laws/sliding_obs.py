"""arena/laws/sliding_obs.py — 边界层滑模收敛 + type-2 扰动观测器
(方向矛盾 CUSUM 归零 + 创新方差 SNR 门 + 检测间隙衰减)。鲁棒优先律的第二形态。

算法原理
========
滑模收敛骨架与 sliding.py 相同: 到达相 (|σ|>ε) 开环弹道甩枪 (Lyapunov 到达增益 =
物理 vmax, 延迟免疫), 边界层 ε = vmax·L̂ 内线性收敛, 收敛带宽按完整延迟设计
wn = (90°−PM)·π/180/L̂ (PM=60, 失配带测试选定的最快点 — 实测 PM=55 即 L20 发散、
PM=50 即 L80 发散)。变化在边界层内的第二控制项: sliding.py 的误差积分 → **扰动
观测器**, 及为其配套的三重证据防护。

1) 误差积分 → type-2 扰动观测器 ----------------------------------------------
误差积分是 holding velocity (等效控制 d = 目标速度) 的间接学习器, 有结构性缺陷:
它无法区分"到达段瞬态"(自身拉枪造成 ê 快速衰减, 不该学)与"目标扰动"(目标真实
速度造成的持续拖尾, 该学)。到达段 ∫ê 把拉枪速度学进积分器, 过零后继续原方向推
(经典 windup): step/relock 过冲 ~10px 并在"积分=P 的伪平衡点"停滞数百 ms; 折返后
积分持有旧速度反向推 ~500ms。反过来它学 d 又必须经误差通道 (速率 ∝ Ki·ê), 慢 →
maneuver 换向尾长、accel 稳态拖尾 a/Ki。

改法: 对象动力学 ë = −u + d ⇒ d = ë + u, 逐拍 d_raw = u_sent + (ê[k]−ê[k−1])/h;
代入 ê 装配式 (帧间 fx/fvx 常量, d(s·Δc)/dt = u_sent) 得恒等式 d_raw ≡ fvx +
帧更新跳变 —— 即滤波器速度估计本身 (代数恒等; s 失配下指令项逐位相消, d_raw 对
s_bel 不敏感)。观测器为 type-2 (PI 型) 跟踪器, 把 d̂ 逼向 d_raw (逐轴, 层内):

    e_d  = ig·g_snr·(d_raw − d̂)
    x_i += k2·e_d·h                          (斜率记忆, k2 = ω_o², 限幅 ±vmax/L̂)
    d̂   += (k1·e_d + x_i)·h                  (k1 = 2·ζ_o·ω_o, ζ_o=1, 限幅 ±vmax)

    ω_o = 1/(obs_frac·L̂)  —— 观测器带宽挂标定量 L̂ (证据到达尺度)。

· **为什么 type-2**: 一阶 LP 对 ramp 扰动有固有速度滞后 τ·a, 平衡时
  trail = (v_t − d̂)/Kp = 滞后/Kp, 小 Kp 下比原 a/Ki 更差 (实测 13.7 vs 5.5px)。
  x_i 在 ramp 上零滞后 (x_i → a), trail 退回滤波器自身加速度滞后/Kp (实测 accel
  rmse 3.8px, 优于原 7.86)。且此积分器**无 windup**: 它积分观测残差 (到达段
  d_raw ≡ 0、d̂ ≡ 0 → 残差 ≡ 噪声), 不积分环路误差 ê。
· 静态目标: d_raw ≡ 0 (与指令无关) → 到达段零学习零 windup → step/relock 过冲
  回到 P-only 一阶衰减 (无过冲), 伪平衡停滞消失。
· 折返: fvx 在 ~3-8 帧翻转 (BETA=0.18), d̂ 跟随 ~1/ω_o → 错误方向推力从"积分
  反卷绕 ~500ms"缩到 ~L̂ + 滤波翻转窗。

2) 方向矛盾 CUSUM → 观测器归零 (移植自 ff_pi 的已验证机制, 宿主 v̂ → d̂) ----------
移动目标上的一切大过冲同源: 目标模型破缺 (急停/变向) 后 fvx/d̂ 成为幽灵, 同时朝
旧方向推观测器、并在 ê 装配里掩盖真实误差。检测 = 双向 CUSUM (Page 序贯检测),
只累计与 v̂ 矛盾方向的创新; 告警即该轴 fvx、d̂、x_i 归零 (位置估计 fx/fy 保留) —
环路回到与阶跃响应相同的初始条件。K=0.5σ 漂移 / C=3σ 单帧封顶 / H=9σ 告警, 全为
无量纲 σ 倍数, σ̂ 在线自标定 (创新方差 EMA) → 噪声越大门自动越宽, 无绝对 px 常数。

3) 创新方差 SNR 门 (失配瞬态污染的防护) ---------------------------------------
L 失配下 (L_真 ≠ L̂), 滤波器的在途 counts 窗口错位 ΔL 把"自身运动"注入创新:
innov_污染 ≈ 自身加速度 × ΔL —— 到达段/折返段自身加减速时产生**同号**创新爆发,
fvx 累积瞬态偏置, 经 fvx·(age+L̂) 与观测器双路进入指令 (实测 L80 step 过冲
9.5 → 20px, 观测器带宽怎么调都压不住; 根源是污染与 legit 证据同通道)。
识别器 = σ̂ 自身: 污染爆发时创新方差先飙 (实测 0.5 → 4-14px), 快慢双 EMA 之比
即瞬时 SNR:

    g_snr = 1/(1 + max(0, σ_fast/σ_slow − 1))       (纯无量纲, 自标定)

静默跟踪时 σ_fast ≈ σ_slow → g_snr = 1 (观测器全力, type-2 性质无损); 自身加速
污染时 σ_fast/σ_slow → 6-20 → 吸收与输出同步收紧一个量级。g_snr 同时作用于观测
器吸收与输出 (学习多少权限就行使多少权限), 但不作用 P 项。SNR_RATIO=8 为快慢
时间常数分离比 (无量纲设计选择): 慢 EMA 须横跨数个环路时间常数, 使 ≤2L̂ 的自身
瞬态对其扰动 <10%; 若 legit 目标变向被误收门 → 调大 CUSUM_H 或调小分离比。

4) 检测间隙衰减 + 丢帧 dt 归一化 ------------------------------------------------
检测中断 (drop/丢帧) 时盲推上界 d̂·STALE(200ms) → ~d̂·L̂: gap = 1 − clamp((age −
frame_dt)/L̂, 0, 1), 输出按 L̂ 尺度撤回观测器 (同 ff_pi gap_scale 原理)。
滤波器增益按实测 dt 归一化 (项目不变量 #1, 同 ff_pi 的 r=dt/DT0 约定): 每毫秒
增益恒定, 120fps 无丢帧时与原固定值逐位相同, 60fps/丢帧时行为与 120fps 等价
(旧 sliding 的 BETA/dt 每帧增益随帧率减半, 60fps accel 拖尾加倍 — 旧误差积分的
无限 DC 增益掩盖了这一帧率依赖, 观测器把它暴露出来后按不变量修正)。

参数表 (默认 / 出处)
====================
| 参数          | 默认 | 出处 (原理导出 / 经验)                                           |
|---------------|------|------------------------------------------------------------------|
| pm_deg        | 60   | 原理+失配带测试选定: wn=(90−PM)π/180/L̂ 按完整延迟取裕度;          |
|               |      | 实测 PM=55 → L20 发散、PM=50 → L80 发散, 60 = 全带最快点          |
| brake_factor  | 1.0  | 原理: 制动距离 = 物理制动距离 vmax·L̂ (= 最小稳定边界层), 不晚制动 |
| vmax_frac     | 0.95 | 原理: 留 5% 速度余量防 count 量化截断                            |
| blend_frac    | 0.30 | 经验: 到达/收敛交接混合带 = 0.30·vmax·L̂, 平滑 1.9:1 速度跳变      |
| i_gate_frac   | 0.25 | 经验: 观测器距离门 = 0.25·vmax·L̂ (稳态拖尾消除需要 ig→1 的窗口)   |
| obs_frac      | 2.0  | 设计选择: ω_o = 1/(obs_frac·L̂), 观测器带宽 = 证据到达尺度之半;    |
|               |      | 3-4 → L20/L70/L80 逐档开始劣化 (ig 门+慢观测器正反馈), 2 = 最快过带 |
| zeta_o        | 1.0  | 原理: 观测器临界阻尼 — d̂ 过冲 = 错误方向推力, 正是要消灭的病      |
| SNR_RATIO     | 8    | 经验: 快慢 σ̂ 时间常数分离比 (见上, 校准方向: 误收门→调小)         |
| CUSUM K/C/H   | .5/3/9 | 原理 (σ 倍数, 无量纲): 漂移/单帧封顶/告警, 同 ff_pi 已验证值     |
| ALPHA/BETA    | .40/.18 | 估计器: alpha-beta 位置/速度增益 (结构同 sliding.py, 加 dt 归一) |

arena 成绩 (实测, 默认种子 1,2,3)
=================================
OVERALL=138.10, matched composite=160.87 (step settle 351.3ms / 过冲 3.00px /
首达 353.3ms; const_vel rmse 0.78px; accel rmse 3.82px in_band 20.8%; maneuver
rmse 25.03px), 失配扫描最坏 111.31 (L70), 重新锁定 426.7ms / 过冲 2.75px,
帧率差 1.0%。
宽延迟 {L20..80} = {88.1, 87.4, 90.3, 101.8, 94.2, 108.3, 117.5} — 全档通过,
逐档优于 sliding 基线, 零发散。s 失配 {0.7..1.3} = {85.2, 92.3, 101.8, 107.1,
117.1} — 全档通过。对照 sliding (OVERALL 174.44) 与 ff_pi (156.67)。

已知局限
========
· 首达时间 (3px 首触) 353ms 慢于旧 sliding 的 218ms: 旧值由积分 windup 补贴
  (快触达 → 10.7px 过冲 → 345ms 拖尾才真稳定)。PM=60 全带约束下 P-only 尾
  (衰减率 wn) 是首达的结构下界; 拉枪"真稳定"时间 (settle 351 vs 563ms) 快 38%。
· 收敛带宽按完整延迟 PM=60 设计, 本质保守 → 匹配速度慢于用残差延迟+前馈的
  ff_pi; 换取全带零发散与最平剖面 (本律的存在意义)。
· CV 滤波无法预测加速度; accel/maneuver 的剩余拖尾 = 滤波器加速度滞后/Kp
  (加速度跟踪由观测器 x_i 斜率记忆承担, 已消除 a/Ki 主项)。
· 极端失配 (|L_真−L̂| > ~30ms 或 s 误差 > ~40%) 超出标定应有精度, 靠标定保证;
  本律在这些极端下仍不发散 (实测), 但精度退化。

debug() 字段 (trace 逐拍采集为 dbg_* 列, 无副作用, 评测路径不调用)
==================================================================
  mode      0=到达/混合带 (r > brake_dist), 1=边界层收敛相
  e_hat_x   Smith 装配误差 ê_x (px)
  e_hat_y   Smith 装配误差 ê_y (px)
  r         |ê| (px)
  ig        观测器距离门 i_gate/(i_gate+r) (0..1)
  g_snr     创新方差 SNR 门 (0..1, 静默≈1)
  gap       检测间隙衰减因子 (0..1)
  gx        观测器输出权重 = gap·g_snr
  dhat_x    扰动观测器 x (px/ms, ±vmax)
  dhat_y    扰动观测器 y (px/ms, ±vmax)
  xi_x      观测器斜率积分器 x (px/ms², ±vmax/L̂)
  xi_y      观测器斜率积分器 y (px/ms², ±vmax/L̂)
  fvx       滤波器速度估计 v̂_x (px/ms, d_raw 的真值来源)
  fvy       滤波器速度估计 v̂_y (px/ms)
  cs_x      CUSUM 状态 x (σ 单位)
  cs_y      CUSUM 状态 y (σ 单位)
  sig_x     快 σ̂ 创新 std x (px)
  sig_y     快 σ̂ 创新 std y (px)
  u_x       层内指令速度 x (px/ms, 限幅后)
  u_y       层内指令速度 y (px/ms, 限幅后)
  age       距最近新帧的时龄 (ms)
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
from arena.core import Observation, LawConfig
from arena.laws.base import CountsHist, Law, register


@register("sliding_obs")
class SlidingObsLaw(Law):
    ALPHA = 0.40
    BETA = 0.18
    JUMP_GATE = 100.0
    STALE = 200.0
    DT0 = 1000.0 / 120.0
    # CUSUM 参数 (σ 倍数, 无量纲, 同 ff_pi 已验证值): K 漂移 / C 单帧封顶 / H 告警
    CUSUM_K = 0.5
    CUSUM_C = 3.0
    CUSUM_H = 9.0
    # 快慢 σ̂ 时间常数分离比 (无量纲设计选择, 见模块 docstring §3)
    SNR_RATIO = 8.0

    def __init__(self, pm_deg=60.0, brake_factor=1.0, vmax_frac=0.95,
                 blend_frac=0.30, i_gate_frac=0.25, obs_frac=2.0, zeta_o=1.0):
        self._pm_deg = pm_deg
        self._brake_factor = brake_factor
        self._vmax_frac = vmax_frac
        self._blend_frac = blend_frac
        self._i_gate_frac = i_gate_frac
        self._obs_frac = obs_frac
        self._zeta_o = zeta_o

    def reset(self, cfg: LawConfig):
        self.cfg = cfg
        self.vmax = cfg.max_v * self._vmax_frac
        L = max(1.0, cfg.L)
        self.wn = (90.0 - self._pm_deg) * math.pi / 180.0 / L
        self.kp = self.wn
        self.omega_o = 1.0 / (self._obs_frac * L)
        self.k1o = 2.0 * self._zeta_o * self.omega_o
        self.k2o = self.omega_o * self.omega_o
        self.xi_lim = self.vmax / L
        self.brake_dist = self.vmax * L * self._brake_factor
        self.boundary_layer = max(1.0, self._blend_frac * self.brake_dist)
        self.i_gate = self._i_gate_frac * self.brake_dist
        self.ch = CountsHist()
        self.filt = False
        self.fx = self.fy = self.fvx = self.fvy = 0.0
        self.prev_det_t = None
        self.t_pub = -1e9
        self.dhx = self.dhy = 0.0
        self.xix = self.xiy = 0.0
        self.sig2x = self.sig2y = 1.0
        self.sig2sx = self.sig2sy = 1.0
        self.csx = self.csy = 0.0
        self.rem_x = self.rem_y = 0.0
        self._mode = 0.0
        self._ex = self._ey = 0.0
        self._r = 0.0
        self._ig = 0.0
        self._g_snr = 1.0
        self._gap = 1.0
        self._gx = 1.0
        self._ux = self._uy = 0.0
        self._age = 0.0

    def _update_filter(self, det: Observation):
        cfg = self.cfg
        if self.prev_det_t is None:
            self.fx, self.fy = det.dx, det.dy
            self.fvx = self.fvy = 0.0
            self.sig2x = self.sig2y = 1.0
            self.sig2sx = self.sig2sy = 1.0
            self.csx = self.csy = 0.0
            self.filt = True
            self.prev_det_t = det.t
            self.t_pub = det.t
            return
        dt = det.t - self.prev_det_t
        dt = max(1.0, min(100.0, dt))
        # 增益按实测 dt 归一化 (项目不变量 #1, 同 ff_pi 的 r=dt/DT0 约定):
        # 每毫秒增益恒定, 120fps 无丢帧时 alpha/beta_s 与原固定值逐位相同,
        # 60fps/丢帧时每帧增益随之放大, 行为与 120fps 等价
        r_dt = dt / self.DT0
        alpha = min(0.90, self.ALPHA * r_dt)
        beta_s = min(0.60, self.BETA * r_dt)
        c0 = self.ch.at(det.t - cfg.L - dt)
        c1 = self.ch.at(det.t - cfg.L)
        cax = c1[0] - c0[0]
        cay = c1[1] - c0[1]
        px_pred = self.fx + self.fvx * dt - cfg.s * cax
        py_pred = self.fy + self.fvy * dt - cfg.s * cay
        inx = det.dx - px_pred
        iny = det.dy - py_pred
        if math.hypot(inx, iny) > self.JUMP_GATE:
            self.fx, self.fy = det.dx, det.dy
            self.fvx = self.fvy = 0.0
            self.csx = self.csy = 0.0
            # 跳变 = 目标模型破缺: 速度态 (滤波器 + 观测器) 一并归零, 位置保留
            self.dhx = self.dhy = 0.0
            self.xix = self.xiy = 0.0
        else:
            # σ̂ 在线自标定: 快 EMA 跟踪当前创新方差, 慢 EMA 维持静默参考
            b = beta_s / dt
            self.sig2x += b * (inx * inx - self.sig2x)
            self.sig2y += b * (iny * iny - self.sig2y)
            bs = b / self.SNR_RATIO
            self.sig2sx += bs * (inx * inx - self.sig2sx)
            self.sig2sy += bs * (iny * iny - self.sig2sy)
            sx = max(math.sqrt(self.sig2x), 1e-6)
            sy = max(math.sqrt(self.sig2y), 1e-6)
            # 双向 CUSUM, 只累计与 v̂ 矛盾方向的创新 (σ 归一)
            if self.fvx > 0:
                accx = -inx / sx
            elif self.fvx < 0:
                accx = inx / sx
            else:
                accx = -self.CUSUM_K
            if self.fvy > 0:
                accy = -iny / sy
            elif self.fvy < 0:
                accy = iny / sy
            else:
                accy = -self.CUSUM_K
            self.csx = max(0.0, self.csx + min(max(accx, 0.0), self.CUSUM_C)
                           - self.CUSUM_K)
            self.csy = max(0.0, self.csy + min(max(accy, 0.0), self.CUSUM_C)
                           - self.CUSUM_K)
            # 告警 → 该轴速度态归零 (滤波器 v̂ 与观测器 d̂/x_i; 位置 fx/fy 保留)
            if self.csx >= self.CUSUM_H and self.fvx != 0.0:
                self.fvx = 0.0
                self.dhx = 0.0
                self.xix = 0.0
                self.csx = 0.0
            if self.csy >= self.CUSUM_H and self.fvy != 0.0:
                self.fvy = 0.0
                self.dhy = 0.0
                self.xiy = 0.0
                self.csy = 0.0
            self.fx = px_pred + alpha * inx
            self.fy = py_pred + alpha * iny
            self.fvx += (beta_s / dt) * inx
            self.fvy += (beta_s / dt) * iny
        self.prev_det_t = det.t
        self.t_pub = det.t

    def step(self, t: float, obs: Optional[Observation]) -> Tuple[int, int]:
        cfg = self.cfg
        if obs is not None and obs.new:
            self._update_filter(obs)

        cx = cy = 0
        age = t - self.t_pub
        self._age = age
        if self.filt and age < self.STALE:
            frame_dt = cfg.frame_dt if cfg.frame_dt > 0 else self.DT0
            gap = 1.0 - max(0.0, min(1.0, (age - frame_dt) / max(1.0, cfg.L)))
            self._gap = gap
            cp = self.ch.at(self.t_pub - cfg.L)
            cn = self.ch.cum()
            ex = self.fx + self.fvx * (age + cfg.L) - cfg.s * (cn[0] - cp[0])
            ey = self.fy + self.fvy * (age + cfg.L) - cfg.s * (cn[1] - cp[1])
            r = math.hypot(ex, ey)
            self._ex, self._ey, self._r = ex, ey, r

            if r > self.brake_dist:
                self._mode = 0.0
                v_inner = self.wn * self.brake_dist
                if r > self.brake_dist + self.boundary_layer:
                    speed = self.vmax
                else:
                    blend = (r - self.brake_dist) / self.boundary_layer
                    speed = v_inner + blend * (self.vmax - v_inner)
                scale = speed / r
                vcx = ex * scale
                vcy = ey * scale
                # 到达相: 观测器冻结 (d_raw 对静态目标恒 0, 到达段零学习零
                # windup; 冻结保持"到达=开环弹道"身份)
            else:
                self._mode = 1.0
                ig = self.i_gate / (self.i_gate + r)
                sf = 0.5 * (math.sqrt(self.sig2x) + math.sqrt(self.sig2y))
                ss = 0.5 * (math.sqrt(self.sig2sx) + math.sqrt(self.sig2sy))
                g_snr = 1.0 / (1.0 + max(0.0, sf / max(ss, 1e-6) - 1.0))
                self._ig = ig
                self._g_snr = g_snr
                for axis in (0, 1):
                    fv = self.fvx if axis == 0 else self.fvy
                    dh = self.dhx if axis == 0 else self.dhy
                    xi = self.xix if axis == 0 else self.xiy
                    e_d = ig * g_snr * (fv - dh)
                    xi_new = xi + self.k2o * e_d * cfg.h
                    xi_new = max(-self.xi_lim, min(self.xi_lim, xi_new))
                    dh_new = dh + (self.k1o * e_d + xi_new) * cfg.h
                    dh_cl = max(-self.vmax, min(self.vmax, dh_new))
                    # 观测器抗饱和: d̂ 顶限幅且仍被同向推时冻结 x_i
                    if dh_cl != dh_new and (dh_new - dh) * xi_new > 0:
                        xi_new = xi
                    xi, dh = xi_new, dh_cl
                    if axis == 0:
                        self.xix, self.dhx = xi, dh
                    else:
                        self.xiy, self.dhy = xi, dh
                self._gx = gap * g_snr
                vcx = max(-self.vmax, min(self.vmax, self.kp * ex + self._gx * self.dhx))
                vcy = max(-self.vmax, min(self.vmax, self.kp * ey + self._gx * self.dhy))
                self._ux, self._uy = vcx, vcy

            s = max(0.05, min(20.0, cfg.s))
            self.rem_x += vcx * cfg.h / s
            self.rem_y += vcy * cfg.h / s
            cx, cy, self.rem_x, self.rem_y = self._counts(
                self.rem_x, self.rem_y, s, cfg.count_limit)
        else:
            self.rem_x = self.rem_y = 0.0
            self.dhx = self.dhy = 0.0
            self.xix = self.xiy = 0.0

        self.ch.add(t, cx, cy)
        return cx, cy

    def debug(self) -> dict:
        return {
            "mode": self._mode,
            "e_hat_x": self._ex,
            "e_hat_y": self._ey,
            "r": self._r,
            "ig": self._ig,
            "g_snr": self._g_snr,
            "gap": self._gap,
            "gx": self._gx,
            "dhat_x": self.dhx,
            "dhat_y": self.dhy,
            "xi_x": self.xix,
            "xi_y": self.xiy,
            "fvx": self.fvx,
            "fvy": self.fvy,
            "cs_x": self.csx,
            "cs_y": self.csy,
            "sig_x": math.sqrt(self.sig2x),
            "sig_y": math.sqrt(self.sig2y),
            "u_x": self._ux,
            "u_y": self._uy,
            "age": self._age,
        }
