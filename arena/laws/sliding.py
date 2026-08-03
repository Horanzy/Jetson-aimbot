"""arena/laws/sliding.py — 边界层滑模收敛 + 延迟免疫弹道甩枪 (鲁棒优先)。

算法原理
========
滑模控制 (Sliding-Mode Control) 对**有界扰动**最鲁棒: 把状态驱向滑动面 σ=0, 到达增益
只要压过扰动上界, 扰动只改变到达时间、不改变最终精度。本律针对"积分器 + 观测延迟"植物
设计, 目标是在宽延迟/灵敏度失配下**不发散、剖面平**, 为此**主动牺牲匹配速度换鲁棒性**。

植物 (积分器 + 延迟)
--------------------
准星对 counts 积分: crosshair += s·counts。把指令看成准星速度 u (px/ms), 误差
e = 目标 − 准星 的动力学一阶:

    ė = −u + d,    d = 目标速度 (有界扰动, |d| ≤ D)

唯一延迟在观测侧 (帧反映 t−L_真 的世界); 控制 500Hz, 观测 120fps。

滑动面与到达律 (全部参数由原理导出)
----------------------------------
取 Smith 预测/延迟补偿后的误差 ê 为滑动面: σ = ê (逐轴)。控制律分两相:

  1. 到达相 (|σ| > ε): u = vmax·sign(σ) —— 开环全速甩枪 (弹道段)。
     **Lyapunov 到达条件**: 取 V = ½σ², V̇ = σ(−u+d) = −vmax|σ| + σd
     ≤ −|σ|(vmax − |d|)。只要到达增益 vmax > 扰动上界 D (目标最大速度), V̇<0,
     面在有限时间内到达。这里**到达增益就是物理速度上限 vmax** (不是调出来的数),
     对任何可达目标 (D < vmax) 天然满足, 裕度 = vmax/D。开环 → 对延迟完全免疫。

  2. 收敛相 (|σ| ≤ ε): u = Kp·σ + Ki·∫σ —— 边界层内线性收敛。
     · **收敛带宽 Kp = wn = (90°−PM)·π/180 / L̂**, PM=60° → wn = 0.5236/L̂
       (L̂=50 时 0.01047 rad/ms)。这是积分器+**完整延迟**环路的相位裕度线:
       开环穿越频率 ωc=Kp, PM = 90° − ωc·L̂·180/π。取 PM=60 即 Kp=0.5236/L̂。
       故意按**完整 L̂** (而非 Smith 补偿后的残差) 设计 → 不依赖延迟精确对消,
       失配时仍留相位裕度。这是本律鲁棒 (而非快) 的根本, 也是它比激进律慢的来源。
     · **Ki = Kp²/(4ζ²), ζ=0.5 → Ki = wn²**: 收敛环 PI 的闭环特征方程
       s² + Kp·s + Ki = 0, 极点配置取 ζ=0.5 ( quarter-amplitude 阻尼, 响应快)。
       ζ=0.5 把积分转折频率 Ki/Kp = 2ζ²·Kp = Kp = wn 放在穿越频率处 —— 这是不
       额外侵蚀 PM=60 相位裕度的最大积分强度 (再大转折频率上穿、PM 塌陷)。积分提供
       滑模"等效控制"累积出 d, 保证匀速目标零稳态拖尾; 带距离衰减 + 条件抗饱和。
       匀加速目标的稳态滞后 ≈ a/Ki = a/wn², 是 PM=60 保守增益下的物理下界。

**边界层厚度 ε = vmax·L̂ (brake_factor=1.0)** —— 一个量, 两重物理含义:
  (a) 制动距离: 全速 vmax 下, 延迟 L̂ 内准星还要走 vmax·L̂ 才能开始反应, 故必须
      在 |ê|≈vmax·L̂ 处制动。早于此 (brake_factor<1) 是"晚制动抢速度", 失配过冲大。
  (b) 最小稳定边界层: 观测陈旧 L̂, 比 vmax·L̂ 更薄的边界层无法被陈旧反馈稳定
      (会抖振)。ε=vmax·L̂ 是延迟物理决定的最小可稳定层, 同时即制动距离。
  到达相/收敛相在 σ=ε 处的速度: 收敛给 wn·ε = 0.5236·vmax, 到达给 vmax, 有 ~1.9:1
  跳变 → 用混合带 boundary_layer 平滑 (唯一经验旋钮, 见参数表)。

为什么鲁棒 (零发散、剖面平)
--------------------------
1. 收敛增益按**完整延迟** PM=60 兜底: 闭环带宽 ≤ 0.5236/L̂, 对 ±40% 延迟失配留足
   相位裕度, 静态目标在任何 L_真 都 settle —— 零发散的根本。
2. 弹道到达相开环、延迟免疫: 大误差不暴露反馈, 失配只影响制动确认时刻, 不失稳。
3. 制动取物理距离 vmax·L̂ (不晚制动): 失配最坏项 (L_真>L̂ 制动偏晚) 的过冲被提前
   制动 + 宽混合带抹平。
4. 不用速度前馈、不做速度过预测: 前馈/过预测把滤波器偏差直通进指令, 失配下发散。
   纯 Smith (pred_factor=1.0) + 保守增益 → 鲁棒性来自结构而非调参。

参数表 (默认 / 出处)
====================
| 参数          | 默认  | 出处 (原理导出 / 经验)                                          |
|---------------|-------|------------------------------------------------------------------|
| pm_deg        | 60    | 原理: 期望相位裕度。wn=(90−PM)π/180/L̂, 按完整延迟取 PM=60 留失配裕度 |
| brake_factor  | 1.0   | 原理: 制动距离=物理制动距离 vmax·L̂ (=最小稳定边界层)。不晚制动     |
| zeta          | 0.5   | 原理: 收敛 PI 极点配置阻尼比。ζ=0.5 → Ki=wn², 积分转折=wn (PM 上限) |
| vmax_frac     | 0.95  | 原理: 留 5% 速度余量防 count 量化截断                            |
| blend_frac    | 0.30  | 经验: 到达/收敛交接混合带 = 0.30·vmax·L̂, 平滑 1.9:1 速度跳变      |
| i_gate_frac   | 0.25  | 经验: 积分距离衰减门限 = 0.25·vmax·L̂ (边界处衰减到 20%, 防暂态饱和)|
| ALPHA/BETA    | .40/.18| 估计器: alpha-beta 位置/速度增益 (结构同 reference.py)           |

自由旋钮仅 2 个经验量 (blend_frac, i_gate_frac), 均为交接/抗饱和平滑系数, 不影响稳定
裕度; 其余全部由 PM/制动距离/极点配置/物理上限导出。

arena 成绩 (实测)
=================
OVERALL=174.44 (对照参考律 209.66)。
匹配 composite=213.51 (step settle ~563ms / 过冲 ~10.6px / 首达 ~218ms; const_vel
rmse 0.86px; accel rmse 7.86px = a/Ki 物理下界; maneuver rmse 27.4px ≈ 延迟下界 v·L)。
**鲁棒性 (本律最大优势, 实测最平剖面 + 全程零发散)**:
· 延迟失配 {L30..70} = {113,113,124,110,124}, 最坏 123.92, 极差 14 (激进律极差 40+)。
· 宽延迟扫描 {L20..80} = {104,112,116,121,112,123,120}: **L=20 与 L=80 两端均不发散**。
  发散边界: 无 (全 20-80 稳定)。
· 灵敏度失配 {s 0.7..1.3} = {94,109,121,133,143}: 全程零发散。
· 重新锁定: 二次稳定 619ms, 过冲 9.95px, 零发散。
· 帧率差 2.9% (60 vs 120fps)。
速度换鲁棒的取舍: 匹配分慢于 ff_pi/ballistic, 但失配/宽延迟/灵敏度失配下零发散且剖面最平。

已知局限
========
· 收敛增益按完整延迟 PM=60 设计, 本质保守 → 匹配速度 (settle/首达) 与加速跟踪
  (accel rmse=a/Ki) 慢于用残差延迟+前馈的律 (如 ff_pi)。这是"最鲁棒"的代价, 非缺陷。
· CV 滤波无法预测加速度; 不做速度过预测 (保失配鲁棒), 故 maneuver rmse 略高于
  用过预测的律 (~27 vs ~23), 但失配下不过预测才不发散。
· 到达相速度=vmax (Lyapunov 要求 >D), 收敛相在 ε 处仅 0.5236·vmax, 交接需混合带;
  blend_frac/i_gate_frac 是两个经验平滑量 (不影响稳定裕度)。
· 极端失配 (|L_真−L̂|>~30ms 或 s 误差>~40%) 超出标定应有精度, 靠标定保证; 本律在
  这些极端下仍不发散 (实测), 但精度退化。
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
from arena.core import Observation, LawConfig
from arena.laws.base import Law, register


class _CountsHist:
    def __init__(self):
        self.t: list[float] = []
        self.cx: list[float] = []
        self.cy: list[float] = []
        self.cumx = 0.0
        self.camy = 0.0

    def add(self, t, dx, dy):
        self.cumx += dx
        self.camy += dy
        self.t.append(t)
        self.cx.append(self.cumx)
        self.cy.append(self.camy)
        if len(self.t) > 2000:
            self.t.pop(0); self.cx.pop(0); self.cy.pop(0)

    def at(self, t):
        ts = self.t
        n = len(ts)
        if n == 0:
            return 0.0, 0.0
        if t <= ts[0]:
            return self.cx[0], self.cy[0]
        if t >= ts[-1]:
            return self.cx[-1], self.cy[-1]
        lo, hi = 0, n - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ts[mid] <= t:
                lo = mid
            else:
                hi = mid
        span = ts[hi] - ts[lo]
        f = (t - ts[lo]) / span if span > 0 else 0.0
        return (self.cx[lo] + (self.cx[hi] - self.cx[lo]) * f,
                self.cy[lo] + (self.cy[hi] - self.cy[lo]) * f)

    def cum(self):
        return self.cumx, self.camy


@register("sliding")
class SlidingLaw(Law):
    ALPHA = 0.40
    BETA = 0.18
    JUMP_GATE = 100.0
    STALE = 200.0

    def __init__(self, pm_deg=60.0, zeta=0.5, brake_factor=1.0, vmax_frac=0.95,
                 blend_frac=0.30, i_gate_frac=0.25):
        self._pm_deg = pm_deg
        self._zeta = zeta
        self._brake_factor = brake_factor
        self._vmax_frac = vmax_frac
        self._blend_frac = blend_frac
        self._i_gate_frac = i_gate_frac

    def reset(self, cfg: LawConfig):
        self.cfg = cfg
        self.vmax = cfg.max_v * self._vmax_frac
        L = max(1.0, cfg.L)
        self.wn = (90.0 - self._pm_deg) * math.pi / 180.0 / L
        self.kp = self.wn
        self.ki = self.kp * self.kp / (4.0 * self._zeta * self._zeta)
        self.brake_dist = self.vmax * L * self._brake_factor
        self.boundary_layer = max(1.0, self._blend_frac * self.brake_dist)
        self.i_gate = self._i_gate_frac * self.brake_dist
        self.ch = _CountsHist()
        self.filt = False
        self.fx = self.fy = self.fvx = self.fvy = 0.0
        self.prev_det_t = None
        self.t_pub = -1e9
        self.int_x = self.int_y = 0.0
        self.rem_x = self.rem_y = 0.0

    def _update_filter(self, det: Observation):
        cfg = self.cfg
        if self.prev_det_t is None:
            self.fx, self.fy = det.dx, det.dy
            self.fvx = self.fvy = 0.0
            self.filt = True
            self.prev_det_t = det.t
            self.t_pub = det.t
            return
        dt = det.t - self.prev_det_t
        dt = max(1.0, min(100.0, dt))
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
        else:
            self.fx = px_pred + self.ALPHA * inx
            self.fy = py_pred + self.ALPHA * iny
            self.fvx += (self.BETA / dt) * inx
            self.fvy += (self.BETA / dt) * iny
        self.prev_det_t = det.t
        self.t_pub = det.t

    def step(self, t: float, obs: Optional[Observation]) -> Tuple[int, int]:
        cfg = self.cfg
        if obs is not None and obs.new:
            self._update_filter(obs)

        cx = cy = 0
        age = t - self.t_pub
        if self.filt and age < self.STALE:
            cp = self.ch.at(self.t_pub - cfg.L)
            cn = self.ch.cum()
            ex = self.fx + self.fvx * (age + cfg.L) - cfg.s * (cn[0] - cp[0])
            ey = self.fy + self.fvy * (age + cfg.L) - cfg.s * (cn[1] - cp[1])
            r = math.hypot(ex, ey)

            if r > self.brake_dist:
                v_inner = self.wn * self.brake_dist
                if r > self.brake_dist + self.boundary_layer:
                    speed = self.vmax
                else:
                    blend = (r - self.brake_dist) / self.boundary_layer
                    speed = v_inner + blend * (self.vmax - v_inner)
                scale = speed / r
                vcx = ex * scale
                vcy = ey * scale
                self.int_x = 0.0
                self.int_y = 0.0
            else:
                ig = self.i_gate / (self.i_gate + r)
                i_lim = self.vmax / max(self.ki, 1e-9)
                vcx_u = self.kp * ex + self.ki * self.int_x
                vcy_u = self.kp * ey + self.ki * self.int_y
                wind_x = (vcx_u > self.vmax and ex > 0) or (vcx_u < -self.vmax and ex < 0)
                wind_y = (vcy_u > self.vmax and ey > 0) or (vcy_u < -self.vmax and ey < 0)
                if not wind_x:
                    self.int_x = max(-i_lim, min(i_lim, self.int_x + ex * cfg.h * ig))
                if not wind_y:
                    self.int_y = max(-i_lim, min(i_lim, self.int_y + ey * cfg.h * ig))
                vcx = max(-self.vmax, min(self.vmax, vcx_u))
                vcy = max(-self.vmax, min(self.vmax, vcy_u))

            s = max(0.05, min(20.0, cfg.s))
            self.rem_x += vcx * cfg.h / s
            self.rem_y += vcy * cfg.h / s
            cx, cy, self.rem_x, self.rem_y = self._counts(
                self.rem_x, self.rem_y, s, cfg.count_limit)
        else:
            self.rem_x = self.rem_y = 0.0
            self.int_x = self.int_y = 0.0

        self.ch.add(t, cx, cy)
        return cx, cy
