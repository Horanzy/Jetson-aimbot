"""arena/laws/ballistic.py — 时间最优两段式自瞄: 开环弹道甩枪 + 临界阻尼闭环收敛。

算法原理
========
延迟 L 是硬约束: 现在发的指令要 L 之后才落地, 时间最优结构被强制为两段。

1. 弹道段 (开环, 延迟免疫): 预测误差 |ê| 大于制动距离时, 以 Vmax 全速朝预测目标
   甩枪。此段指令只取决于 ê 的方向 (饱和), 与幅值无关 → 对延迟与估计误差免疫。
2. 收敛段 (闭环 PI): |ê| 降到制动距离以内, 临界阻尼 PI 无过冲地收尾。

制动点 = 物理值 Vmax·L (brake_factor=1.0):
   以 Vmax 接近目标时, 现在发的指令 L 后才生效, 那 L 内还会滑行 Vmax·L。所以必须
   在距目标约 Vmax·L 处开始制动——提前一个延迟长度。这是物理值, 不是调出来的。
   (brake_factor<1 = 制动更晚 = 拿延迟裕度换 arena 速度分, 是本律拒绝的过拟合。)

收敛带宽 wn 由延迟裕度导出:
   纯延迟 e^{-sL} 在频率 w 处贡献相位滞后 w·L (rad)。要求相位裕度 PM, 令
   wn·L = (90°−PM) → wn = (90°−PM)·π/180 / L。PM=60° 时 wn≈0.01047 @ L=50。
   wn 随 L 自动缩放: 延迟越大带宽越低, 无需额外 τ 兜底——这就是相比原版去掉
   tau_l_ratio / tau_i_ratio / pred_factor 三个经验旋钮的原因。ζ=1 临界阻尼 →
   Kp=2ζωn, Ki=ωn², 无过冲。注意 Kp·(Vmax·L)=2·(90−PM)π/180·Vmax≈1.05·Vmax,
   即制动点处 P 项恰好接近饱和 → 弹道↔收敛切换天然平滑 (边界层只做噪声下的软过渡)。

边界层 = Vmax·L 的比例 (bound_frac):
   制动点外侧一个薄带内, 指令从全速弹道平滑混到收敛值, 避免噪声使模式硬切换抖动
   (bang-bang)。厚度按 Vmax·L 缩放 (帧率/延迟无关), 不是任意 px 常数。

预测器: alpha-beta (α0=0.30 β0=0.08, 按 dt/DT0 归一 → 帧率无关), Smith 前推 (age+L)
并扣除在途 counts。前推时域就是相信的物理延迟 L, **不做过预测**: 原版 pred_factor=1.15
是帮欠补偿侧的经验补丁, 配高速度增益在 L_true=30 制造稳定悬崖; 按"低补丁味"要求移除,
接受欠补偿侧 (L真>L信) 跟踪略慢, 换全程无发散悬崖的鲁棒剖面 (实测失配单调退化、不发散)。

参数表 (默认 / 来源)
====================
| 参数         | 默认    | 来源                                                          |
|--------------|---------|---------------------------------------------------------------|
| pm_deg       | 60      | 原理导出: 目标相位裕度, 定 wn=(90−PM)π/180/L                    |
| zeta         | 1.0     | 原理导出: 临界阻尼无过冲 (Kp=2ζωn, Ki=ωn²)                      |
| brake_factor | 1.0     | 物理值: 制动点=Vmax·L (一个延迟长度); <1 是被拒绝的过拟合       |
| bound_frac   | 0.20    | 经验(缩放): 边界层=bound_frac·Vmax·L; 防切换抖+稳切换。0.15-0.25 等效(不敏感), 取中; <0.10 切换失稳 |
| i_gate_frac  | 0.20    | 经验(缩放): 积分门限=i_gate_frac·Vmax·L; 远处弱积分防过冲       |
| vmax_frac    | 0.95    | 经验(小): 留 5% 速度余量防量化截断                              |
| ALPHA0/BETA0 | .30/.08 | 继承 reference: alpha-beta 增益@120fps, 按 dt/DT0 归一 → 帧率无关 |

实机调参
========
1. 标定 s,L。wn 自动随 L 缩放, 无需手调 τ。
2. 过冲 → 升 brake_factor (≥1, 更早制动) 或升 bound_frac。
3. 拉枪慢 → 升 vmax_frac (→1.0); 制动点已是物理极限, 不要降 brake_factor 到 <1。
4. 移动目标拖尾 → 升 i_gate_frac; 静止目标收尾过冲 → 降 i_gate_frac。
5. 失配振荡 → 升 pm_deg (降 wn) 或升 zeta。
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


@register("ballistic")
class BallisticLaw(Law):
    DT0 = 1000.0 / 120.0
    ALPHA0 = 0.30
    BETA0 = 0.08
    JUMP_GATE = 100.0
    STALE = 200.0

    def __init__(self, pm_deg=60.0, zeta=1.0, brake_factor=1.0,
                 bound_frac=0.20, i_gate_frac=0.20, vmax_frac=0.95):
        self._pm_deg = pm_deg
        self._zeta = zeta
        self._brake_factor = brake_factor
        self._bound_frac = bound_frac
        self._i_gate_frac = i_gate_frac
        self._vmax_frac = vmax_frac

    def reset(self, cfg: LawConfig):
        self.cfg = cfg
        self.vmax = cfg.max_v * self._vmax_frac
        L = max(1.0, cfg.L)
        wn = (90.0 - self._pm_deg) * math.pi / 180.0 / L
        self.kp = 2.0 * self._zeta * wn
        self.ki = wn * wn
        self.i_lim = self.vmax / max(self.ki, 1e-9)
        self.brake_dist = self.vmax * L * self._brake_factor
        self.boundary = self._bound_frac * self.vmax * L
        self.i_gate = self._i_gate_frac * self.vmax * L
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
        dt = max(1.0, min(100.0, det.t - self.prev_det_t))
        c0 = self.ch.at(det.t - cfg.L - dt)
        c1 = self.ch.at(det.t - cfg.L)
        px_pred = self.fx + self.fvx * dt - cfg.s * (c1[0] - c0[0])
        py_pred = self.fy + self.fvy * dt - cfg.s * (c1[1] - c0[1])
        inx = det.dx - px_pred
        iny = det.dy - py_pred
        if math.hypot(inx, iny) > self.JUMP_GATE:
            self.fx, self.fy = det.dx, det.dy
            self.fvx = self.fvy = 0.0
        else:
            r = dt / self.DT0
            alpha = min(0.90, self.ALPHA0 * r)
            beta = min(0.60, self.BETA0 * r)
            self.fx = px_pred + alpha * inx
            self.fy = py_pred + alpha * iny
            self.fvx += (beta / dt) * inx
            self.fvy += (beta / dt) * iny
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

            brake_outer = self.brake_dist + self.boundary
            if r > brake_outer:
                scale = self.vmax / r
                vx = ex * scale
                vy = ey * scale
                self.int_x = 0.0
                self.int_y = 0.0
            else:
                vcx_u = self.kp * ex + self.ki * self.int_x
                vcy_u = self.kp * ey + self.ki * self.int_y
                if r > cfg.fov_radius:
                    self.int_x = self.int_y = 0.0
                else:
                    ig = self.i_gate / (self.i_gate + r)
                    wx = (vcx_u > self.vmax and ex > 0) or (vcx_u < -self.vmax and ex < 0)
                    wy = (vcy_u > self.vmax and ey > 0) or (vcy_u < -self.vmax and ey < 0)
                    if not wx:
                        self.int_x = max(-self.i_lim, min(self.i_lim,
                                                          self.int_x + ex * cfg.h * ig))
                    if not wy:
                        self.int_y = max(-self.i_lim, min(self.i_lim,
                                                          self.int_y + ey * cfg.h * ig))
                vcx = max(-self.vmax, min(self.vmax, vcx_u))
                vcy = max(-self.vmax, min(self.vmax, vcy_u))
                if r > self.brake_dist:
                    b = (r - self.brake_dist) / self.boundary
                    bscale = self.vmax / r
                    vx = b * (ex * bscale) + (1.0 - b) * vcx
                    vy = b * (ey * bscale) + (1.0 - b) * vcy
                else:
                    vx = vcx
                    vy = vcy

            s = max(0.05, min(20.0, cfg.s))
            self.rem_x += vx * cfg.h / s
            self.rem_y += vy * cfg.h / s
            cx, cy, self.rem_x, self.rem_y = self._counts(
                self.rem_x, self.rem_y, s, cfg.count_limit)
        else:
            self.rem_x = self.rem_y = 0.0
            self.int_x = self.int_y = 0.0

        self.ch.add(t, cx, cy)
        return cx, cy
