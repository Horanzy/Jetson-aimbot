"""arena/laws/reference.py — 参考基线 law: Smith+PI (手调 τ)。

alpha-beta 跟踪 (预测步扣除自身控制作用) + Smith 预测 (前推 age+L, 扣除在途 counts)
+ PI (τ_eff/τ_i_eff 兜底, I 距离衰减, 抗饱和) + 余数量化。
用于 arena 自检: 结构保守、行为已知 (稳定收敛、可见爬坡), 复现不出 → arena 没对齐。
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


@register("reference")
class ReferenceLaw(Law):
    ALPHA = 0.30
    BETA = 0.08
    JUMP_GATE = 100.0
    STALE = 200.0
    TAU_L_RATIO = 2.0
    TI_TAU_RATIO = 0.5
    I_GATE_DIST = 30.0

    def __init__(self, tau=80.0, tau_i=60.0, max_v=1.5):
        self.tau = tau
        self.tau_i = tau_i
        self._max_v = max_v

    def reset(self, cfg: LawConfig):
        self.cfg = cfg
        self.max_v = self._max_v if self._max_v > 0 else cfg.max_v
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
            ifx = cfg.s * (cn[0] - cp[0])
            ify = cfg.s * (cn[1] - cp[1])
            ex = self.fx + self.fvx * (age + cfg.L) - ifx
            ey = self.fy + self.fvy * (age + cfg.L) - ify

            tau_eff = max(self.tau, self.TAU_L_RATIO * cfg.L)
            tau_i_eff = max(self.tau_i, self.TI_TAU_RATIO * tau_eff)
            ki = 1.0 / (tau_i_eff * tau_eff)
            i_lim = self.max_v / ki

            vcx_u = ex / tau_eff + ki * self.int_x
            vcy_u = ey / tau_eff + ki * self.int_y
            if ex * ex + ey * ey > cfg.fov_radius * cfg.fov_radius:
                self.int_x = self.int_y = 0.0
            else:
                r = math.hypot(ex, ey)
                ig = self.I_GATE_DIST / (self.I_GATE_DIST + r)
                wind_x = (vcx_u > self.max_v and ex > 0) or (vcx_u < -self.max_v and ex < 0)
                wind_y = (vcy_u > self.max_v and ey > 0) or (vcy_u < -self.max_v and ey < 0)
                if not wind_x:
                    self.int_x = max(-i_lim, min(i_lim, self.int_x + ex * cfg.h * ig))
                if not wind_y:
                    self.int_y = max(-i_lim, min(i_lim, self.int_y + ey * cfg.h * ig))

            vcx = max(-self.max_v, min(self.max_v, vcx_u))
            vcy = max(-self.max_v, min(self.max_v, vcy_u))
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
