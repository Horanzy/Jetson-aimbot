"""arena/laws/kalman_pi.py — Kalman state predictor + phase-margin PI.

PRINCIPLE
=========
Two time-scale-separated stages, every parameter derived from a physical
quantity or a stated dimensionless design choice. No trial-and-error gains.

1) ESTIMATOR — constant-velocity (CV) Kalman filter, per axis.
   State x = [e, v]ᵀ: e = target−crosshair error (px), v = relative velocity
   (px/ms) with our own control action removed (Smith-style, via the B·u term).
       F = [[1,dt],[0,1]],  B = [[-dt],[0]],  H = [[1,0]]
       Q = q·[[dt³/3, dt²/2],[dt²/2, dt]]   (white-acceleration process noise)
       R = noise_std²                        (measurement variance)
   The filter is a PURE estimator: it outputs position+velocity at the moment
   the frame reflected (t_pub − L̂). No control logic lives inside it.

   Process noise q from the SEPARATION PRINCIPLE. The continuous CV Kalman
   filter solves to a 2nd-order system with damping ζ_f = 1/√2 and natural
   frequency ω_f = (q/R)^(1/4) (from the algebraic Riccati equation:
   P12 = √(qR), P11 = √2·R^(3/4)·q^(1/4) → filter char. poly s²+K1·s+K2,
   K2 = √(q/R) ⇒ ω_f = √K2 = (q/R)^(1/4)). An observer must be faster than
   the controller it feeds; choose ω_f = c·wn with separation factor c (a
   dimensionless design choice, c≈3–5 is standard). Hence
       q = R·(c·wn)⁴.
   Because wn ∝ 1/L (below), q auto-scales: more delay → smoother filter.

   Measurement noise R = noise_std². noise_std is the detector jitter (arena
   default 0.5 px/axis; on real hardware measure from a static scene).

2) DELAY COMPENSATION — linear Smith extrapolation (control rate, 500 Hz).
   The filtered state lives at (t_pub − L̂); extrapolate to "now" by horizon
   T = age + L̂ (age = t − t_pub):
       e_pred = e_filt + v_filt·T − s·(in-flight counts since t_pub − L̂)
   Pure linear extrapolation (no ad-hoc horizon damping): the loop bandwidth
   wn is low enough (PM against the FULL delay) that any residual mismatch
   delay stays inside the phase margin, so bounded-error extrapolation is
   unnecessary for stability.

3) CONTROLLER — pole-placement PI on e_pred.
   Plant ≈ unit integrator (crosshair += s·counts ≈ v·h). PI C(s)=Kp+Ki/s on
   1/s gives closed loop s²+Kp·s+Ki = 0 ⇒ ωn = √Ki, ζ = Kp/(2ωn). Choose
       ζ = 1            (critical damping — dimensionless design choice)
       ωn = (90°−PM)·π/180 / L̂,  PM = 60°   (dimensionless design choice)
   ωn is the bandwidth that leaves PM=60° of phase margin if the delay L̂ were
   NOT compensated (ωn·L̂ = 30°). With the Smith predictor the loop is fast;
   if the predictor fails under mismatch it falls back to this guaranteed
   60° margin → no divergence. Computed in reset() from cfg.L (auto-scales
   ~1/L; ≈0.01047 rad/ms at L=50). NOT a hardcoded tuned constant.
       Kp = 2ζωn,  Ki = ωn².
   Anti-windup: conditional integration + integral clamp ±max_v/Ki (so the
   integral can command up to max_v → zero lag for a target at max speed).
   Distance gate ig = i_gate/(i_gate+|e|): integral off during flicks
   (|e|≫i_gate → no arrival overshoot), on during tracking (|e|≲i_gate →
   zero steady-state drag).

PARAMETERS (each justified)
===========================
  pm_deg = 60      DIMENSIONLESS DESIGN CHOICE. Phase margin the loop keeps
                   even with the delay fully uncompensated. Sets ωn.
  zeta = 1         DIMENSIONLESS DESIGN CHOICE. Critical damping (no overshoot
                   from the linear poles; robust under mismatch).
  sep (c) = 5      DIMENSIONLESS DESIGN CHOICE. Observer/controller separation
                   factor: filter bandwidth = c·wn. ≈5× is the standard rule of
                   thumb (observer fast enough to add negligible lag, slow enough
                   to reject noise). ↑ = more maneuver detail but more noise;
                   ↓ = smoother.
  noise_std = 0.5  PHYSICAL. Detector noise std (px/axis). R = noise_std².
                   Arena default; measure on real hardware from static jitter.
  i_gate = 8.0     PHYSICAL distance (px). Integral activation scale ≈ a few ×
                   the settle tolerance; separates flick (gate off) from track
                   (gate on). Mildly empirical physical scale — flagged.
  l_comp = 1.1     Smith compensation ratio (extrapolate/subtract over L̂·l_comp).
                   >1 over-compensates to guard the dangerous under-compensation
                   side (L_true > L̂): calibration L is a lower bound (the
                   bilateral-key procedure measures the minimum observable delay),
                   so the operating point is placed just past L̂. 1.1 matches the
                   established pi_pm convention. The PM=60 floor absorbs
                   residuals beyond this coverage.
  max_v            from cfg (velocity saturation).

  Derived in reset(): wn, Kp, Ki, q, R. JUMP_GATE=100 / STALE=200 are
  discontinuity/timeout conventions (reference), not tuned control gains.

KNOWN LIMITATIONS
=================
  • CV model cannot predict acceleration; constant-accel targets leave a lag
    ≈ a/Ki the integral removes slowly (the accel-scenario residual).
  • ωn = 0.5236/L̂ is deliberately conservative (PM=60 vs full delay) → slower
    flick than aggressive laws; this buys mismatch robustness / generalization.
  • Delay-mismatch boundary: stable for L_true ≈ 20–70 at belief=50 (no
    divergence); at L_true=80 (+60% delay error) the Smith predictor's
    control-removal window misaligns enough to contaminate the velocity
    estimate and the step does not settle. l_comp=1.1 pushes this boundary out
    versus l_comp=1.0 but cannot cover a +60% error at the required PM=60
    bandwidth. Beyond ±~30ms residual relies on calibration accuracy.
  • If real detector noise ≫ noise_std, the filter over-trusts measurements;
    set noise_std from a real static-scene measurement.
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
import numpy as np
from arena.core import Observation, LawConfig
from arena.laws.base import Law, register


class _CountsHist:
    __slots__ = ("t", "cx", "cy", "cumx", "camy")

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


@register("kalman_pi")
class KalmanPILaw(Law):
    JUMP_GATE = 100.0
    STALE = 200.0
    V0_STD = 1.0          # initial velocity prior std (px/ms); transient only

    def __init__(self, pm_deg=60.0, zeta=1.0, sep=5.0, noise_std=0.5,
                 i_gate=8.0, l_comp=1.1, max_v=1.5):
        self.pm_deg = pm_deg
        self.zeta = zeta
        self.sep = sep
        self.noise_std = noise_std
        self.i_gate = i_gate
        self.l_comp = l_comp
        self._max_v = max_v

    def reset(self, cfg: LawConfig):
        self.cfg = cfg
        self.max_v = self._max_v if self._max_v > 0 else cfg.max_v

        L = max(1.0, cfg.L)
        wn = (90.0 - self.pm_deg) * math.pi / 180.0 / L
        self.kp = 2.0 * self.zeta * wn
        self.ki = wn * wn

        self.R = self.noise_std * self.noise_std
        self.q = self.R * (self.sep * wn) ** 4

        self.H = np.array([[1.0, 0.0]])
        self.ch = _CountsHist()
        self.x_x = np.zeros(2)
        self.x_y = np.zeros(2)
        self.P_x = np.array([[self.R, 0.0], [0.0, self.V0_STD ** 2]])
        self.P_y = np.array([[self.R, 0.0], [0.0, self.V0_STD ** 2]])
        self.initialized = False
        self.prev_det_t = None
        self.t_pub = -1e9
        self.int_x = self.int_y = 0.0
        self.rem_x = self.rem_y = 0.0

    def _update_filter(self, det: Observation):
        cfg = self.cfg
        if not self.initialized:
            self.x_x = np.array([det.dx, 0.0])
            self.x_y = np.array([det.dy, 0.0])
            self.P_x = np.array([[self.R, 0.0], [0.0, self.V0_STD ** 2]])
            self.P_y = np.array([[self.R, 0.0], [0.0, self.V0_STD ** 2]])
            self.initialized = True
            self.prev_det_t = det.t
            self.t_pub = det.t
            return

        dt = max(1.0, min(100.0, det.t - self.prev_det_t))
        Lc = cfg.L * self.l_comp
        c0 = self.ch.at(det.t - Lc - dt)
        c1 = self.ch.at(det.t - Lc)
        u_x = cfg.s * (c1[0] - c0[0]) / dt
        u_y = cfg.s * (c1[1] - c0[1]) / dt

        F = np.array([[1.0, dt], [0.0, 1.0]])
        B = np.array([-dt, 0.0])
        Q = self.q * np.array([[dt ** 3 / 3.0, dt ** 2 / 2.0],
                               [dt ** 2 / 2.0, dt]])

        for axis in (0, 1):
            x = self.x_x if axis == 0 else self.x_y
            P = self.P_x if axis == 0 else self.P_y
            u = u_x if axis == 0 else u_y
            xp = F @ x + B * u
            Pp = F @ P @ F.T + Q
            S = (self.H @ Pp @ self.H.T)[0, 0] + self.R
            K = (Pp @ self.H.T).flatten() / S
            z = det.dx if axis == 0 else det.dy
            innov = z - (self.H @ xp)[0]
            xn = xp + K * innov
            Pn = (np.eye(2) - np.outer(K, self.H.flatten())) @ Pp
            if axis == 0:
                self.x_x, self.P_x, inx = xn, Pn, innov
            else:
                self.x_y, self.P_y, iny = xn, Pn, innov

        if math.hypot(inx, iny) > self.JUMP_GATE:
            self.x_x = np.array([det.dx, 0.0])
            self.x_y = np.array([det.dy, 0.0])
            self.P_x = np.array([[self.R, 0.0], [0.0, self.V0_STD ** 2]])
            self.P_y = np.array([[self.R, 0.0], [0.0, self.V0_STD ** 2]])
            self.int_x = self.int_y = 0.0

        self.prev_det_t = det.t
        self.t_pub = det.t

    def step(self, t: float, obs: Optional[Observation]) -> Tuple[int, int]:
        cfg = self.cfg
        if obs is not None and obs.new:
            self._update_filter(obs)

        cx = cy = 0
        age = t - self.t_pub
        if self.initialized and age < self.STALE:
            Lc = cfg.L * self.l_comp
            cp = self.ch.at(self.t_pub - Lc)
            cn = self.ch.cum()
            T = age + Lc
            ex = self.x_x[0] + self.x_x[1] * T - cfg.s * (cn[0] - cp[0])
            ey = self.x_y[0] + self.x_y[1] * T - cfg.s * (cn[1] - cp[1])

            vx_u = self.kp * ex + self.ki * self.int_x
            vy_u = self.kp * ey + self.ki * self.int_y

            if ex * ex + ey * ey > cfg.fov_radius * cfg.fov_radius:
                self.int_x = self.int_y = 0.0
            else:
                ig = self.i_gate / (self.i_gate + math.hypot(ex, ey)) \
                    if self.i_gate > 0 else 1.0
                i_lim = self.max_v / max(self.ki, 1e-9)
                wx = (vx_u > self.max_v and ex > 0) or (vx_u < -self.max_v and ex < 0)
                wy = (vy_u > self.max_v and ey > 0) or (vy_u < -self.max_v and ey < 0)
                if not wx:
                    self.int_x = max(-i_lim, min(i_lim, self.int_x + ex * cfg.h * ig))
                if not wy:
                    self.int_y = max(-i_lim, min(i_lim, self.int_y + ey * cfg.h * ig))

            vx = max(-self.max_v, min(self.max_v, vx_u))
            vy = max(-self.max_v, min(self.max_v, vy_u))
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
