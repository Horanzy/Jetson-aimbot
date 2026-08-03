"""arena/laws/ff_pi.py — PI (pole-placed from the delay) + type-2 velocity feedforward.

ALGORITHM PRINCIPLE
===================
Smith-predicted PI loop (same backbone as reference.py / pi_pm.py) plus an
additive target-velocity feedforward term:

    v_cmd = (Kp*e + Ki*∫e)            # PI: drives the *residual* error to zero
          + ff_gain * gate * v̂_target  # FF: drives the bulk of constant-vel tracking

where gate = i_gate/(i_gate+|e|) is the settled-regime gate (see below).

Plant model and why each piece is what it is
--------------------------------------------
The crosshair is a pure integrator of commanded velocity: sending v (px/ms)
moves the crosshair by v*h per tick, so P(s) = 1/s. The only delay is in the
measurement (a frame stamped t reflects the world at t - L_true).

* Bandwidth from the delay (no tuned wn).
  A type-1/2 loop is ~ -90deg at crossover; the measurement delay e^{-sL} adds
  -w*L radians more. Phase margin is therefore
      PM = 180 - 90 - w_c*L*(180/pi) = 90 - w_c*L*(180/pi).
  Solving for the crossover that yields a chosen PM:
      w_c = (90 - PM)*pi/180 / L.
  We use this *as* the design natural frequency wn (computed in reset() from the
  believed L), with PM = 60deg. At L=50 this gives wn = 0.01047 rad/ms. This is
  deliberately a LOW bandwidth: it buys delay margin (generalization) and we
  recover tracking speed with feedforward instead of by raising wn.

* Pole placement (zeta = 1).
  PI C(s)=Kp+Ki/s against P(s)=1/s gives, after the Smith predictor removes the
  delay, the closed-loop characteristic s^2 + Kp*s + Ki = 0, i.e.
      wn = sqrt(Ki),  zeta = Kp/(2*wn)   <=>   Kp = 2*zeta*wn,  Ki = wn^2.
  zeta = 1 (critical damping) is a dimensionless design choice: no overshoot and
  the best mismatch robustness of the well-damped family.

* Smith predictor.
  An alpha-beta tracker estimates the error f and the *target* velocity v̂ at the
  measurement instant (t - L̂); its prediction step subtracts the effect of our
  own in-flight counts so v̂ tracks target motion only. Each tick the error is
  propagated forward:  e = f + v̂*(age + L̂*l_comp) - s*Σcounts(in flight).
  l_comp > 1 over-compensates on purpose: under-estimating L (L_true > L̂) is the
  dangerous direction (residual positive delay -> phase lag -> instability), so
  we bias the compensation window to give that side phase lead.

* Type-2 velocity feedforward (ff_gain = 1).
  For a constant-velocity target the crosshair must move at the target velocity
  v_t to track with zero lag. Since the plant is an integrator, commanding
  v_cmd = v_t makes the crosshair velocity equal v_t directly. Feeding forward
  ff_gain * v̂_target with ff_gain = 1 is therefore the *exact* open-loop command
  for a constant-velocity target: the loop becomes type-2 (zero steady-state
  error to constant velocity) immediately, instead of waiting for the integrator
  to build up the same command over ~1/Ki. The PI then only has to correct the
  residual (estimation error, acceleration, mismatch). ff_gain = 1 is the
  principled value; it is not tuned.

WHY FEEDFORWARD IS DANGEROUS, AND THE PRINCIPLED DEFENSES
---------------------------------------------------------
FF is open loop: nothing in the characteristic equation corrects it (so it does
NOT change linear closed-loop stability). Its two failure modes both enter
through the *estimator*, not the control polynomial:

  1. Estimator noise / recoil. v̂ carries measurement noise; ff_gain=1 injects it
     straight into the command. This is the oscillation the real project warns
     about. Defense (from principle, not an ad-hoc filter): the alpha-beta
     velocity gain beta IS the smoothing. A small beta makes v̂ a low-bandwidth
     estimate that rejects per-frame noise/recoil before it reaches the command.
     We deliberately add NO separate EMA — that would be hidden control tuning.
     beta is one estimator parameter doing double duty (noise + maneuver).

  2. Delay-mismatch self-contamination. When L̂ != L_true the filter's count
     subtraction uses the wrong window, so v̂ absorbs a bias proportional to our
     own command rate; ff_gain=1 then re-commands it -> a positive-feedback loop
     through the estimator (this is what makes naive FF diverge at L_true=70).
     Defenses (all principled): (a) low beta lowers the loop gain through the
     estimator; (b) l_comp>1 cancels the bias on the dangerous underestimate
     side; (c) the low feedback bandwidth (PM=60) keeps the residual the PI has
     to handle well inside its stability margin.

THE SETTLED-REGIME GATE, AND THE SPEED/GENERALIZATION TRADEOFF
--------------------------------------------------------------
The type-2 FF is EXACT only for a constant-velocity target in steady tracking;
during a transient (flick, maneuver, acceleration) v̂ is contaminated by the
closing dynamics and is not the target velocity. We therefore gate FF (and the
integrator) by gate = i_gate/(i_gate+|e|): both steady-state mechanisms are full
strength only when the loop is settled (small error) and fade over the settling
band. This reuses the single integrator gate scale i_gate (no new knob) — the
"near equilibrium" scale is one physical concept used twice.

This gate is also what makes the law frame-rate robust. The velocity estimate v̂
adapts at a slightly frame-rate-dependent rate, so an ungated FF tracks an
accelerating target noticeably worse at 60fps than 120fps (arena: accel rmse
3.4 -> 6.5, fps delta ~13%). Gating FF to the settled regime removes the
frame-rate-sensitive transient contribution and brings the 60/120fps delta to
~3%. The honest cost: an accelerating target's error grows, so the gate fades FF
just where it would help, and acceleration tracking falls back toward the slower
I term (arena accel rmse ~11). This is a deliberate speed->generalization trade,
accepted per the design brief; constant-velocity tracking (the common case) is
unaffected (rmse < 1px, 100% in band).

PARAMETER TABLE  (each: principle-derived how, or EMPIRICAL why)
----------------------------------------------------------------
wn = (90 - pm_deg)*pi/180 / L      [PRINCIPLE] crossover from delay phase margin.
    Computed in reset() from cfg.L. No hardcoded wn. ~0.01047 at L=50.
pm_deg = 60.0                      [PRINCIPLE/dimensionless] target phase margin.
zeta = 1.0                         [PRINCIPLE/dimensionless] critical damping.
ff_gain = 1.0                      [PRINCIPLE] exact type-2 command for const vel.
l_comp = 1.1                       [PRINCIPLE] Smith over-compensation; biases the
    compensation window toward the dangerous L_true>L̂ side. 1.0 = neutral.
alpha0 = 0.50                      [estimator] position gain @120fps, dt-normalized
    (frame-rate independent). Codebase convention (pi_pm).
beta0 = 0.04                       [estimator] velocity gain @120fps, dt-normalized.
    THE noise/maneuver/robustness tradeoff: smaller = smoother v̂ (less FF noise,
    more mismatch robustness) but slower maneuver response. This single knob is
    what tames the FF; it is an estimator-bandwidth choice, not a control patch.
beta_exp = 1.0                     [estimator] dt-scaling exponent of beta
    (beta = beta0*(dt/DT0)**beta_exp). 1 = constant velocity adaptation per
    maneuver event (codebase convention, pi_pm); keeps the loop frame-rate
    consistent. Not retuned.
i_gate = 8.0 px                    [EMPIRICAL, mild — the one settled-regime scale]
    gate = i_gate/(i_gate+|e|) gates BOTH the integrator (prevents flick windup ->
    low step overshoot) and the FF (FF valid only when settled). pi_pm's value.
    Smaller = cleaner flick / better fps robustness but weaker accel tracking;
    larger = the reverse. This is the only place arena speed is traded away.
i_frac = 1.0                       [PRINCIPLE] integrator clamp = i_frac*max_v/Ki;
    the largest steady target speed trackable without lag = i_frac*max_v.

ARENA RESULT (default params)
-----------------------------
OVERALL=153.2 (reference pure-PI = 209.7). Matched L=50/120fps composite=171.3
(step settle 377ms / over 3.5px / first 198ms; const_vel rmse 0.9px 100% in band;
accel rmse 11.3 [the gate tradeoff]; maneuver rmse 23.1). Worst delay mismatch
=122.9 (L_true=70); NO divergence across the full wide-delay sweep L_true=20..80
nor the s-mismatch sweep s_belief=0.7..1.3. Relock settle 483ms / over 3.5px.
60/120fps delta 3.0%.

REAL-DEVICE TUNING
------------------
1. Calibrate s and L; if L reads systematically low keep l_comp>=1.1.
2. wn auto-scales with 1/L; nothing to set. If the device is noisier than the
   simulator, lower beta0 (smoother v̂) before touching anything else.
3. ff_gain=1 is the design point; only lower it (toward 0 = pure PI) if recoil
   noise on real hardware is far worse than the estimator smoothing can absorb.
4. Re-run the delay-mismatch sweep after any change to beta0 or l_comp.
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
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


@register("ff_pi")
class FFPILaw(Law):
    DT0 = 1000.0 / 120.0
    JUMP_GATE = 100.0
    STALE = 200.0

    def __init__(self, zeta=1.0, pm_deg=60.0, ff_gain=1.0, l_comp=1.1,
                 alpha0=0.50, beta0=0.04, beta_exp=1.0, i_gate=8.0, i_frac=1.0,
                 max_v=1.5):
        self.zeta = zeta
        self.pm_deg = pm_deg
        self.ff_gain = ff_gain
        self.l_comp = l_comp
        self.alpha0 = alpha0
        self.beta0 = beta0
        self.beta_exp = beta_exp
        self.i_gate = i_gate
        self.i_frac = i_frac
        self._max_v = max_v

    def reset(self, cfg: LawConfig):
        self.cfg = cfg
        self.max_v = self._max_v if self._max_v > 0 else cfg.max_v
        L = max(1.0, cfg.L)
        wn = (90.0 - self.pm_deg) * math.pi / 180.0 / L
        self.kp = 2.0 * self.zeta * wn
        self.ki = wn * wn
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
        Lc = cfg.L * self.l_comp
        c0 = self.ch.at(det.t - Lc - dt)
        c1 = self.ch.at(det.t - Lc)
        px_pred = self.fx + self.fvx * dt - cfg.s * (c1[0] - c0[0])
        py_pred = self.fy + self.fvy * dt - cfg.s * (c1[1] - c0[1])
        inx = det.dx - px_pred
        iny = det.dy - py_pred
        if math.hypot(inx, iny) > self.JUMP_GATE:
            self.fx, self.fy = det.dx, det.dy
            self.fvx = self.fvy = 0.0
        else:
            r = dt / self.DT0
            alpha = min(0.90, self.alpha0 * r)
            beta = min(0.60, self.beta0 * r ** self.beta_exp)
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
            Lc = cfg.L * self.l_comp
            cp = self.ch.at(self.t_pub - Lc)
            cn = self.ch.cum()
            ex = self.fx + self.fvx * (age + Lc) - cfg.s * (cn[0] - cp[0])
            ey = self.fy + self.fvy * (age + Lc) - cfg.s * (cn[1] - cp[1])
            r = math.hypot(ex, ey)
            gate = self.i_gate / (self.i_gate + r) if self.i_gate > 0 else 1.0

            i_lim = self.i_frac * self.max_v / max(self.ki, 1e-9)
            vx_u = self.kp * ex + self.ki * self.int_x
            vy_u = self.kp * ey + self.ki * self.int_y

            if ex * ex + ey * ey > cfg.fov_radius * cfg.fov_radius:
                self.int_x = self.int_y = 0.0
            else:
                wx = (vx_u > self.max_v and ex > 0) or (vx_u < -self.max_v and ex < 0)
                wy = (vy_u > self.max_v and ey > 0) or (vy_u < -self.max_v and ey < 0)
                if not wx:
                    self.int_x = max(-i_lim, min(i_lim, self.int_x + ex * cfg.h * gate))
                if not wy:
                    self.int_y = max(-i_lim, min(i_lim, self.int_y + ey * cfg.h * gate))

            vx_u += self.ff_gain * gate * self.fvx
            vy_u += self.ff_gain * gate * self.fvy

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
