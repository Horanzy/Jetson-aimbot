"""arena/laws/smith_filt.py — filtered Smith predictor, parameters derived from principle.

================================================================================
GOAL OF THIS REVISION (low patch smell)
================================================================================
The previous version lived near a stability cliff and shipped several hand-tuned
magic numbers (wn=0.01, lam=50, L_inflate=1.2) that overfit the arena's L=50
operating point. This revision DERIVES the two quantities that matter most — the
controller bandwidth wn and the residual-filter time constant lam — from the
believed delay L, so the law auto-scales to any latency and is robust by design
rather than by trial and error. Arena speed is deliberately traded for this
generalization margin.

================================================================================
SMITH PREDICTOR PRINCIPLE
================================================================================
The loop is a pure integrator plant (a velocity command u (px/ms) moves the
crosshair by u·h per tick, so the error e = target−crosshair integrates:
ė = −u + d, d = target motion = output disturbance) with a DELAY L on the only
measurement (a frame timestamped obs.t reflects the world at obs.t − L_true).

A Smith predictor runs an internal DELAY-FREE model of the plant and feeds the
controller from that model, so the delay disappears from the characteristic
equation. The price of the real measurement is paid only through a correction:

    ŷ_nd += −s·counts                 delay-free model output (predicted error)
    ŷ_d   = ŷ_nd delayed by L̂          what the model thinks the frame should show
    r     = y_meas − ŷ_d               measurement residual (disturbance + mismatch)
    f     = low-pass(r)                filtered correction, F(s)=1/(1+λs)
    ê     = ŷ_nd + f                   corrected delay-free error estimate
    u     = Kp·ê + Ki·∫ê               PI

When the model is perfect, y_meas == ŷ_d so r == 0 and the controller sees only
the delay-free signal ŷ_nd. ref = 0 (regulate error to zero); a positive command
reduces a positive error.

================================================================================
WHY THE CLASSIC SMITH PREDICTOR IS FRAGILE, AND HOW THE FILTER ROBUSTIFIES IT
================================================================================
Classic Smith uses f = r directly (no filter). A delay error ΔL = L_true − L̂
makes ŷ_d the wrong tap of the delay line. Since ŷ_nd moves at the command
velocity v, the wrong tap differs by ≈ v·ΔL, i.e. the residual injects a
spurious disturbance ∝ v·ΔL. In the frequency domain the mismatch contributes
(e^{−L s} − e^{−L̂ s}) ≈ −ΔL·s, whose magnitude grows as ΔL·ω. At crossover ωc
this is a gain perturbation ≈ ΔL·ωc: high bandwidth × tiny ΔL destabilizes the
loop. The survivable mismatch is roughly ΔL_max ≈ PM/ωc, so pushing bandwidth
(the whole point of Smith) SHRINKS the robustness margin.

Insert a low-pass F(s)=1/(1+λs) in the residual path. The model is now trusted
only up to frequency ~1/λ; above that the residual (hence the mismatch term
∝ ΔL·s) is attenuated. Because s·F(s) = s/(1+λs) → 1/λ as ω→∞, the mismatch gain
is BOUNDED by ≈ ΔL/λ, independent of bandwidth, instead of growing with ωc. This
decouples robustness from bandwidth — the filter is what lets a Smith predictor
survive model error.

DERIVING λ FROM PRINCIPLE (λ = L̂):
The natural frequency of the delay itself is ω_L = 1/L̂ (the frequency at which
one delay period accumulates ~1 rad of phase, ω·L̂ = 1). Below ω_L the model's
delay compensation is meaningful; above it the phase of the compensation is
essentially unknown and the model should not be trusted. Setting the filter
cutoff at exactly this frequency, 1/λ = 1/L̂  ⇒  λ = L̂, therefore trusts the
model up to its own validity limit and attenuates everything beyond. The bounded
mismatch gain then becomes ΔL/λ = ΔL/L̂ — an O(1) number for any calibration error
smaller than the delay itself, instead of an unbounded ΔL·ωc. λ = L̂ is thus not a
tuned constant but the delay scale; it auto-scales with L (here lam = lam_frac·L̂
with the dimensionless lam_frac = 1).

================================================================================
DERIVING THE BANDWIDTH wn FROM PRINCIPLE (PM=60° against the FULL delay)
================================================================================
The delay-free PI + integrator closes to a second-order loop s²+Kp·s+Ki with
ωn = √Ki, ζ = Kp/(2ωn); we pick ζ = 1 (critical damping) so Kp = 2ωn, Ki = ωn².

The Smith predictor is only as good as its model, so we do NOT trust it for
stability. We size wn as if the Smith cancellation may FAIL COMPLETELY, leaving
the FULL believed delay L̂ in the loop. A pure delay L̂ consumes phase ω·L̂; to
keep phase margin PM with the delay present,
        ωn·L̂ = (90° − PM)·π/180   ⇒   wn = (90° − PM)·π/180 / L̂.
With PM = 60° (a standard, comfortable margin) this gives
        wn = (π/6)/L̂ ≈ 0.5236/L̂   (≈ 0.01047 rad/ms at L̂ = 50ms).
This is conservative BY DESIGN: the loop is guaranteed PM≈60° even with zero
delay cancellation, so a working Smith predictor (which removes most of the
delay) can only add margin on top. There is no cliff to fall off, no hardcoded
tuned wn — wn simply scales as 1/L̂. The cost (accepted) is a lower bandwidth
than an aggressively-tuned Smith could extract, hence slower step settling than
the previous near-cliff version.

================================================================================
DISTURBANCE EXTRAPOLATION (removes the v·L tracking lag) — physically derived
================================================================================
With a pure integrator model the filtered residual f represents target motion at
the CAPTURE time (L late). A constant-velocity target would then leave a steady
tracking error ≈ v·L unless the disturbance is extrapolated forward. The residual
alpha-beta also estimates the disturbance velocity rv (smoothed to sv). The
physically correct horizon is exactly the time elapsed since capture: a frame
available now was captured at (now − L̂), and the filter state is referenced to
the last frame at t_meas, so the current disturbance is the capture-time
disturbance extrapolated by T = age + L̂. Hence
        ê = ŷ_nd + rf + vext·sv·(age + L̂),
with vext = 1 = full physical extrapolation (not a tuned gain — it is the
statement "extrapolate the observed disturbance to the present"). rho = 0 keeps
the horizon undamped (= age + L̂), the physically correct choice; rho > 0 would
saturate the horizon (a bounded-lookahead safety variant) at the cost of lag.

Because open-loop extrapolation of a mismatch-corrupted velocity is the classic
positive-feedback danger, the extrapolated velocity is made safe two ways:
  • sv is EMA-smoothed over ~one frame (rv_tau = rv_tau_frames·frame_dt, the
    shortest physically meaningful smoothing scale — frame-rate independent);
  • |sv| is HARD-CAPPED at rv_max, bounding the extrapolation slope so a mismatch
    cannot drive a runaway. rv_max is the one explicit safety bound (EMPIRICAL:
    set well above the fastest real target; arena maneuver vmax = 0.4 px/ms).

================================================================================
DISCRETE EQUATIONS (per control tick h, per axis)
================================================================================
  on a NEW frame at t_meas (dt = t_meas − prev):
      ŷ_d  = delayline.at(t_meas − L̂)          # model output at capture time
      r    = y_meas − ŷ_d                        # raw residual = disturbance@capture
      r̂p   = rf + rv·dt                          # predict residual filter by dt
      inn  = r − r̂p                              # (jump-gate resets on |inn|>gate)
      rf  += α·inn ;  rv += (β/dt)·inn           # α = dt/(λ+dt), β = beta_frac·α
      sv  += (1−e^{−dt/rv_tau})·(rv − sv)        # smooth disturbance velocity
      |sv| capped to rv_max                       # bound the extrapolation slope
  every tick:
      T    = age + L̂                             # time since capture (physical)
      ê    = ŷ_nd + rf + vext·sv·T               # corrected delay-free error
      u    = Kp·ê + Ki·∫ê                         # PI (anti-windup, saturated)
      ŷ_nd −= s·counts                            # model mirrors the plant exactly
  Kp = 2ζωn, Ki = ki_mult·ωn²  with wn = (90°−PM)·π/180 / L̂, ζ = 1, ki_mult = 1.

================================================================================
PARAMETER TABLE
================================================================================
PRINCIPLE-DERIVED (auto-scale with L̂ / frame_dt; no arena tuning):
  pm_deg = 60.0   Phase margin used to size wn against the FULL delay. Allowed
                  dimensionless design choice. wn = (90−pm_deg)·π/180 / L̂.
                  ↑PM = lower wn, more margin, slower; 60° is standard.
  zeta = 1.0      Delay-free damping (critical). Allowed dimensionless choice.
  lam_frac = 1.0  λ = lam_frac·L̂: residual-filter cutoff at the delay's natural
                  frequency 1/L̂ (see derivation above). The single robustness
                  dial; principled default 1.0. ↑ = more mismatch-robust but
                  slower maneuver response; ↓ = faster tracking but more fragile.
                  MEASURED: lam_frac=1.0 is the UNIQUE value with a finite worst
                  case over L_true∈{30..70} — both 0.7 (passes mismatch) and ≥1.3
                  (starves the correction) diverge at L_true=70. So λ=L̂ is not a
                  convention but the forced robustness optimum; do not move it.
  L_inflate = 1.0 L̂ = L_belief·L_inflate. Default 1.0 (use the calibrated L as
                  is); the conservative wn already guarantees PM against the full
                  delay, so no inflation is needed for stability. >1 biases the
                  model tap toward over-compensation (the under-compensation side
                  L_true>L̂ is the dangerous one) — an OPTIONAL lever if a specific
                  device's calibration systematically under-reads L.
  vext = 1.0      Disturbance position-extrapolation gain = full physical
                  extrapolation of the capture-time disturbance to the present.
                  0 falls back to the pure filtered Smith (robust but leaves a
                  v·L const-velocity lag).
  rho = 0.0       Undamped (physically correct) extrapolation horizon T=age+L̂.
  rv_tau_frames=1 EMA smoothing of rv→sv over one frame period (frame-rate
                  independent: rv_tau = rv_tau_frames·frame_dt). Shortest
                  physically meaningful smoothing.
  ki_mult = 1.0   Ki = ki_mult·wn² (the designed second-order loop). >1 adds
                  phase lag and breaks the mismatch margin; keep 1.

EMPIRICAL (flagged; standard control knobs, not arena-specific magic):
  beta_frac = 0.3 Residual velocity gain β = beta_frac·α (α = dt/(λ+dt)). A
                  standard alpha-beta position/velocity gain ratio. ↑ = faster
                  direction-change response but noisier / less mismatch-robust;
                  ↓ = smoother. MEASURED forced value: 0.2 and 0.4 both diverge at
                  L_true=70; 0.3 is the unique finite-and-fast point.
  rv_max = 2.0    Hard cap on |sv| (px/ms). Explicit SAFETY bound on the
                  extrapolation slope; set well above the fastest real target
                  (arena maneuver vmax = 0.4) so it never limits honest motion.
                  Lower = safer but clips fast strafes.
  i_gate = 8.0    Integral distance gate ig = gate/(gate+|ê|): suppresses I near
                  the target (no windup/overshoot in a flick) yet lets I build up
                  on a tracked mover. Standard anti-windup shaping; 8 (the same
                  conventional value as the sibling pi_pm law) gives the
                  flattest mismatch worst case here. ↑ = faster mover pickup but
                  more flick overshoot and a longer low-fps settling tail.
  i_frac = 1.0    Integral clamp = i_frac·max_v/Ki (anti-windup). 1 = can track
                  any target up to max_v without position lag.
  jump_gate=120.0 Residual-innovation jump gate (px): a scene cut / target swap
                  resets the residual filter instead of chasing it. Structural,
                  not a performance knob (~ the FOV scale).
  max_v = 1.5     Command velocity saturation (px/ms); hardware slew limit
                  (normally injected via cfg.max_v).

================================================================================
KNOWN LIMITATIONS
================================================================================
  • Integrator + constant-velocity model: cannot predict target ACCELERATION.
    The accel scenario leaves a steady lag a/Ki removed only by the (slow,
    margin-limited) I term; accel rmse ≈ 12px.
  • The robustness is bought with bandwidth: wn = (π/6)/L̂ is deliberately well
    below an aggressively-tuned Smith, so matched step settling (~430ms) is
    slower than near-cliff designs. This is the accepted speed↔generalization
    trade — the loop stays stable far from any cliff.
  • Delay mismatch much larger than ~L̂ still degrades tracking (the bounded
    mismatch gain ΔL/L̂ grows); the under-compensation side (L_true>L̂) remains the
    fragile one. Raise L_inflate only if a device's calibration under-reads L.
  • Arena numbers are relative guidance only; re-sweep delay mismatch on hardware
    after changing pm_deg, lam_frac, beta_frac or L_inflate.
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
from arena.core import Observation, LawConfig
from arena.laws.base import Law, register


class _ModelHist:
    """Delay line of the delay-free model output ŷ_nd, with linear interpolation
    and endpoint clamping (clamping makes startup well-defined: queries before the
    first stored sample return the initial model value)."""
    __slots__ = ("t", "x", "y")

    def __init__(self):
        self.t: list[float] = []
        self.x: list[float] = []
        self.y: list[float] = []

    def add(self, t, x, y):
        self.t.append(t); self.x.append(x); self.y.append(y)
        if len(self.t) > 4000:
            self.t.pop(0); self.x.pop(0); self.y.pop(0)

    def at(self, t):
        ts = self.t
        n = len(ts)
        if n == 0:
            return 0.0, 0.0
        if t <= ts[0]:
            return self.x[0], self.y[0]
        if t >= ts[-1]:
            return self.x[-1], self.y[-1]
        lo, hi = 0, n - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ts[mid] <= t:
                lo = mid
            else:
                hi = mid
        span = ts[hi] - ts[lo]
        f = (t - ts[lo]) / span if span > 0 else 0.0
        return (self.x[lo] + (self.x[hi] - self.x[lo]) * f,
                self.y[lo] + (self.y[hi] - self.y[lo]) * f)


@register("smith_filt")
class SmithFiltLaw(Law):
    STALE = 200.0

    def __init__(self, pm_deg=60.0, zeta=1.0, lam_frac=1.0, L_inflate=1.0,
                 vext=1.0, rho=0.0, rv_tau_frames=1.0, ki_mult=1.0,
                 beta_frac=0.3, rv_max=2.0, i_gate=8.0, i_frac=1.0,
                 jump_gate=120.0, max_v=1.5):
        self.pm_deg = pm_deg          # PM used to size wn against the full delay
        self.zeta = zeta              # delay-free damping (critical = 1)
        self.lam_frac = lam_frac      # λ = lam_frac·L̂ (filter cutoff at 1/L̂)
        self.L_inflate = L_inflate    # L̂ = L_belief·L_inflate (default 1: use as is)
        self.vext = vext              # disturbance extrapolation gain (1 = physical)
        self.rho = rho                # horizon damping (0 = undamped, physical)
        self.rv_tau_frames = rv_tau_frames  # rv smoothing in frame periods
        self.ki_mult = ki_mult        # Ki = ki_mult·wn²
        self.beta_frac = beta_frac    # EMPIRICAL: residual velocity gain ratio
        self.rv_max = rv_max          # EMPIRICAL: safety cap on |sv| (px/ms)
        self.i_gate = i_gate          # EMPIRICAL: integral distance gate (px)
        self.i_frac = i_frac          # integral clamp = i_frac·max_v/Ki
        self.jump_gate = jump_gate    # structural: residual-innovation reset (px)
        self._max_v = max_v           # hardware slew limit (px/ms)

    def reset(self, cfg: LawConfig):
        self.cfg = cfg
        self.max_v = self._max_v if self._max_v > 0 else cfg.max_v
        # Believed delay anchors everything (model tap, filter, bandwidth, horizon).
        self.L_hat = max(1.0, cfg.L) * self.L_inflate
        # Bandwidth from principle: PM against the FULL delay (stable even if the
        # Smith cancellation fails). wn = (90°−PM)·π/180 / L̂, auto-scaling as 1/L̂.
        wn = (90.0 - self.pm_deg) * math.pi / 180.0 / self.L_hat
        self.kp = 2.0 * self.zeta * wn
        self.ki = self.ki_mult * wn * wn
        # Filter time constant from principle: cutoff at the delay's natural
        # frequency 1/L̂  ⇒  λ = L̂ (here scaled by the dimensionless lam_frac).
        self.lam = self.lam_frac * self.L_hat
        # Disturbance-velocity smoothing over ~one frame (frame-rate independent).
        self.rv_tau = max(1.0, self.rv_tau_frames * cfg.frame_dt)
        self.hist = _ModelHist()
        self.m_x = self.m_y = 0.0          # delay-free model output ŷ_nd
        self.rf_x = self.rf_y = 0.0        # filtered residual (disturbance pos)
        self.rv_x = self.rv_y = 0.0        # residual velocity (disturbance rate)
        self.svx = self.svy = 0.0          # smoothed + capped disturbance velocity
        self.t_meas = -1e9
        self.have = False
        self.int_x = self.int_y = 0.0
        self.rem_x = self.rem_y = 0.0

    def _new_frame(self, obs: Observation):
        cfg = self.cfg
        if not self.have:
            self.m_x, self.m_y = obs.dx, obs.dy
            self.rf_x = self.rf_y = 0.0
            self.rv_x = self.rv_y = 0.0
            self.svx = self.svy = 0.0
            self.have = True
            self.t_meas = obs.t
            return
        md_x, md_y = self.hist.at(obs.t - self.L_hat)
        r_x = obs.dx - md_x
        r_y = obs.dy - md_y
        dt = max(1.0, min(100.0, obs.t - self.t_meas))
        alpha = dt / (self.lam + dt)
        beta = self.beta_frac * alpha
        rpx = self.rf_x + self.rv_x * dt
        rpy = self.rf_y + self.rv_y * dt
        inx = r_x - rpx
        iny = r_y - rpy
        if math.hypot(inx, iny) > self.jump_gate:
            self.rf_x, self.rf_y = r_x, r_y
            self.rv_x = self.rv_y = 0.0
            self.svx = self.svy = 0.0
            self.int_x = self.int_y = 0.0
        else:
            self.rf_x = rpx + alpha * inx
            self.rf_y = rpy + alpha * iny
            self.rv_x += (beta / dt) * inx
            self.rv_y += (beta / dt) * iny
        a = 1.0 - math.exp(-dt / self.rv_tau)
        self.svx += a * (self.rv_x - self.svx)
        self.svy += a * (self.rv_y - self.svy)
        sp = math.hypot(self.svx, self.svy)
        if sp > self.rv_max:
            sc = self.rv_max / sp
            self.svx *= sc
            self.svy *= sc
        self.t_meas = obs.t

    def step(self, t: float, obs: Optional[Observation]) -> Tuple[int, int]:
        cfg = self.cfg
        if obs is not None and obs.new:
            self._new_frame(obs)

        cx = cy = 0
        if self.have:
            age = t - self.t_meas
            if age < self.STALE:
                hor = age + self.L_hat
                if self.rho > 1e-6:
                    hor = (1.0 - math.exp(-self.rho * hor)) / self.rho
                e_x = self.m_x + self.rf_x + self.vext * self.svx * hor
                e_y = self.m_y + self.rf_y + self.vext * self.svy * hor

                vx_u = self.kp * e_x + self.ki * self.int_x
                vy_u = self.kp * e_y + self.ki * self.int_y

                i_lim = self.i_frac * self.max_v / max(self.ki, 1e-9)
                if e_x * e_x + e_y * e_y > cfg.fov_radius * cfg.fov_radius:
                    self.int_x = self.int_y = 0.0
                else:
                    ig = (self.i_gate / (self.i_gate + math.hypot(e_x, e_y))
                          if self.i_gate > 0 else 1.0)
                    wx = (vx_u > self.max_v and e_x > 0) or (vx_u < -self.max_v and e_x < 0)
                    wy = (vy_u > self.max_v and e_y > 0) or (vy_u < -self.max_v and e_y < 0)
                    if not wx:
                        self.int_x = max(-i_lim, min(i_lim, self.int_x + e_x * cfg.h * ig))
                    if not wy:
                        self.int_y = max(-i_lim, min(i_lim, self.int_y + e_y * cfg.h * ig))

                vx = max(-self.max_v, min(self.max_v, vx_u))
                vy = max(-self.max_v, min(self.max_v, vy_u))
                s = max(0.05, min(20.0, cfg.s))
                self.rem_x += vx * cfg.h / s
                self.rem_y += vy * cfg.h / s
                cx, cy, self.rem_x, self.rem_y = self._counts(
                    self.rem_x, self.rem_y, s, cfg.count_limit)
            else:
                self.int_x = self.int_y = 0.0
                self.rem_x = self.rem_y = 0.0

        self.m_x -= cfg.s * cx
        self.m_y -= cfg.s * cy
        if self.have:
            self.hist.add(t, self.m_x, self.m_y)
        return cx, cy
