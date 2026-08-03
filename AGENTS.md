# AGENTS.md

## Overview

AI visual aimbot (mouse pass-through). Target hardware: **NVIDIA Jetson Orin** (aarch64, JetPack / TensorRT 10 / CUDA / OpenCV4 / GStreamer).

```
Capture card (UVC 1080p NV12) → GStreamer nvvidconv → CUDA preprocess → TensorRT YOLO
→ alpha-beta tracking → control law (pole-placement PI + type-2 velocity feedforward)
→ merged with the real mouse → USB Gadget (/dev/hidg0, IE3.0) → game
```

No hand-tuned gains: a bilateral side-key trigger runs auto-calibration, estimating sensitivity s (px/count) and loop delay L (ms) online. Adapts to PC / PS5 / 60fps / 120fps.

**Control law**: the main program `src/aimbot.cu` uses **ffpi** (pole-placement PI + type-2 velocity feedforward) — the convergence bandwidth `wn` is derived from the calibrated delay `L` via phase margin (`wn=(90°−PM)π/180/L`, PM=60°), **no hand-tuned magic numbers**, good generalization. The same binary has **optional training-data collection** (enabled with `-o`, otherwise pure aimbot). `src/aimbot_ballistic.cu` / `src/aimbot_sliding.cu` are **alternative control laws**, kept as-is (no collection), each a Pareto point in speed/robustness (see the "arena control-law simulation" section). All control-law exploration/comparison/tuning happens in the pure-Python `arena/` simulation (this machine cannot compile .cu).

## Environment

- This folder is a **Windows development mirror**; code editing only.
- Compile and run happen on the Jetson. Deployment directory e.g. `/mnt/TF/aimbot/`; layout (dev mirror matches deployment):

```
/mnt/TF/aimbot/
├── src/         aimbot.cu (ffpi + optional collection) / aimbot_ballistic.cu / aimbot_sliding.cu
├── scripts/     compile.sh / convert.sh / setup_mouse.sh
│   └── game/    per-game launch scripts (battlefield.sh)
├── bin/         build outputs (aimbot / aimbot_ballistic / aimbot_sliding)
├── engine/      *.engine model library
├── onnx/        *.onnx
└── dataset/     collection output (fire/ det/ auto/)
```

- **All scripts resolve paths from their own location** (`realpath "$0"`, walking up to `ROOT`), independent of the calling cwd. **The whole directory can be moved/renamed freely** without breaking anything — no hardcoded deployment paths; only system paths like `/usr`, `/dev`, `/sys` are absolute.
- `dataset/` is always at the **project root** (`$ROOT/dataset`, a sibling of `engine/` and `scripts/`); it never ends up inside `bin/` just because the binary lives there — `-o` receives an absolute path computed from the script location.
- This machine has no TRT/GStreamer and **cannot compile or verify**. After edits, hand off to the user to run `scripts/compile.sh` on the Jetson; never claim anything was verified here.

## Files

| File | Role |
|---|---|
| `src/aimbot.cu` | **Main program**: full aimbot + optional training-data collection. Control law ffpi (pole-placement PI + type-2 velocity feedforward; wn derived from L, no hand tuning). Without `-o` it is pure aimbot |
| `src/aimbot_ballistic.cu` | Alternative law: ballistic flick + critically damped convergence (brake point = Vmax·L, wn derived from L). Fastest flick + best maneuver tracking + frame-rate independent. Pure aimbot, no collection |
| `src/aimbot_sliding.cu` | Alternative law: boundary-layer sliding mode + ballistic flick (wn from the full delay at PM=60). Robustness first: zero divergence under all mismatches + flattest profile, but slowest. Pure aimbot, no collection |
| `scripts/compile.sh` | nvcc build of aimbot / aimbot_ballistic / aimbot_sliding → `bin/` (run on the Jetson) |
| `scripts/convert.sh` | Batch ONNX → TensorRT engine conversion |
| `scripts/setup_mouse.sh` | USB Gadget config, creates `/dev/hidg0` |
| `scripts/game/*.sh` | Per-game launch scripts (relative paths; screenshot-collection switch `CAPTURE`; auto write-back of calibration values `S_EST`/`L_EST`) |
| `arena/` | Pure-Python control-law simulation evaluator (neutral simulator + 8 laws + standard battery); see dedicated section |

Linking note: all three binaries need `-lopencv_video` (calibration uses `phaseCorrelate`); `aimbot` additionally needs `-lopencv_imgcodecs` (collection `imwrite`). The alternative laws have no collection and don't need imgcodecs.

## Parameters

**aimbot** (missing `-s`/`-l`/`-S` silently fall back to defaults; other missing args prompt interactively):

```
-m model  -c class  -t confidence  -y height offset  -d capture card (Hagibis/Asus or /dev/videoN)
-f framerate (120/60)  -x speed cap px/s  -s initial s  -l initial L
-S write-back script path  -k trigger key (fire/ads/both)  -v preview
```

Note: the ffpi bandwidth is derived automatically from the calibrated `L`; **there are no hand-tuning parameters**. Structural parameters (PM/ζ/FF_GAIN_VAL/FF_I_GATE/over-compensation) are header constants in `src/aimbot.cu`, see "Tuning".

**Collection options** (enabled with `-o`, otherwise pure aimbot):

```
-o output dir (auto-creates fire/ det/ auto/)  -F fire interval ms  -A timed interval s  -C cooldown ms  -q JPEG quality
```

**Alternative laws** (`aimbot_ballistic` / `aimbot_sliding`): same interface minus the collection options; bandwidth likewise derived from L — no hand-tuning parameters.

**Calibration**: aim at a static background with texture, hold both side keys for 5 s. Start = draw a square; success = nod; failure = shake. With `-S`, `S_EST=`/`L_EST=` are written back into the script automatically (atomic rename).

## Architecture

### State estimation

| State | Method | Source | Runtime |
|---|---|---|---|
| Position/velocity | alpha-beta (gains normalized by measured dt: α=PRED_ALPHA0·dt/DT0, β=PRED_BETA0·dt/DT0; prediction step subtracts own control action) | first detection | every frame |
| Sensitivity s | least-squares calibration (coarse 8ms + fine 2ms sweep) | hip-fire calibration | constant |
| Delay L | phase-correlation delay sweep (same two rounds) | hip-fire calibration | constant |

### Control law (ffpi, `src/aimbot.cu`)

```
Predictor (Smith, dt-normalized): α=min(.9, PRED_ALPHA0·dt/DT0), β=min(.6, PRED_BETA0·dt/DT0)
  Lc = L̂·PRED_L_COMP                                   // over-compensation; favors the under-compensated side (the dangerous one)
  ê = f + fvx·(age+Lc) − s·Σcounts(in flight)          // delay-removed error
Convergence bandwidth: wn = (90°−PM)π/180 / L̂   (PM=60°, no hand tuning, auto-scales with L)
  Kp = 2ζ·wn, Ki = wn²   (ζ=1 critical damping, no overshoot)
  gate = FF_I_GATE/(FF_I_GATE+|ê|)                     // settled-region gate / I distance decay
  v = clamp(Kp·ê + Ki·∫err·gate + FF_GAIN_VAL·gate·v̂_target, ±vmax)   // PI + type-2 velocity feedforward
Quantization: rem += v·h/s; counts = clamp(trunc(rem), ±120); rem −= counts
```

The **P term** `Kp·ê` is the fast channel: flicks and instant corrections. The **I term** `Ki·∫err` is the slow channel: it removes the steady-state trail behind constant-velocity targets; the integrator is a low-pass, so zero-mean periodic disturbances like recoil cannot accumulate → rejected as a side effect. **No D term**: D amplifies high frequencies and would feed recoil noise back into the commands.

**Type-2 velocity feedforward**: `FF_GAIN_VAL·gate·v̂_target`. `FF_GAIN_VAL=1` is the **exact open-loop command** for zero trail on constant-velocity targets (the plant is an integrator; derived from the plant model, not tuned). `gate` gates the integrator and the feedforward into the settled region together (no buildup during a flick → no overshoot on static targets; buildup after arrival → zero trail on movers). Estimator noise carried by the feedforward is smoothed by the velocity gain `PRED_BETA0`; no extra filter.

**No hand tuning & anti-windup**: the bandwidth `wn` depends only on the calibrated `L` (a deliberately low bandwidth buys delay margin; tracking speed is recovered by the feedforward instead of raising wn). Anti-windup = conditional integration (freeze the integrator when the output is saturated and the error still pushes toward saturation) + integrator clamp `±FF_I_FRAC·vmax/Ki`, preventing windup overshoot on long flicks.

### Calibration

- Half-resolution 3×3 block phase correlation (full resolution would drop to ~30fps).
- Calibration framerate ≠ usage framerate does not hurt accuracy (measured dt and real timestamps are used).
- The hip-fire calibrated s is a practical upper bound (scopes only lower it), so the initial value is inherently safe.

## Invariants

1. **Gains are normalized by measured dt**; framerate changes don't change the feel.
2. **Never assume 1 count = 1 px**; everything is converted through s.
3. **`g_counts` records exactly the counts the game actually received** (mouse + aimbot + calibration); filter compensation / in-flight correction / calibration all depend on it.
4. **Jumps beyond `TRACK_JUMP_GATE` reset the filter**; no patch-style clamps.
5. Calibration sampling (phase correlation) **does not depend on AI detection**; the two couple only through `s_est`/`l_est`.
6. The calibration state machine is driven by the 500Hz mouse thread; the AI thread only responds to the three atomics `g_calib_collect`/`g_calib_request`/`g_calib_done`.

## Tuning

**The main aimbot (ffpi) needs no hand tuning**: after calibrating `s,L`, `wn` scales with `L` automatically. Structural parameters are header constants in `src/aimbot.cu`:

| Constant | Default | Meaning | On-device adjustment |
|---|---|---|---|
| `FF_PM_DEG` | 60 | Phase margin (wn=(90−PM)π/180/L) | Mismatch oscillation → raise (lower wn: stabler, slower) |
| `FF_ZETA` | 1.0 | Convergence damping ratio (critical) | Overshoot → raise; too slow → lower (<0.7 overshoots) |
| `FF_GAIN_VAL` | 1.0 | Velocity feedforward gain (type-2 exact value) | Fixed by principle; normally don't touch |
| `FF_I_GATE` | 8.0 | Settled-region gate / I distance-decay scale (px) | Tracking trail → raise; flick overshoot → lower |
| `FF_I_FRAC` | 1.0 | Integrator clamp (×vmax/Ki) | Windup overshoot → lower |
| `PRED_ALPHA0/BETA0` | 0.50/0.04 | Filter position/velocity gains @120fps | Model jitter → lower ALPHA0; **real device much noisier → lower BETA0 first** (FF noise goes through it) |
| `PRED_L_COMP` | 1.10 | Smith over-compensation factor | Calibrated L too low (dangerous) → keep >1; too high → 1.0 |

On-device workflow: ① calibrate s,L (L too low is the dangerous direction). ② If real-device noise is far above arena's 0.5px: **lower `PRED_BETA0` first** — don't rush to add filters (that becomes hidden control tuning). ③ Mismatch oscillation → raise `FF_PM_DEG` (lower wn) or raise `FF_ZETA`. ④ After changing any estimator/compensation constant, rerun the wide-delay sweep of `arena.integrate ff_pi` to confirm no divergence. `FOV_RADIUS`, `KEEP_ALIVE_MS` and the `CalibSeg` trajectory segments are also in the header constants area.

For alternative-law (ballistic/sliding) tuning, see each `.cu`'s header constants and the "arena control-law simulation → header constants" table.

Collection uses raw NV12 (not MJPEG): NV12 is the only format both the Hagibis and the ASUS CU4K30 support at 1080p120, taking ~3Gbps of USB3 bandwidth; don't put two cards on the same USB controller. Preprocessing runs on the GPU (CUDA kernel BGR→RGB CHW) and is no longer the framerate bottleneck.

## arena control-law simulation

`arena/` is a **neutral pure-Python simulator** for fair evaluation/comparison/tuning of control laws. All control-law conclusions stand on its measurements. This machine has no TRT/GStreamer and can't compile .cu, so all control-law exploration happens here; the winner is then ported into `src/*.cu`.

### Design principles (important)

- **arena simulates only "plant + sensor"; it contains no estimation/prediction/control logic.** Smith predictors, alpha-beta, Kalman, MPC internal models, etc. are implementation details of a law and don't belong in arena. Swapping laws = swapping one object; any method can be compared fairly.
- **Minimal interface** (`arena/laws/base.py`):
  - Observation arena→law: every control tick receives the latest frame `Observation(t, dx, dy, new)`. `dx,dy` = target−crosshair (px), reflecting the world at `t − L_true`, noise included; `new=False` means no new detection since the previous frame.
  - Input law→arena: `step(t, obs) -> (cx, cy)` integer counts. The law keeps its own detection and command history and does its own estimation/prediction/quantization.
- **Pure timestamp-driven**: sensing uses the real `L_true`; the `L` a law believes internally is its own business (`cfg.L`). Testing delay mismatch is just setting the two differently — arena supports it natively.
- **Observation cadence ≠ control cadence**: detections are published at framerate (120/60fps); control runs at 500Hz (2ms); the two are modeled separately (~4 ticks per frame).
- **Faithful plant/sensor**: pure delay (a frame reflects the world at t−L; target and crosshair are both sampled at that moment), framerate/control rate, command quantization and clamping (±120 counts), optional detection noise, near-instant crosshair response to commands (delay only on the observation side).
- **Self-test**: a conservative Smith+PI baseline law (`laws/reference.py`) with known behavior (stable convergence, visible ramp-up) validates the arena — if it fails to reproduce these behaviors, fix arena first. Currently passing.

### Files & running

```
arena/
├── core.py        neutral simulator (plant+sensor, Observation/LawConfig/ArenaConfig)
├── scenarios.py   target motion (static/const-vel/const-accel/random maneuver/relock jump) + standard suite
├── metrics.py     metrics from ground truth (settle time/overshoot/RMSE/in-band fraction/divergence)
├── runner.py      runs law × scenario, composite score, leaderboard
├── eval.py        standard battery: multi-scenario + delay-mismatch sweep + 60/120fps
├── integrate.py   integration: all-law leaderboard + relock + wide-delay sweep + sensitivity mismatch
├── selftest.py    reference-law self-test
├── AUTHORING.md   law author guide (interface/plant ground truth/evaluation method)
└── laws/          control laws (base interface + registry; one file per law, @register)
    ├── reference.py  Smith+PI baseline (self-test)
    ├── ff_pi.py      pole-placement PI + type-2 velocity feedforward (chosen as the main aimbot)
    ├── ballistic.py  open-loop ballistic flick + critically damped convergence (alternative)
    ├── sliding.py    boundary-layer sliding mode + ballistic flick (alternative, most robust)
    ├── pi_pm.py      pole-placement PI + PM cap
    ├── kalman_pi.py  Kalman predictor + PM-PI
    ├── smith_filt.py filtered Smith predictor
    └── mpc.py        model predictive control (rate/quantization constraints)
```

Dependencies: stdlib + numpy (Kalman/MPC) + scipy (DARE solve for MPC); see `requirements.txt`. **This machine (Windows dev mirror) must use a venv; `--break-system-packages` is forbidden**: `python3.12 -m venv .venv` → `.venv\Scripts\python.exe -m pip install -r requirements.txt`. `.venv/` is not committed (gitignored); after cloning, rebuild with the commands above. All arena commands use the venv interpreter (Windows: `.venv\Scripts\python.exe`; Linux: `.venv/bin/python`). The Jetson doesn't need arena.

```bash
.venv\Scripts\python.exe -m arena.selftest            # reference-law self-test (validates arena alignment)
.venv\Scripts\python.exe -m arena.eval ff_pi          # single-law standard battery: matched L=50 + mismatch sweep {30..70} + 60/120fps
.venv\Scripts\python.exe -m arena.integrate           # all-law integrated leaderboard + relock + wide delay {20..80} + s mismatch
.venv\Scripts\python.exe -m arena.integrate ff_pi mpc # run only the given laws
```

**Adding a new law**: create a file in `laws/`, subclass `Law`, `@register("name")`, implement `reset(cfg)`/`step(t,obs)->(cx,cy)`, and add a try-import line in `laws/__init__.py`. See `arena/AUTHORING.md`.
**Adding a new scenario**: write a `Target` subclass + `Scenario` in `scenarios.py` and add it to `standard_suite()` (all laws are then evaluated on the same scenario automatically).
**Adding a new metric**: add a function in `metrics.py`, aggregate in `runner.py`.

### Leaderboard results (`arena.integrate`, all laws re-tuned from principles; lower is better)

| law | OVERALL | matched | worst mismatch | relock | fps delta | mismatch divergence boundary | compute |
|---|---|---|---|---|---|---|---|
| mpc | 144.8 | 160.3 | **113.4** | 444 | 4.0% | L20 **and** L80 | **heavy (QP/tick)** |
| **ff_pi (main aimbot)** | 153.2 | 171.3 | 122.9 | 483 | 3.0% | **no divergence** (L20–80, s0.7–1.3) | light |
| **ballistic (alternative)** | 157.8 | 173.5 | 142.0 | 437 | **0.0%** | L80 | light |
| kalman_pi | 163.4 | 189.9 | 129.4 | 477 | 1.8% | L80 | medium |
| pi_pm | 164.9 | 199.1 | 128.0 | 483 | 0.6% | no divergence | light |
| **sliding (alternative)** | 174.4 | 213.5 | 123.9 | 619 | 2.9% | **no divergence + flattest profile** | light |
| smith_filt | 180.0 | 182.9 | 161.2 | 198 | 4.0% | L80 + s0.7 | light |
| reference | 209.7 | 235.8 | 173.9 | 771 | 2.4% | L80 | light |

### The chosen law (ffpi → `src/aimbot.cu`)

**Core constraint (low patch-smell / generalization first)**: reject parameters "tuned by trial that cannot be explained from principle" (games change, and the tests don't run in-game). Therefore:

- **The convergence bandwidth `wn` is always derived from the calibrated delay `L`**: `wn = (90°−PM)·π/180 / L`, PM=60° (a dimensionless design choice), auto-scaling with `L`; **no hardcoded tuned constants**. At L=50, wn≈0.01047 rad/ms.
- **The type-2 velocity feedforward `FF_GAIN_VAL=1`** comes from the plant model (integrator): it is the exact open-loop command for zero trail on constant-velocity targets, not a tuning knob; the deliberately low bandwidth buys delay margin, and tracking speed is recovered by the feedforward.
- **Damping ratio ζ and phase margin PM are dimensionless design choices** (ffpi/ballistic ζ=1 critical damping; sliding ζ=0.5; PM=60°).
- The truly free knobs are few, and each is explainable from principle; empirical ones are explicitly labeled in each law's docstring.
- **Accept giving up some arena score (speed) in exchange for generalization.**

**Why ffpi is the main aimbot**: it is the **only fast law with zero divergence across the full delay/sensitivity sweep (L20–80, s0.7–1.3)** — flick ~377ms, overshoot ~3.5px, balanced and deployable. Cost: mid-pack maneuver tracking (the feedforward is gated while not settled).

**The two alternatives** (each a Pareto point; estimator/quantization/counts match their arena laws line for line):

- **`src/aimbot_ballistic.cu` (fastest + best tracking)**: open-loop ballistic flick (brake point = Vmax·L) + ζ=1 critically damped convergence. Among the fastest flicks (~361ms) + best maneuver tracking (~19.6px) + best frame-rate independence (0.0%). Cost: highest worst-case mismatch (142); diverges at L80 (+30ms).
- **`src/aimbot_sliding.cu` (most robust)**: boundary-layer sliding mode (convergence gains set from the **full delay** at PM=60 as a floor, not relying on exact Smith cancellation) + ballistic flick. Flattest mismatch profile (range 14); **zero divergence across wide delay L20–80 and sensitivity s0.7–1.3**. Cost: slowest matched speed (flick ~563ms).

**Candidates not shipped**: `mpc` is strongest on paper (first in OVERALL/worst-mismatch/overshoot/maneuver in-band), but it solves a QP per tick — unverified against Jetson 500Hz embedded compute — and has the narrowest delay band (diverges at both L20 and L80); kept in arena for future needs. `pi_pm` is subsumed by `ff_pi` (ff_pi = pi_pm + principled feedforward) and is not shipped separately. `kalman_pi`/`smith_filt` are covered by the above on the Pareto front.

**Header constants** (constants area of each `.cu`; principled rationale in the docstrings of the corresponding `arena/laws/*.py`):

| law | key constants | default | source |
|---|---|---|---|
| shared | `wn=(90−PM)π/180/L` | PM=60 | principle: delay phase margin, auto-scales with L |
| ff_pi | `FF_GAIN_VAL` / `FF_ZETA` / `FF_I_GATE` | 1.0 / 1.0 / 8.0px | principle (type-2 exact FF) / principle (critical damping) / empirical (settled-region gate) |
| ff_pi | `PRED_ALPHA0/BETA0/L_COMP` | 0.50/0.04/1.10 | estimator (dt-normalized) / estimator / Smith over-comp |
| ballistic | `BALL_BRAKE_FACTOR` / `BALL_ZETA` | 1.0 / 1.0 | physics (brake point = Vmax·L) / principle (critical damping) |
| ballistic | `BALL_BOUND_FRAC` / `BALL_I_GATE_FRAC` / `BALL_VMAX_FRAC` | 0.20/0.20/0.95 | empirical (scaling, blend band) / empirical (scaling) / empirical (quantization margin) |
| ballistic | `PRED_ALPHA0/BETA0/L_COMP` | 0.30/0.08/1.00 | estimator (dt-normalized) / estimator / no over-comp |
| sliding | `SLID_ZETA` / `SLID_BRAKE_FACTOR` | 0.5 / 1.0 | principle (ζ=0.5→Ki=wn², integral corner=wn) / physics (brake=min stable layer) |
| sliding | `SLID_BLEND_FRAC` / `SLID_I_GATE_FRAC` / `SLID_VMAX_FRAC` | 0.30/0.25/0.95 | empirical (handoff smoothing) / empirical (anti-windup) / empirical (quantization margin) |
| sliding | `PRED_ALPHA0/BETA0/L_COMP` | 0.40/0.18/1.00 | estimator (**fixed gains, no dt normalization**) / estimator / no over-comp |

### Known limitations

- arena's default 0.5px noise is optimistic; on a noisier real device fast laws converge slower — lower `PRED_BETA0` first (see Tuning); in extreme cases consider the sliding approach (more robust, slower).
- The constant-velocity (CV) predictor cannot predict acceleration: constant-accel targets have an a/Ki steady-state lag, removed slowly by the I term (maneuver RMSE ~20px is mostly the delay lower bound, not a law flaw).
- mpc solves a QP per tick; 500Hz embedded compute is unverified (feasible in arena); shipping it would need explicit MPC or a lower solve rate.
- Every law diverges under extreme mismatch (|L_true−L̂|>~30ms or s error >~40%) — beyond what calibration should ever produce; rely on calibration, not on the law toughing it out.

### How to rerun & extend

1. `.venv\Scripts\python.exe -m arena.selftest` to confirm arena alignment.
2. After changing/adding a law: `arena.eval <law>` for the standard battery; `arena.integrate <law>` for the integration (mismatch/relock/framerate included).
3. Once a better law is found, port its `step()` logic line for line into the corresponding `.cu` under `src/` (main law → the control section of `src/aimbot.cu`; or a new alternative-law file). Units/quantization/counts/estimator must match the winning law exactly.
4. Tuning stands on arena measurements, and **delay mismatch must be tested** (the wide-delay sweep in `integrate.py`); a scheme that diverges under mismatch loses, no matter how fast.
