# AGENTS.md

## Overview

AI visual aimbot (mouse pass-through). Target hardware: **NVIDIA Jetson Orin** (aarch64, JetPack / TensorRT 10 / CUDA / OpenCV4 / GStreamer).

```
Capture card (UVC 1080p NV12) → GStreamer nvvidconv → CUDA preprocess → TensorRT YOLO
→ alpha-beta tracking → control law (pole-placement PI + type-2 velocity feedforward)
→ merged with the real mouse → USB Gadget (/dev/hidg0, generic HID mouse) → game
```

No hand-tuned gains: a bilateral side-key trigger runs auto-calibration, estimating sensitivity s (px/count) and loop delay L (ms) online. Adapts to PC / PS5 / 60fps / 120fps.

**Control law**: the single program `src/aimbot.cu` uses **ff_pi_acc** (pole-placement PI + type-2 velocity feedforward with direction-contradiction CUSUM velocity reset and detection-gap FF decay, plus an innovation-mean â channel that removes the α-β structural lag on accelerating targets) — the convergence bandwidth `wn` is derived from the calibrated delay `L` via phase margin (`wn=(90°−PM)π/180/L`, PM=50°), **no hand-tuned magic numbers**, good generalization. The same binary has **optional training-data collection** (enabled with `-o`, otherwise pure aimbot). All control-law exploration/comparison/tuning happens in the pure-Python `arena/` simulation (this machine cannot compile .cu).

## Author's requirements (binding)

The author's standing requirements for this repository. They are not style preferences: a
change that violates one is rejected even when its measured benefit is real. Read this
section before editing anything.

**1. The library reads as if it had been written in one go.** No patch-style narration, no
version comparison, no timeline or provenance voice — not in this file, not in docstrings,
not in code comments. Forbidden: "patch 1 …", "this version improves on the previous one",
"recently added", "the old implementation was …", "a debugging round on <date> found …".
Required instead: design rationale and measured numbers stated as standing facts ("the cap
is a physical requirement because …"), and rejected paths recorded as conclusions ("a
persistently faster gain is forbidden: …") — never as a history of who changed what when.

> 作者原话: "让整个库看起来'像是一次性写出来的'，意思就是不要有什么补丁1：xxxx patch2：
> xxxx 这个版本相比上个版本提升了xxxx之类的话……总之库要是干净的整洁的。"

**2. No magic numbers.** Every constant traces to one of three things: an explicit physical
quantity, a reproducible derivation, or a written-down selection rule. Forbidden: thresholds
produced by tuning with no criterion and no provenance, and compromises found on a single
scenario. The accepted form is already used throughout — `PRED_BETA0=0.03` is labelled
"band-edge margin", `ACC_SNR=10` "the smallest whole value holding the whole delay and s
band bit-exact", `PM=50` "chosen by the mismatch-band test suite", `FF_PM_DEG`/`FF_ZETA`
dimensionless design choices with the reason stated. Corollary, and the reason this matters:
a number no command can reproduce — typically one that came from an experimental file since
deleted — must either gain a reproduction path or be removed, or be rewritten as a
qualitative statement without it.

> 作者原话: "不能有'魔法数字'。……没有什么经过无数次微调得到的那种魔法数字。"

**3. The result is what counts, and method-level change is welcome.** Anything that measures
better is acceptable, including structural change — replacing the estimator or the law, or
adding a mechanism of the CUSUM kind. A rewrite is not required; but knob-tuning alone is the
weakest available option, not the target. What is *not* acceptable is an unsourced constant
added for feel (requirement 2).

> 作者原话: "我不管你怎么做，我只要最后结果好。"

**4. The evaluation groups are the contract.** A change ships only after passing all of:
`arena.eval` (matched composite, worst mismatch over L30–70, and the 60/120 fps delta must
not rise), `arena.integrate` (no divergence anywhere in the wide delay band L20–80 or s
0.7–1.3 — divergence loses outright, however fast the scheme), `arena.fps_eval` (neither the
clean nor the flaky `drop_p=0.12` variant may get worse; the per-phase air / dash / slide
tables are the focus), and `arena.selftest`. Experiment in `arena/laws/_wip_*.py`; a shared
evaluation group or the shipped law is never edited to accommodate an experiment.

**5. Mismatch and flakiness are tested for anything claiming to be faster.** Any "faster /
more aggressive" mechanism reports its behaviour across the mismatch band *and* under flaky
detection. A scheme that wins at matched and breaks in either place is not a candidate.

**6. Reproduce before believing; report numbers, not intentions.** Re-derive the baseline of
whatever is about to change before touching it, and do not accept a number in this file as
fact without re-running its command — the figures here are measurements with reproduction
paths, not axioms. After a change, re-run the full set above and list the files touched.
Keep what was measured, what is inferred from documentation, and what is still unverified
clearly apart; "should be better" is not a result — give a number or write "unverified".

**7. Documentation and code move together.** A changed constant, default, CLI flag or module
list updates its prose, its tables and its docstring defaults in the same edit.

**8. `src/aimbot.cu` is edited here, verified on the Jetson.** This machine cannot compile or
run it; every `.cu` change needs a rebuild and on-device validation on the Jetson and is
never reported as verified from here.

## Environment

- This folder is a **Windows development mirror**; code editing only.
- Compile and run happen on the Jetson. Deployment directory e.g. `/mnt/TF/aimbot/`; layout (dev mirror matches deployment):

```
/mnt/TF/aimbot/
├── src/         aimbot.cu (ff_pi_acc + optional collection)
├── scripts/     compile.sh / convert.sh / setup_mouse.sh
│   └── game/    launcher template (template.sh.example → 复制成 <game>.sh 使用)
├── bin/         build output (aimbot)
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
| `src/aimbot.cu` | **The program**: full aimbot + optional training-data collection. Control law ff_pi_acc (pole-placement PI + type-2 velocity feedforward with direction-contradiction CUSUM velocity reset and detection-gap FF decay; gated â acceleration-bias compensation; wn derived from L, no hand tuning). Without `-o` it is pure aimbot |
| `scripts/compile.sh` | nvcc build of aimbot → `bin/` (run on the Jetson) |
| `scripts/convert.sh` | Batch ONNX → TensorRT engine conversion |
| `scripts/setup_mouse.sh` | USB Gadget config, creates `/dev/hidg0` |
| `scripts/game/template.sh.example` | Per-game launcher template — copy to `<game>.sh` (`.example` keeps the webui from listing it as a launchable profile). Relative paths; mouse-takeover switch `AIM_ENABLED` and screenshot-collection switches (`CAPTURE` master + per-source `CAP_FIRE`/`CAP_DET`/`CAP_AUTO`); auto write-back of calibration values `S_EST`/`L_EST`; the `MAX_SPEED` rule and its derivation live in the file |
| `arena/` | Pure-Python control-law simulation evaluator (neutral simulator + 10 laws + standard + FPS test suites); see dedicated section |

Linking note: `aimbot` needs `-lopencv_video` (calibration uses `phaseCorrelate`) and `-lopencv_imgcodecs` (collection `imwrite`).

## Parameters

**aimbot** (missing `-s`/`-l`/`-S` silently fall back to defaults; other missing args prompt interactively):

```
-m model  -c class  -t confidence  -y height offset  -d capture card (Hagibis/Asus or /dev/videoN)
-f framerate (120/60)  -x speed cap px/s  -s initial s  -l initial L  -r FOV radius px (default 150)
-a mouse takeover (default y; n = pure pass-through: no injected motion, detection/collection keep running)
-S write-back script path  -k trigger key (fire/ads/both)  -v preview
```

Hot params: the binary opens a localhost-only UDP control channel (127.0.0.1:47700, `key=value;...`); the webui pushes whitelisted params (`t`/`y`/`x`/`fov`/`k`/`aim`/`cap_fire`/`cap_det`/`cap_auto`, clamped firmware-side) into the running process without restart — protocol in `webui/README.md`. Structural constants stay compile-time.

Note: the ff_pi_acc bandwidth is derived automatically from the calibrated `L`; **there are no hand-tuning parameters**. Structural parameters (PM/ζ/FF_GAIN_VAL/FF_I_GATE/over-compensation and the â-channel constants) are header constants in `src/aimbot.cu`, see "Tuning".

**Collection options** (enabled with `-o`, otherwise pure aimbot; the three sources are the `-e` list, each also a hot switch):

```
-o output dir (auto-creates fire/ det/ auto/)  -e enabled sources fire,det,auto (default all)  -F fire interval ms  -A timed interval s  -C cooldown ms  -q JPEG quality
```

**Calibration**: aim at a static background with texture, hold both side keys for 5 s. Start = draw a square; success = nod; failure = shake. With `-S`, `S_EST=`/`L_EST=` are written back into the script automatically (atomic rename).

## Architecture

### State estimation

| State | Method | Source | Runtime |
|---|---|---|---|
| Position/velocity | alpha-beta (gains normalized by measured dt: α=PRED_ALPHA0·dt/DT0, β=PRED_BETA0·dt/DT0; prediction step subtracts own control action) | first detection | every frame |
| Sensitivity s | least-squares calibration (coarse 8ms + fine 2ms sweep) | hip-fire calibration | constant |
| Delay L | phase-correlation delay sweep (same two rounds) | hip-fire calibration | constant |

### Control law (ff_pi_acc, `src/aimbot.cu`)

```
Predictor (Smith, dt-normalized): α=min(.9, PRED_ALPHA0·dt/DT0), β=min(.6, PRED_BETA0·dt/DT0)
  Lc = L̂·PRED_L_COMP                                   // over-compensation; favors the under-compensated side (the dangerous one)
  ε = â·T·(α/β − ½),  W = age+Lc                        // α-β structural velocity lag on accelerating targets
  ê = f + (v̂+ε)·W + ½â·W² − s·Σcounts(in flight)       // delay-removed error
Convergence bandwidth: wn = (90°−PM)π/180 / L̂   (PM=50°, no hand tuning, auto-scales with L)
  Kp = 2ζ·wn, Ki = wn²   (ζ=1 critical damping, no overshoot)
  gate = FF_I_GATE/(FF_I_GATE+|ê|)                     // settled-region gate / I distance decay
  Direction-contradiction CUSUM (per axis, σ-normalized, σ online-estimated):
    S = max(0, S + clip(∓sign(v̂)·in/σ, 0, C) − K);  alarm S > H → reset that axis v̂=0 (position kept)
  Acceleration channel â = ȳ·β/T²                      // innovation-mean inversion (exact at any framerate)
    ȳ = gated EMA of the cleaned innovation             // three gates: rebuild suppression (after CUSUM
    floor = ACC_SNR·σ_noise·√(ρ/(2−ρ));  |ȳ| ≤ floor → â = 0   // reset/jump), own-acceleration activity gate,
  v = clamp(Kp·ê + Ki·∫err·gate + FF_GAIN_VAL·gate·gap_scale·(v̂+ε), ±vmax)   // significance floor
    gap_scale = 1 − clamp((age−frame_dt)/L̂, 0, 1)       // detection gap: withdraw the open-loop term on the L timescale
Quantization: rem += v·h/s; counts = clamp(trunc(rem), ±120); rem −= counts
```

Without sustained real acceleration the three gates keep â ≡ 0 and the command stream is identical to plain ff_pi (bit-exact in arena).

The **P term** `Kp·ê` is the fast channel: flicks and instant corrections. The **I term** `Ki·∫err` is the slow channel: it removes the steady-state trail behind constant-velocity targets; the integrator is a low-pass, so zero-mean periodic disturbances like recoil cannot accumulate → rejected as a side effect. **No D term**: D amplifies high frequencies and would feed recoil noise back into the commands.

**Direction-contradiction CUSUM → velocity reset (归零重拉)**: every large overshoot on moving targets traces to one root — the target model breaks (stop/ADAD reversal) and v̂ becomes a ghost that simultaneously pushes FF the wrong way AND masks the error in the Smith prediction (the loop can't see its own trail → mushy pull-back). When the CUSUM (Page sequential change test, σ-normalized, σ online-estimated — **no absolute-px constants**; K/C/H are σ multiples so the gate auto-widens with device noise) detects innovation sustained against v̂, that axis's velocity is reset to zero while the position estimate is kept: the loop re-runs the step response (the best-tuned behavior: settle 277ms / 3.1px) with no ghost in the projection (P sees the full true error → sharp pull-back) and FF rebuilding from zero in the correct direction (no wrong-way push, no re-engagement kick). Only contradiction-direction innovation accumulates, so the chase after a reset cannot re-trigger (innovation then agrees with the rebuilding v̂); the per-frame increment cap (3σ) rejects single-frame kicks (recoil).

**No hand tuning & anti-windup**: the bandwidth `wn` depends only on the calibrated `L` (tracking speed is recovered by the feedforward; PM=50 is the fastest design point that passes the full mismatch band L20–80 — test-suite-chosen, not feel-chosen). Anti-windup = conditional integration (freeze the integrator when the output is saturated and the error still pushes toward saturation) + integrator clamp `±FF_I_FRAC·vmax/Ki`, preventing windup overshoot on long flicks.

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

**The aimbot needs no hand tuning**: after calibrating `s,L`, `wn` scales with `L` automatically. Structural parameters are header constants in `src/aimbot.cu`:

| Constant | Default | Meaning | On-device adjustment |
|---|---|---|---|
| `FF_PM_DEG` | 50 | Phase margin (wn=(90−PM)π/180/L) | Mismatch oscillation → raise (lower wn: stabler, slower); 50 = fastest passing the full delay band |
| `FF_ZETA` | 1.0 | Convergence damping ratio (critical) | Overshoot → raise; too slow → lower (<0.7 overshoots) |
| `FF_GAIN_VAL` | 1.0 | Velocity feedforward gain (type-2 exact value) | Fixed by principle; normally don't touch |
| `FF_I_GATE` | 8.0 | Settled-region gate / I distance-decay scale / withdrawal-weight scale (px) | Tracking trail → raise; flick overshoot → lower |
| `FF_I_FRAC` | 1.0 | Integrator clamp (×vmax/Ki) | Windup overshoot → lower |
| `PRED_ALPHA0/BETA0` | 0.50/0.03 | Filter position/velocity gains @120fps | Model jitter → lower ALPHA0; **real device much noisier → lower BETA0 first** (FF noise goes through it; 0.03 = band-edge margin) |
| `PRED_L_COMP` | 1.10 | Smith over-compensation factor | Calibrated L too low (dangerous) → keep >1; too high → 1.0 |
| `FOV_RADIUS` | 150 px | Default FOV radius — target selection gate AND integrator-start boundary; runtime value overridable via `-r` and hot-param `fov` | Widen: farther targets enter the gate (multi-target grab risk); >~452 px is wasted (capture window diagonal) |

On-device workflow: ① calibrate s,L (L too low is the dangerous direction). ② If real-device noise is far above arena's 0.5px: **lower `PRED_BETA0` first** — don't rush to add filters (that becomes hidden control tuning). ③ Mismatch oscillation → raise `FF_PM_DEG` (lower wn) or raise `FF_ZETA`. ④ After changing any estimator/compensation constant, rerun the wide-delay sweep of `arena.integrate ff_pi_acc` and the FPS behavior test suite `arena.fps_eval ff_pi_acc` to confirm no divergence and no event regression. `FOV_RADIUS`, `KEEP_ALIVE_MS` and the `CalibSeg` trajectory segments are also in the header constants area.

Collection uses raw NV12 (not MJPEG): NV12 is the only format both the Hagibis and the ASUS CU4K30 support at 1080p120, taking ~3Gbps of USB3 bandwidth; don't put two cards on the same USB controller. Preprocessing runs on the GPU (CUDA kernel BGR→RGB CHW) and does not bound the framerate.

## arena control-law simulation

`arena/` is a **neutral pure-Python simulator** for fair evaluation/comparison/tuning of control laws. All control-law conclusions stand on its measurements. This machine has no TRT/GStreamer and can't compile .cu, so all control-law exploration happens here; the winner is then ported into `src/aimbot.cu`.

### Design principles (important)

- **arena simulates only "plant + sensor"; it contains no estimation/prediction/control logic.** Smith predictors, alpha-beta, Kalman, MPC internal models, etc. are implementation details of a law and don't belong in arena. Swapping laws = swapping one object; any method can be compared fairly.
- **Minimal interface** (`arena/laws/base.py`):
  - Observation arena→law: every control tick receives the latest frame `Observation(t, dx, dy, new)`. `dx,dy` = target−crosshair (px), reflecting the world at `t − L_true`, noise included; `new=False` means no new detection since the previous frame.
  - Input law→arena: `step(t, obs) -> (cx, cy)` integer counts. The law keeps its own detection and command history and does its own estimation/prediction/quantization.
- **Pure timestamp-driven**: sensing uses the real `L_true`; the `L` a law believes internally is its own business (`cfg.L`). Testing delay mismatch is just setting the two differently — arena supports it natively.
- **Observation cadence ≠ control cadence**: detections are published at framerate (120/60fps); control runs at 500Hz (2ms); the two are modeled separately (~4 ticks per frame).
- **Faithful plant/sensor**: pure delay (a frame reflects the world at t−L; target and crosshair are both sampled at that moment), framerate/control rate, command quantization and clamping (±120 counts), optional detection noise, per-frame detection dropout (`drop_p`), near-instant crosshair response to commands (delay only on the observation side).
- **Self-test**: a conservative Smith+PI baseline law (`laws/reference.py`) with known behavior (stable convergence, visible ramp-up) validates the arena — if it fails to reproduce these behaviors, fix arena first. Currently passing.

### Files & running

```
arena/
├── core.py        neutral simulator (plant+sensor, Observation/LawConfig/ArenaConfig; drop_p = per-frame detection dropout)
├── scenarios.py   target motion (static/const-vel/const-accel/random maneuver/relock jump) + standard suite
├── fps.py         FPS behavior library in screen space (stop/jump-land/wall-bounce/strafe-switch/jiggle/bhop/slide/turn/approach/dash + near-range jumps at 5m/3m by 1/d scaling) + fps_suite
├── metrics.py     metrics from ground truth (settle time/overshoot/RMSE/in-band fraction/divergence) + event_metrics (post-event overshoot/recovery) + phase_metrics (per-declared-phase rmse/mean/p95/max + on_body at the distance-scaled torso half-width) + pooled_phase
├── runner.py      runs law × scenario, composite score, leaderboard
├── eval.py        standard test suite: multi-scenario + delay-mismatch sweep + 60/120fps
├── fps_eval.py    FPS behavior test suite: fps_suite × {clean, flaky drop_p=0.12}, per-event overshoot/recovery + per-phase tables
├── trace.py       per-tick process tracer: single law × scenario → terminal process summary + per-tick CSV (+PNG if matplotlib present); optional law debug() hook; pure observation
├── diag.py        error attribution: same law ± true delayed velocity → splits error into estimator-limited vs delay/loop/saturation-limited; pure observation
├── integrate.py   integration: all-law leaderboard + relock + wide-delay sweep + sensitivity mismatch
├── selftest.py    reference-law self-test
├── AUTHORING.md   law author guide (interface/plant ground truth/evaluation method)
└── laws/          control laws (base interface + registry + shared CountsHist; one file per law, @register)
    ├── reference.py   Smith+PI baseline (self-test)
    ├── ff_pi_acc.py   ff_pi family + innovation-mean â acceleration-bias compensation (control-law record holder)
    ├── ballistic.py   open-loop ballistic flick + critically damped convergence
    ├── ballistic_ff.py ballistic two-phase + type-2 FF convergence (three-layer ghost-FF protection)
    ├── sliding.py     boundary-layer sliding mode + ballistic flick (robustness-first baseline)
    ├── sliding_obs.py sliding skeleton + type-2 disturbance observer (SNR-gated; full-band zero divergence)
    ├── pi_pm.py       pole-placement PI + PM cap (no FF; full band pass)
    ├── pi_guard.py    pi_pm structure + CUSUM dual speed-memory reset (v̂ and I cleared together)
    ├── kalman_pi.py   Kalman predictor + PM-PI
    ├── smith_filt.py  filtered Smith predictor (fastest settle/relock/event-recovery; fragile under mismatch)
    ├── mpc_osc.py     MPC + innovation-oscillation-signature FF gate (heavy QP/tick)
    ├── imm_pi.py      per-axis 2-model IMM maneuver-adaptive estimator (best matched/FPS, fails the mismatch band)
    └── reseed_pi.py   pole-placement PI + type-2 FF + evidence-gated CUSUM re-seeding
```

Dependencies: stdlib + numpy (Kalman/MPC) + scipy (DARE solve for MPC); see `requirements.txt`. **This machine (Windows dev mirror) must use a venv; `--break-system-packages` is forbidden**: `python3.12 -m venv .venv` → `.venv\Scripts\python.exe -m pip install -r requirements.txt`. `.venv/` is not committed (gitignored); after cloning, rebuild with the commands above. All arena commands use the venv interpreter (Windows: `.venv\Scripts\python.exe`; Linux: `.venv/bin/python`). The Jetson doesn't need arena.

```bash
.venv\Scripts\python.exe -m arena.selftest            # reference-law self-test (validates arena alignment)
.venv\Scripts\python.exe -m arena.eval ff_pi_acc      # single-law standard test suite: matched L=50 + mismatch sweep {30..70} + 60/120fps
.venv\Scripts\python.exe -m arena.integrate           # all-law integrated leaderboard + relock + wide delay {20..80} + s mismatch
.venv\Scripts\python.exe -m arena.integrate ff_pi_acc mpc_osc # run only the given laws
.venv\Scripts\python.exe -m arena.fps_eval            # FPS behavior test suite (default ff_pi_acc + reference)
.venv\Scripts\python.exe -m arena.fps_eval ff_pi_acc ballistic_ff sliding_obs  # chosen laws only
.venv\Scripts\python.exe -m arena.trace ff_pi_acc step_80px [--L-true 30] [--csv out.csv]  # per-tick process trace of one run (debug; see "Process tracing" below)
.venv\Scripts\python.exe -m arena.diag ff_pi_acc [scenario ...]  # error attribution, per scenario/phase (debug; see "Error attribution" below)
.venv\Scripts\python.exe -m arena.diag ff_pi_acc --suite         # same, at standard-suite composite level
```

**Why 2D screen space is the right arena (and not "3D")**: the whole sense-control loop lives in screen pixels (capture → detect → dx,dy → law → counts → crosshair); the 3D game world is just one generator of screen-space trajectories, and the law never sees the world. The FPS behavior library (`fps.py`) therefore models the *screen-space shape* of 3D behaviors: jumps are parabolas on screen-y only (world-vertical motion projects to screen-vertical, orthogonal to any strafe heading), strafe heading is a free angle, wall-bounce is a full 2V velocity reversal, jump-landing is a hard y-velocity step. The only unmodeled 3D effect is tan-projection nonlinearity (s varies by sec² across the screen): ~2.4% inside the ±150px FOV circle — negligible; a 3D world+camera+projection Target subclass can be added later without touching core.

**Adding a new law**: create a file in `laws/`, subclass `Law`, `@register("name")`, implement `reset(cfg)`/`step(t,obs)->(cx,cy)`, and add an import line in `laws/__init__.py`. See `arena/AUTHORING.md`.
**Adding a new scenario**: write a `Target` subclass + `Scenario` in `scenarios.py` and add it to `standard_suite()` (all laws are then evaluated on the same scenario automatically).
**Adding a new metric**: add a function in `metrics.py`, aggregate in `runner.py`.

### Process tracing (`arena/trace.py`, debug-only)

Aggregate finals alone do not show where a law breaks; `trace.py` replays **one** law × **one** scenario and shows where it breaks: per-tick CSV (`t/ex/ey/|e|/sent counts/new-frame/obs fields`), a compact terminal summary (band-entry ladder 10/5/3/1px, `event_metrics` event windows, worst-1s window, tail-oscillation verdict), event/auto window export, optional PNG. **Pure observation by construction** (law proxy; core/runner untouched) — the three default test suites are bit-identical with or without it (verified by diff). Optional law-side protocol: a law may implement `debug() -> dict[str, float]`; trace records it per tick as `dbg_*` columns — field names/semantics belong to the law's own docstring, arena never interprets them; `debug()` must be side-effect-free and is never called by eval/integrate/fps_eval. `laws/__init__.py` auto-imports `_wip_*.py` experiment copies (register as `wip_<name>`) so parallel debugging never touches the shared file; a broken WIP file is skipped with a stderr note, never blocks the real laws. Trace outputs live in gitignored `arena/trace_out/`.

### Error attribution: where a law's error actually lives (`arena/diag.py`, debug-only)

Aggregate finals — including the FPS suite's headline `over = peak − median|e| over the 200 ms before the event` — are **relative**. Once a law is already trailing for a long time before an event (the airborne phase of a jump is the canonical case), `pre` is already large, so `over` stays small and the sustained trailing is invisible in both `over` and the scenario-wide RMSE (diluted by the normal ground-phase tracking). Two instruments close that hole:

- **`metrics.phase_metrics`** — scenarios declare sustained-phase windows (`Scenario.phases`; the jump / bhop / wall-bounce / dash / slide entries in `fps_suite` declare `air`/`dash`/`slide`) and report rmse/mean/p95/max plus `on_body` (|e| ≤ `metrics.body_px(dist_m)`, the torso half-width at the scenario's engagement distance). The band must scale with distance: a fixed band systematically understates hits up close (a 0.25 m half-width is 80 px at 3 m, 24 px at 10 m). `fps_eval` prints the phase table and an `[air]` aggregate. `Scenario.dist_m` also carries the **1/d projection scaling** used by the near-range jump entries: screen velocity and screen gravity both ∝ 1/d, so the same world jump at distance d is the 10 m calibration times (10/d), with air time 2·v_z/g and hence arc apex ∝ 1/d.
- **`arena.diag`** — replays the *same* law with only its velocity state replaced by the true velocity at the observation's capture time `t−L_true`: the newest world state its information set could recover, i.e. the honest **causal** bound (using the truth at `t` would be peeking at future maneuvers). The pair of runs splits a scenario's error into the estimator-limited part (`est_share = 1 − oracle/law`) and the residual that delay, loop bandwidth and actuator saturation set.

Shipped law, matched L=50 / 120 fps / noise 0.5 / seeds 1–3:

| scenario / phase | law rmse | oracle rmse | est_share | on_body law → oracle |
|---|---|---|---|---|
| fps_jump_land_stop / air (10 m) | 18.90 | 7.02 | **63%** | 87% → 98% |
| fps_jump_5m / air | 38.02 | 15.61 | **59%** | 87% → 98% |
| fps_jump_3m / air | 68.26 | 39.41 | 42% | 80% → 89% |
| fps_wall_bounce / air | 30.80 | 15.65 | 49% | 68% → 90% |
| fps_bhop / air (3 legs) | 19.0–26.8 | 6.9–13.1 | 51–64% | 78–87% → 91–98% |
| maneuver / all | 19.00 | 10.29 | 46% | 79% → 95% |
| fps_dash / dash | 70.64 | 54.48 | **23%** | 16% → 16% |

The jump / bhop / bounce / maneuver family is **estimator-limited** (42–64% of the error is the α-β velocity estimate; with a perfect one the on-body fraction reaches 89–98%). `fps_dash` is not — its burst (4 × 0.4 = 1.6 px/ms) exceeds the 1.5 px/ms speed cap, so the crosshair is saturated and on-body stays at 16% even with a perfect velocity: a speed-cap problem is not a control-law problem, and no estimator work fixes it. Raising the cap confirms the ordering — `arena.diag ff_pi_acc fps_dash --max-v 3` moves the oracle from 54.5 to 35.4 px and on-body 16% → 43%, while the law itself barely moves (70.6 → 70.0, the estimator cannot use the higher cap); the cap is that phase's first constraint and latency its second. The 3 m jump is mixed, because the cap binds wherever |v_z| exceeds it — which is why its est_share falls to 42%.

At composite level the same substitution gives **matched 114.66 → 54.06 and worst mismatch 125.12 → 87.69** (`arena.diag <law> --suite`): the mismatch band does *not* degrade, so the architecture tolerates a correct velocity estimate. The estimator's remaining error is its own transient behaviour rather than noise — the β leak turns a 40 px *position* step into a 0.14 px/ms ghost velocity that the feedforward projects over the whole Smith horizon, and acquiring a velocity step takes ~1/β ≈ 278 ms, longer than a whole jump.

Constraints for any future estimator work (each measured, each a constraint rather than a preference):

- **A persistently faster gain is forbidden.** `PRED_BETA0` = 0.06 fails the band: at L_true = 30 the step response hunts and never enters the 3 px band (overshoot 24 px). β0 = 0.03 is pinned by loop gain and phase, not by innovation magnitude.
- **Robustifying the β input does not rescue it.** Clipping the velocity update's innovation (M-estimator style) buys a little on the mismatch band but costs matched and framerate consistency; the exact trade depends on where the clip is inserted, and no variant rescues β0 = 0.06, which fails the band exactly as before. The filter already clips the cleaned innovation at `SIG_CLIP_K`, so a second clip above that point is a no-op.
- **Only an evidence-gated one-shot correction can be both fast and safe.** A σ̂-normalized one-sided agreement CUSUM with a one-shot velocity catch-up holds the mismatch band with no divergence — the σ̂ self-widening that makes the reset robust also blinds any σ-normalized gate under mismatch — but as tuned it loses on the composite, because detection dropouts and ordinary maneuvers produce the same sustained same-sign innovation signature as a takeoff. Its amplitude must be the alarm-window mean; taking the last frame's innovation alone is markedly worse on both the matched composite and accel tracking. The residual difficulty is structural rather than a matter of sizing: the window-mean innovation also carries the α-β *position* transient, so a catch-up dimensioned by it over-shoots and costs the airborne on-body fraction.
- **An attention/arming test cannot share the alarm's scale.** A deadband suppressing contradiction accumulation below a σ̂ multiple removes ~91% of all alarms and improves matched and framerate consistency, but it also removes the reset that flushes v̂ at real reversals (a clearly worse strafe-switch event overshoot at a 2σ̂ multiple), and it **self-blinds at the largest maneuvers**: at a 2V wall-bounce the discarded velocity is 0.45 px/ms — unambiguously real — yet classified as noise, because the bounce inflates σ̂ to ≈ 4.4 px so 2σ̂ ≈ 8.8 exceeds |v̂|·dt ≈ 3.8. Magnitude alone cannot be the discriminant either: over a full pass of the standard + FPS suites (baseline law, CUSUM resets that zero a velocity) the discarded velocity is a continuum running from the estimate's own velocity noise floor σ_v up to full target speed, so a magnitude threshold either keeps most false alarms or loses most real ones.
- **The â channel's rebuild suppression is not the airborne bottleneck.** A noise-level v̂ (≈0.008 px/ms) does arm the CUSUM and can hold the rebuild suppression for ~417 ms, but forcing that suppression off changes the 10 m airborne RMSE by 0.02 px (18.90 → 18.88) — the estimate itself is what trails, not the gate that hides it.
- **Velocity-estimate bandwidth is the airborne ceiling, and it is exactly the mismatch margin.** `arena.diag` attributes 42–64% of the airborne RMSE to the velocity state (causal bound 6.99–39.4 px against the law's 18.90–68.26 px), and the entire gap is estimator bandwidth: `FFPiAccLaw(beta0=0.04)` — the first value that breaks the L20 corner, where the step response hunts and never settles (0.035 still passes, with a worse composite) — buys 5.6% (10 m airborne 18.90 → 17.85), `beta0=0.06` buys 17% (15.67) and hunts at L_true = 30, and `imm_pi`, the library's own fast estimator, reaches 10.92 px on the same scenario while failing the same band. Splitting the two places v̂ is consumed (the Smith assembly and the feedforward) does not separate the gain from the instability — a fast channel diverges through either path. The estimator a challenger must supply is therefore not a faster one but one whose error is *uncorrelated with the loop's own motion*: error that correlates with it closes a positive feedback whose gain rises with the mismatch, which is why every bandwidth increase is paid for in margin rather than being free tracking speed.
- **The â channel's gates are not where its headroom lies; its sensor is its own ceiling.** The significance floor forms the noise scale as the second moment of the clipped cleaned innovation minus ȳ², so the signal's own variance is booked as noise — during a 10 m jump the implied noise reads ≈1.3 px against a 0.5 px detection noise — and the floor therefore rises with the very maneuver it is meant to detect. Each gate lifted alone is worth under a pixel; lifting the floor, the rebuild suppression and the CUSUM contradiction gate together still leaves most of the oracle gap, because ȳ is an EMA of a transient-dominated clipped innovation and carries only a fraction of the true acceleration. The contradiction gate is self-blocking by construction — the CUSUM declares a contradiction precisely because the α-β velocity lags, which is what the channel exists to correct — but it is load-bearing elsewhere: removing it trades airborne RMSE for FPS event overshoot and recovery. A challenger has to replace the sensor formulation, not the gates around it.

### Leaderboard results (`arena.integrate`, all laws tuned from principles; lower is better)

| law | OVERALL | matched | worst mismatch | relock | fps delta | mismatch divergence boundary | compute |
|---|---|---|---|---|---|---|---|
| **ff_pi_acc** | **123.6** | **114.7** | 125.1 | 386 | 1.9% | L20–70 + s0.7–1.3 pass; L80 edge settle-fail | light |
| ballistic_ff | 134.9 | 125.8 | 135.0 | **219** | 2.3% | L20–70 + s0.7–1.3 pass; L80 edge settle-fail | light |
| sliding_obs | 138.1 | 160.9 | **111.3** | 427 | **1.0%** | **full band L20–80 + s0.7–1.3 pass** | light |
| mpc_osc | 144.8 | 160.3 | 113.4 | 444 | 4.0% | L80 only (L20 fixed) | **heavy (QP/tick)** |
| ballistic | 157.8 | 173.5 | 142.0 | 437 | **0.0%** | L80 | light |
| pi_guard | 160.2 | 190.7 | 116.7 | 483 | 3.3% | full band L20–80 + s0.7–1.3 pass | light |
| reseed_pi | 161.5 | 146.6 | 125.6 | 386 | 12.7%* | L20–70 + s0.7–1.3 pass; L80 edge settle-fail | light |
| kalman_pi | 163.4 | 189.9 | 129.4 | 477 | 1.8% | L80 | medium |
| pi_pm | 164.9 | 199.1 | 128.0 | 483 | 0.6% | no divergence | light |
| sliding | 174.4 | 213.5 | 123.9 | 619 | 2.9% | **no divergence + flattest profile** | light |
| smith_filt | 180.0 | 182.9 | 161.2 | **198** | 4.0% | L80 + s0.7 | light |
| reference | 209.7 | 235.8 | 173.9 | 771 | 2.4% | L80 | light |
| imm_pi | inf | **104.0** | inf | 389 | 10.3% | L20–40 & L70–80 + s0.7 settle-fail | light |

\* reseed_pi's fps-delta is a denominator effect: on the 60fps side step×2/const_vel/accel are bit-identical to 120fps behavior and maneuver moves +0.5px; the improved 120fps composite shrinks the ratio. The FPS behavior tests (`arena.fps_eval`) are the meaningful law-vs-law comparison. Superseded prototypes (ff_pi, mpc) were removed from the library when a successor dominated them on every test-suite cell; they live in git history.

### The shipped law (ff_pi_acc → `src/aimbot.cu`)

**Core constraint (low patch-smell / generalization first)**: reject parameters "tuned by trial that cannot be explained from principle" (games change, and the tests don't run in-game). Therefore:

- **The convergence bandwidth `wn` is always derived from the calibrated delay `L`**: `wn = (90°−PM)·π/180 / L`, PM=50° (a dimensionless design choice — the fastest point whose full delay band L20–80 passes the test suite), auto-scaling with `L`; **no hardcoded tuned constants**. At L=50, wn≈0.01396 rad/ms.
- **The type-2 velocity feedforward `FF_GAIN_VAL=1`** comes from the plant model (integrator): it is the exact open-loop command for zero trail on constant-velocity targets, not a tuning knob; when the target model breaks the FF is **withdrawn on measurement evidence** (innovation-gated), not re-aimed — re-aiming needs a fast v̂, which raises estimator loop gain and diverges under mismatch (measured).
- **Damping ratio ζ and phase margin PM are dimensionless design choices** (ff_pi ζ=1 critical damping; PM=50° chosen by the mismatch-band test suite).
- The truly free knobs are few, and each is explainable from principle; empirical ones are explicitly labeled in each law's docstring.

**Why ff_pi_acc is the shipped law** — best balance of tracking/lock AND maneuver overshoot, and a strict Pareto improvement over its predecessor ff_pi: matched composite 114.7 (step settle 277ms / 3.11px, accel rmse 4.4px, maneuver rmse 19.4px, relock 386ms), FPS behavior suite RMSE 20.4px / event overshoot 28.0px, while holding the **full delay band L20–80 and s0.7–1.3 with no divergence** — the entire mismatch band, step/maneuver/relock and event metrics are bit-identical to ff_pi (the â channel is exactly zero without sustained real acceleration), while accel tracking, matched composite and framerate independence improve (fpsΔ 9.2%→1.9%). The CUSUM-reset mechanism is principled: a broken target model (stop/reversal) is handled by discarding the contradicted velocity state and re-running the proven step response, not by patching gains. The â channel follows the same discipline: innovation-mean acceleration inversion is admitted only on sustained, plausibility-checked evidence (rebuild suppression + own-acceleration activity gate + significance floor).

**Predecessor status (arena side)**: `ff_pi.py` was removed from the library after `ff_pi_acc` dominated it on every test-suite cell (see above). `src/aimbot.cu` now implements ff_pi_acc; the AI-thread filter update carries the â sensor (cleaned innovation → robust scale → own-acceleration gate → gated ȳ EMA → inversion) and the 500Hz control tick assembles ê with (v̂+ε) and ½â·W² — mirroring `arena/laws/ff_pi_acc.py` line for line. Any future challenger must beat ff_pi_acc under the same triple gate before replacing it.

**Laws not shipped** (kept in `arena/`): `ballistic_ff` has the fastest convergence segment (step settle 167ms, relock 219ms, accel 3.9px) but pays worst-mismatch 135.0 and slightly higher event overshoot under flaky detection, and keeps the L80 settle-fail. `sliding_obs` is the robustness record (full L20–80 + s0.7–1.3 band, flattest profile, OVERALL 138.1) but its P-only tail makes first-reach slow, and hard y-axis stops can trip the CUSUM reset. `mpc_osc` holds the best worst-mismatch (113.4) and fixes L20, but solves a QP per tick — unverified against Jetson 500Hz embedded compute — and keeps the L80 knife-edge. `pi_guard` is pi_pm's no-FF structure plus model-break reset (full band pass); without FF the accel lag a/(Ki·ig) is structural. `kalman_pi`/`smith_filt` are covered on the Pareto front: the Kalman estimator's model overtrust makes a FF+CUSUM pack unfixable under mismatch (measured across 50+ configurations), and smith_filt's settle/relock/recovery records (151ms/198ms/9ms) are bound to an unfiltered extrapolation whose L80/s0.7 corners are structural. `pi_pm`/`sliding`/`ballistic` remain as undominated baselines (their successors carry disclosed regressions). `imm_pi` is the best matched/FPS law of the whole set (accel 4.3px / maneuver 13.6px, FPS RMSE 17.1px / event recovery 7ms) but fails the mismatch band outright. `reseed_pi` keeps ff_pi-level nominal behavior with an evidence-gated seed (exact counts-window junk bound) and now ties the zero-reset design at the mismatch edge (125.6 vs 125.1) while keeping the tail gains. If a different trade-off is ever needed, port the corresponding law's `step()` into `src/aimbot.cu` (units/quantization/counts/estimator must match line for line).

**Rejected paths (documented so they aren't re-explored)**: re-aiming the FF through a fast second velocity channel raises estimator loop gain and diverges at L30–70; hot design points (PM45–55 × β0≥0.06) pass matched but their estimator contamination makes step hunting at the band edges — the mismatch band is the hard constraint of the linear Smith+PI+FF family, and PM50/β0.03 is its test-suite-selected fastest point. Always-on maneuver-adaptive estimation (IMM, `imm_pi`) dies the same death from inside the estimator: under mismatch the Smith window misalignment turns own-command transients into large innovations, large innovations always favor the wide-covariance maneuver model, and the resulting ghost v̂ closes its loop through the physical plant — every σ̂-normalized gate (NIS authority gate, CUSUM) goes blind exactly when the loop self-oscillates, because the filter's covariance and the σ̂ EMA absorb the oscillation as "noise" (the σ̂ EMA runs away and the NIS gate collapses to unity). Online residual-delay adaptation is unobservable in this bookkeeping: the Smith error is first-order exact under constant velocities (the anchor offset cancels between the α-β velocity bias and the ê assembly), so the innovation carries no steady-state signature of Δ = L_true − L̂ — only transient bursts proportional to own-accel × Δ, which are exactly the frames where any estimate is contaminated. Event-overshoot peaks are bounded below by v·L (delay floor) plus the ~2–3 frame CUSUM alarm latency (set by the anti-false-alarm per-frame cap), so no estimator-side fix can cut them; only the post-peak tail is attackable, and paying mismatch margin for it loses on the composite. Evidence-gated adaptation is the counter-principle that works: keep the adaptive channel closed (or frozen) whenever own-motion contamination is possible and let it in only on sustained, plausibility-checked evidence — the validated instances are ff_pi_acc's triple-gated â channel, mpc_osc's innovation-alternation signature gate, and reseed_pi's window-junk-bound seed gate.

Same status, **the FF+CUSUM pack does not transfer onto a Kalman estimator** — its model overtrust (position gain ~0.04/frame vs α-β's 0.5) integrates the signed mismatch junk ∝ own-accel × Δ into v̂ where no gate can separate it from true target motion, and the L20 corner (phantom error v̂·35ms) forbids exactly the estimator bandwidth the accel tail needs (50+ official test-suite configurations, all reject). **The filtered-Smith family's L80/s0.7 corners are structural**: the pseudo-residual (window mismatch × own velocity) and real target disturbances are inseparable inside the residual channel; exact cleaning would need L_true, which is not in the law's input, and every online gating criterion either leaves one contamination window open or fires on legitimate re-capture transients (const_vel/accel break first). **imm_pi's own docstring premise was false**: the steady-state Riccati gain k2 was used with the wrong units (per-sample velocity gain treated as the per-frame α-β β), so its "low model" actually ran at β≈0.25 — 8.3× the documented 0.03; the same too-fast channel produces both the matched wins and the mismatch collapse. A rebuilt steady-gain MMAE on the corrected semantics tracks markedly better on matched and accel with L30–70 finite, but still fails s0.7/L80 and pays a large maneuver and framerate penalty — rejected by the gates (kept out of the library; recoverable from git history).

**Header constants** (constants area of `src/aimbot.cu`; principled rationale in `arena/laws/ff_pi_acc.py`'s docstring):

| constant | default | source |
|---|---|---|
| `wn=(90−PM)π/180/L` | PM=50 | principle: delay phase margin, auto-scales with L; 50 = fastest passing the full delay band (test suite) |
| `FF_GAIN_VAL` / `FF_ZETA` / `FF_I_GATE` | 1.0 / 1.0 / 8.0px | principle (type-2 exact FF) / principle (critical damping) / empirical (settled-region gate + withdrawal scale) |
| `PRED_ALPHA0/BETA0/L_COMP` | 0.50/0.03/1.10 | estimator (dt-normalized) / estimator (band-edge margin) / Smith over-comp |
| `ACC_SIG_CLIP_K/ACC_TAU_L` | 2.0 / 4.0 | â sensor: Huber clip (σ̂_r multiples, M-estimator standard) / ȳ EMA memory in L̂ units |
| `ACC_RB_HOLD_N/ACC_OW_ACTIV_K` | 1.5 / 2.0 | â sensor: rebuild hold in v̂-time-constants / own-acceleration activity gate in delay-window units |
| `ACC_SNR` | 10.0 | â significance floor (white-noise σ multiples; smallest whole value holding the full mismatch band bit-exact) |

### Known limitations

- arena's default 0.5px noise is optimistic; on a noisier real device ff_pi_acc converges slower — lower `PRED_BETA0` first (see Tuning); in extreme cases fall back to a more conservative design point (raise `FF_PM_DEG`), or port the sliding_obs law from arena (most robust, slowest).
- The constant-velocity (CV) predictor cannot predict acceleration: constant-accel targets have an a/Ki steady-state lag, removed slowly by the I term (maneuver RMSE ~20px is mostly the delay lower bound, not a law flaw).
- The speed cap binds before the law does at short range, and it is the first constraint on every fast-transient scenario: `arena.diag` puts only 3% of `fps_approach`'s error — the suite's largest, 45.85 px with a 184 px peak — on the velocity estimate, and `fps_dash` (4×0.4 = 1.6 px/ms) and the parts of `fps_jump_3m` where |v_z| > 1.5 px/ms are saturated the same way. Check `est_share` before attributing such errors to the control law. The cap is a physical requirement rather than a preference: the crosshair must be able to move at least as fast as the target's screen motion (960·v_world/d px/s, f≈960px@1080p), or the error grows without bound. Below that speed the error is a cliff and above it nothing changes — `arena.diag ff_pi_acc fps_approach --max-v 1.7` already moves it 45.85 → 18.88 px — so the useful setting is the smallest one clearing the fastest screen motion the system must face, and 2000 px/s clears the whole behaviour library (10 m dash 1.6, near jump 1.83, approach 1.99 px/ms). Raising it also loosens the integrator clamp and the â channel's limits, which moves the 60/120 fps composite split (60 fps improves in absolute terms; the ratio widens) and the worst-mismatch composite by about a percent — measure with the suites' `max_v` parameter, and see `-x` together with its derivation in the launcher template.
- mpc_osc solves a QP per tick; 500Hz embedded compute is unverified (feasible in arena); shipping it would need explicit MPC or a lower solve rate.
- Every law degrades under extreme mismatch (|L_true−L̂|>~30ms or s error >~40%) — beyond what calibration should ever produce; ff_pi_acc holds L20–70 + s0.7–1.3 fully, and at the L80 (+30ms) corner its step settle rides the 3px knife edge (final ≈3–4px, no divergence). Rely on calibration, not on the law toughing it out.

- `src/aimbot.cu` advances the filter's `dt` once per frame regardless of whether a detection was produced (`t_prev` is updated outside the `found` branch), so across a detection gap the prediction advances by a single frame interval and `beta` stays at `PRED_BETA0` — where arena uses the interval since the previous *detection*, which is what invariant 1 means. A re-acquisition frame therefore carries the whole gap's displacement in its innovation, making the `TRACK_JUMP_GATE` hard reset (and its ~417 ms weak-tracking window) more likely. Emulating that deviation (clamping the filter's `dt` to the frame interval) is nevertheless measurably neutral rather than harmful under the flaky FPS variant — airborne RMSE equal to slightly better than the current law, with the same on-body fraction — because holding `alpha` at its frame-rate value makes the re-acquisition update more conservative, not less. Fixing it is therefore not a route to the reported trailing, and carries on-device risk for no measured gain. Changing the control section needs an on-device rebuild.

### How to rerun & extend

1. `.venv\Scripts\python.exe -m arena.selftest` to confirm arena alignment.
2. After changing/adding a law: `arena.eval <law>` for the standard test suite; `arena.integrate <law>` for the integration (mismatch/relock/framerate included).
3. To see *which layer* a scenario's error lives in before changing anything: `arena.diag <law> [scenario ...]` — `est_share` says whether the error is worth attacking with a better estimator (high) or is the delay/loop/saturation bound (low); `arena.trace <law> <scenario> --csv out.csv` gives the per-tick process.
4. Once a better law is found, port its `step()` logic line for line into the control section of `src/aimbot.cu`. Units/quantization/counts/estimator must match the winning law exactly.
5. Tuning stands on arena measurements, and **delay mismatch must be tested** (the wide-delay sweep in `integrate.py`); a scheme that diverges under mismatch loses, no matter how fast.
