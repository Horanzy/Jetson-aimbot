# Jetson-aimbot

AI visual aimbot (mouse pass-through) running on an NVIDIA Jetson Orin. The Jetson receives the game picture through a capture card, detects targets with TensorRT YOLO, and computes mouse corrections with a delay-aware control law. Commands are merged with the real mouse and emitted through a USB gadget (the device identifies as a Microsoft IntelliMouse Explorer 3.0), so it behaves like an ordinary mouse.

## Pipeline

```
Capture card (UVC 1080p NV12) → GStreamer nvvidconv → CUDA preprocess → TensorRT YOLO
→ alpha-beta tracking → control law (pole-placement PI + type-2 velocity feedforward)
→ merged with the real mouse → USB Gadget (/dev/hidg0) → game
```

## No hand-tuned gains

Aim at a static background with texture and hold both side keys for 5 seconds: the program excites the loop (draws a square), measures background motion with block phase correlation, and estimates sensitivity `s` (px/count) and loop delay `L` (ms) online with least squares. The control-law bandwidth is then derived from the calibrated `L` via phase margin (`wn=(90°−PM)π/180/L`, PM=60°) — no hand-tuned magic numbers, adapts to PC/PS5 and 60/120fps. The calibration values are written back into the per-game launch script automatically.

## Control laws

| binary | law | character |
|---|---|---|
| `bin/aimbot` | ffpi — pole-placement PI + type-2 velocity feedforward | **main program**; the only fast law with zero divergence across the full delay/sensitivity sweep; optional training-data collection (`-o`) |
| `bin/aimbot_ballistic` | ballistic flick + critically damped convergence | fastest flick + best maneuver tracking (alternative) |
| `bin/aimbot_sliding` | boundary-layer sliding mode + ballistic flick | most robust: zero divergence + flattest mismatch profile, but slowest (alternative) |

All laws were developed and benchmarked in `arena/`, a neutral pure-Python plant+sensor simulator. `AGENTS.md` is the full design document; `arena/AUTHORING.md` is the law-author guide.

## Repository layout

```
src/       CUDA/C++ sources (aimbot.cu + 2 alternative control laws)
scripts/   compile.sh / convert.sh (ONNX→engine) / setup_mouse.sh (USB gadget) / game/*.sh
arena/     pure-Python control-law simulator + benchmark suite
engine/    TensorRT engines (not committed)
onnx/      ONNX models (not committed)
```

## Build & run (on the Jetson)

```bash
scripts/compile.sh            # → bin/
scripts/convert.sh            # onnx/*.onnx → engine/*.engine (TensorRT 10)
scripts/setup_mouse.sh        # create the /dev/hidg0 USB gadget mouse
scripts/game/battlefield.sh   # per-game launcher (calibration write-back included)
```

Requires JetPack with TensorRT 10, CUDA, OpenCV 4, GStreamer, and a UVC capture card supporting 1080p NV12 @ 120 Hz.

## arena (control-law development)

```bash
python3 -m venv .venv
.venv/Scripts/python.exe -m pip install -r requirements.txt   # Windows dev machine
.venv/Scripts/python.exe -m arena.selftest                    # validate the simulator
.venv/Scripts/python.exe -m arena.integrate                   # all-law leaderboard + robustness sweeps
```
