# Jetson-aimbot

AI visual aimbot (mouse pass-through) running on an NVIDIA Jetson Orin. The Jetson receives the game picture through a capture card, detects targets with TensorRT YOLO, and computes mouse corrections with a delay-aware control law. Commands are merged with the real mouse and emitted through a user-space USB device stack (USB raw_gadget; the device identifies as a generic USB mouse), so it behaves like an ordinary mouse.

## Pipeline

```
Capture card (UVC 1080p NV12) → GStreamer nvvidconv → CUDA preprocess → TensorRT YOLO
→ alpha-beta tracking → control law (pole-placement PI + type-2 velocity feedforward)
→ merged with the real mouse → USB raw_gadget device stack → game
```

## No hand-tuned gains

Aim at a static background with texture and hold both side keys for 5 seconds: the program excites the loop (draws a square), measures background motion with block phase correlation, and estimates sensitivity `s` (px/count) and loop delay `L` (ms) online with least squares. The control-law bandwidth is then derived from the calibrated `L` via phase margin (`wn=(90°−PM)π/180/L`, PM=50°) — no hand-tuned magic numbers, adapts to PC/PS5 and 60/120fps. The calibration values are written back into the per-game launch script automatically.

## Control law

The single binary `bin/aimbot` runs **ff_pi**: pole-placement PI + type-2 velocity feedforward with direction-contradiction CUSUM velocity reset, plus optional training-data collection (`-o`, otherwise pure aimbot). The law was selected and tuned in `arena/`, a neutral pure-Python plant+sensor simulator that also hosts the alternative laws (ballistic, sliding, MPC, …) kept as Pareto points in speed/robustness. `AGENTS.md` is the full design document; `arena/AUTHORING.md` is the law-author guide.

## Repository layout

```
src/       CUDA/C++ source (aimbot.cu — the ff_pi law)
scripts/   compile.sh / convert.sh (ONNX→engine) / setup_mouse.sh (raw_gadget mouse channel)
           / game/template.sh.example
arena/     pure-Python control-law simulator + benchmark suite
engine/    TensorRT engines (not committed)
onnx/      ONNX models (not committed)
```

## Build & run (on the Jetson)

```bash
scripts/compile.sh            # → bin/
scripts/convert.sh            # onnx/*.onnx → engine/*.engine (TensorRT 10)
scripts/setup_mouse.sh        # load raw_gadget, free the UDC, /dev/raw-gadget permissions
cp scripts/game/template.sh.example scripts/game/<game>.sh   # one launcher per game
chmod +x scripts/game/<game>.sh
scripts/game/<game>.sh        # calibrate once; S_EST/L_EST are written back into it
```

Requires JetPack with TensorRT 10, CUDA, OpenCV 4, GStreamer, and a UVC capture card supporting 1080p NV12 @ 120 Hz. The USB output needs the kernel **`raw_gadget` module** — an external dependency to provide on the deployment machine (distro package, or built out-of-tree per the kernel doc `Documentation/usb/raw_gadget.rst`).

## arena (control-law development)

```bash
python3 -m venv .venv
.venv/Scripts/python.exe -m pip install -r requirements.txt   # Windows dev machine
.venv/Scripts/python.exe -m arena.selftest                    # validate the simulator
.venv/Scripts/python.exe -m arena.eval ff_pi                  # standard test suite for the main law
.venv/Scripts/python.exe -m arena.integrate                   # all-law leaderboard + robustness sweeps
```
