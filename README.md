# Jetson-aimbot

AI visual aimbot running on an NVIDIA Jetson Orin. The Jetson receives the game picture through a capture card, detects targets with TensorRT YOLO, and computes mouse corrections with a delay-aware control law. Commands are merged with the real mouse and emitted through a userspace USB device stack built on the kernel's `raw_gadget` (the device identifies as a generic USB mouse), so it behaves like an ordinary mouse.

Two output modes (`-M`, mutually exclusive): **hid** — the mouse path above; **pad** — a physical Xbox-layout gamepad is passed through 1:1 and the law's desired velocity is merged onto its right stick, while the Jetson presents itself to the host as a **wired Xbox 360 pad** (`0x045E/0x028E`, the reference descriptor set plus a machine-derived serial), so the host's own XInput stack reads it as a controller at a 1 kHz terminal refresh rate. The gamepad channel keeps full 16-bit stick resolution end to end: an 8-bit device (`0..255`, rest `128`) is expanded by a compile-time 256-entry table — the midpoint code 128 maps to exactly 0 and the 255 physical levels are spread over ±32767 with a uniform step of 258 (`= floor(32767/127)`, chosen over 257 so the top level is not left 0.4 % short of full deflection); the single short step and the overflow both land at the deepest level (`out[0] = −32767`, `out[255] = +32766`). Already-centred devices pass through unchanged.

## Pipeline

```
Capture card (UVC 1080p NV12) → GStreamer nvvidconv → CUDA preprocess → TensorRT YOLO
→ alpha-beta tracking → control law (pole-placement PI + type-2 velocity feedforward)
→ merged with the real mouse (hid) / with the physical gamepad's right stick (pad)
→ USB raw_gadget userspace device: generic HID mouse, or wired Xbox 360 pad
→ control tick 1 kHz → game
```

## No hand-tuned gains

Aim at a static background with texture and hold both side keys for 5 seconds: the program excites the loop (draws a square), measures background motion with block phase correlation, and estimates the loop delay `L` (ms) online — the one calibrated quantity, written back into the per-game launch script automatically. The control-law bandwidth is then derived from `L` via phase margin (`wn=(90°−PM)π/180/L`, PM=50°) and the per-game speed feel is dialled by four per-axis **pull-speed ratios** (`--spd`, `--ads-spd`: effective sensitivity = baseline / (ratio/100), so `100` is the baseline and a larger ratio means a faster pull) — no hand-tuned magic numbers, adapts to PC/PS5 and 60/120fps.

## Control law

The single binary `bin/aimbot` runs **ff_pi_acc**: pole-placement PI + type-2 velocity feedforward with direction-contradiction CUSUM velocity reset and an innovation-mean acceleration channel, plus optional training-data collection (`-o`, otherwise pure aimbot). The law was selected and tuned in `arena/`, a neutral pure-Python plant+sensor simulator that also hosts the alternative laws (ballistic, sliding, MPC, …) kept as Pareto points in speed/robustness. `AGENTS.md` is the full design document; `arena/AUTHORING.md` is the law-author guide.

## Repository layout

```
src/       CUDA/C++ source — main.cu (entry) + core/ (shared state, control law, estimator,
           calibration, TensorRT helpers) + io/ (capture, mouse input, gamepad input/merge,
           USB device stack + the mouse and Xbox 360 pad device definitions, hot params)
scripts/   compile.sh / convert.sh (ONNX→engine) / setup_mouse.sh (raw_gadget setup) /
           game/template.sh.example / test/ (uinput pad e2e, Windows XInput probe, key probe)
arena/     pure-Python control-law simulator + benchmark suite
build/     per-TU object files (not committed)
engine/    TensorRT engines (not committed)
onnx/      ONNX models (not committed)
```

## Build & run (on the Jetson)

```bash
scripts/compile.sh            # → bin/
scripts/convert.sh            # onnx/*.onnx → engine/*.engine (TensorRT 10)
scripts/setup_mouse.sh        # load raw_gadget, free the UDC, set /dev/raw-gadget permissions
cp scripts/game/template.sh.example scripts/game/<game>.sh   # one launcher per game
chmod +x scripts/game/<game>.sh
scripts/game/<game>.sh        # calibrate once; L_EST is written back into it
```

The launcher's `OUTPUT_MODE` picks the channel (`hid` mouse / `pad` Xbox 360 pad, with `PAD_KEYWORD`, `PAD_TRIG_THR` and `PAD_DUMP` for the gamepad side). The gamepad end-to-end check on the Jetson is `sudo python3 scripts/test/uinput_pad_test.py --mode pad` (synthesizes a virtual pad and asserts the whole input/merge chain through `--pad-dump`); the host-side verdict is `powershell -ExecutionPolicy Bypass -File scripts/test/xinput_probe.ps1` (`XInputGetState` rc / packet number / decoded fields).

Requires JetPack with TensorRT 10, CUDA, OpenCV 4, GStreamer, the kernel's `raw_gadget` module (distro package, or built out-of-tree per `Documentation/usb/raw_gadget.rst`), and a UVC capture card supporting 1080p NV12 @ 120 Hz.

## arena (control-law development)

```bash
python3 -m venv .venv
.venv/Scripts/python.exe -m pip install -r requirements.txt   # Windows dev machine
.venv/Scripts/python.exe -m arena.selftest                    # validate the simulator
.venv/Scripts/python.exe -m arena.eval ff_pi_acc              # standard test suite for the main law
.venv/Scripts/python.exe -m arena.integrate                   # all-law leaderboard + robustness sweeps
```
