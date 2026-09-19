# Jetson-aimbot

AI visual aimbot (mouse pass-through) running on an NVIDIA Jetson Orin. The Jetson receives the game picture through a capture card, detects targets with TensorRT YOLO, and computes mouse corrections with a delay-aware control law. Commands are merged with the real mouse and emitted through a user-space USB device stack (USB raw_gadget; the device identifies as a generic USB mouse), so it behaves like an ordinary mouse.

A second output mode (`-M pad`) passes a physical Xbox-layout gamepad through, merges the same law's velocity onto its right stick, and presents the emulated **wired Xbox 360 pad** (0x045E/0x028E) — the host's XInput stack reads it as a controller (`XInputGetState` rc=0). The two modes are mutually exclusive.

## Pipeline

```
Capture card (UVC 1080p NV12) → GStreamer nvvidconv → CUDA preprocess → TensorRT YOLO
→ alpha-beta tracking → control law (pole-placement PI + type-2 velocity feedforward)
→ merged with the real mouse   → USB raw_gadget device stack (generic HID mouse) → game
  merged with the real gamepad → USB raw_gadget device stack (XInput pad, -M pad) → game
```

## No hand-tuned gains

Aim at a static background with texture and hold both side keys for 5 seconds: the program excites the loop (draws a square), measures background motion with block phase correlation, and estimates sensitivity `s` (px/count) and loop delay `L` (ms) online with least squares. The control-law bandwidth is then derived from the calibrated `L` via phase margin (`wn=(90°−PM)π/180/L`, PM=50°) — no hand-tuned magic numbers, adapts to PC/PS5 and 60/120fps. The calibration values are written back into the per-game launch script automatically.

Each output mode has its own calibration, stored separately and never overwriting the other: mouse mode (`-M hid`) measures `s`/`L` from the side-key trigger and writes `S_EST`/`L_EST`; pad mode (`-M pad`) is triggered by L3+R3 (or the WebUI's 「开始标定」 button = hot param `padcalib=1`) and measures the loop delay plus the stick's **full-deflection screen speed** — the firmware owns the right stick for ~11 s and plays the same square at full deflection — writing `PAD_STICK_GAIN` (px/s) / `L_EST_PAD`. Both follow the same measurement chain (phase correlation → least squares → delay sweep); a pad fit outside its design band (e.g. a screen that does not answer the injection, i.e. no game running) fails with a shake and writes nothing.

## Control law

The single binary `bin/aimbot` runs **ff_pi**: pole-placement PI + type-2 velocity feedforward with direction-contradiction CUSUM velocity reset, plus optional training-data collection (`-o`, otherwise pure aimbot). The law was selected and tuned in `arena/`, a neutral pure-Python plant+sensor simulator that also hosts the alternative laws (ballistic, sliding, MPC, …) kept as Pareto points in speed/robustness. `AGENTS.md` is the full design document; `arena/AUTHORING.md` is the law-author guide.

## Repository layout

```
src/       CUDA/C++ source: main.cu (entry) + core/ (control law, estimator,
           calibration) + io/ (capture, USB mouse output, pad input/merge,
           XInput pad output, hot params)
scripts/   compile.sh / convert.sh (ONNX→engine) / setup_mouse.sh (raw_gadget mouse channel)
           / game/template.sh.example
           / test/uinput_pad_test.py (on-device pad-mode e2e)
           / test/xinput_probe.ps1 (Windows-side XInput verdict probe, P/Invoke xinput1_4.dll)
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
scripts/game/<game>.sh        # calibrate once; the calibration values are written back into it
```

`OUTPUT_MODE` in the launcher selects the channel (`hid` = USB mouse, `pad` = XInput pad); pad
mode additionally passes `-P <by-id substring>`, `-G <stick gain>` and `-l <L_EST_PAD>`, hid mode
passes `-s`/`-l` — the launcher branches on the mode, the mouse-mode arguments being exactly as
before.

Requires JetPack with TensorRT 10, CUDA, OpenCV 4, GStreamer, and a UVC capture card supporting 1080p NV12 @ 120 Hz. The USB output needs the kernel **`raw_gadget` module** — an external dependency to provide on the deployment machine (distro package, or built out-of-tree per the kernel doc `Documentation/usb/raw_gadget.rst`). Both output modes (`-M hid` mouse, `-M pad` XInput pad) own the UDC, so they run one at a time; `scripts/setup_mouse.sh` frees the UDC and sets `/dev/raw-gadget` permissions for either.

Pad mode (`-M pad`, physical gamepad on `/dev/input/by-id`, `-P` to select it; L3+R3 held 5 s calibrates it):

```bash
bin/aimbot -M pad -m engine/apex.engine -d /dev/video0 -f 120 -k fire   # XInput pad output
scripts/test/xinput_probe.ps1 -Count 8                                  # on the Windows host
```

The Windows probe polls `xinput1_4.dll!XInputGetState` on all four user slots and prints the
return code, the packet number and the decoded report: rc=1167 means the slot is empty,
rc=0 with a rising packet number means the emulated pad is live.

## arena (control-law development)

```bash
python3 -m venv .venv
.venv/Scripts/python.exe -m pip install -r requirements.txt   # Windows dev machine
.venv/Scripts/python.exe -m arena.selftest                    # validate the simulator
.venv/Scripts/python.exe -m arena.eval ff_pi                  # standard test suite for the main law
.venv/Scripts/python.exe -m arena.integrate                   # all-law leaderboard + robustness sweeps
```
