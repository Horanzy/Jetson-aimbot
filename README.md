# Jetson-aimbot

AI visual aimbot running on an NVIDIA Jetson Orin. The Jetson receives the game picture through a capture card, detects targets with TensorRT YOLO, and computes mouse corrections with a delay-aware control law. Commands are merged with the real mouse and emitted through a userspace USB device stack built on the kernel's `raw_gadget` (the device identifies as a generic USB mouse), so it behaves like an ordinary mouse.

Three output modes (`-M`, mutually exclusive): **hid** — the mouse path above; **pad** — a physical Xbox-layout gamepad is passed through 1:1 and the law's desired velocity is merged onto its right stick, while the Jetson presents itself to the host as a **wired Xbox 360 pad** (`0x045E/0x028E`, the reference descriptor set plus a machine-derived serial), so the host's own XInput stack reads it as a controller at a 1 kHz terminal refresh rate; **p5g** — the same input/merge/calibration chain aimed at the PS5: the Jetson enumerates as the **P5 General dongle's own shape** (`0x2B81/0x0101`, a 165-byte HID report descriptor, 64-byte input reports) while a **real P5 General dongle** plugged into one of its host ports signs every report over hidraw, so the PS5 only ever receives bytes the dongle returned, and the PS5's auth challenge/response (feature reports `0xF0`/`0xF1`/`0xF2`) is forwarded verbatim — with no dongle present, no report is produced at all. The gamepad channel keeps full 16-bit stick resolution end to end: an 8-bit device (`0..255`, rest `128`) is expanded by a compile-time 256-entry table — the midpoint code 128 maps to exactly 0 and the 255 physical levels are spread over ±32767 with a uniform step of 258 (`= floor(32767/127)`, chosen over 257 so the top level is not left 0.4 % short of full deflection); the single short step and the overflow both land at the deepest level (`out[0] = −32767`, `out[255] = +32766`). Already-centred devices pass through unchanged, and each wire format inverts that channel exactly: the XInput encoder writes the int16 logical value, the P5G encoder writes the 8-bit code back (`floor(v/258) + 128` — the identity on all 256 codes, with the injection's own 16-bit amount landing as whole 8-bit steps of 258 logical counts).

## Pipeline

```
Capture card (UVC 1080p NV12) → GStreamer nvvidconv → CUDA preprocess → TensorRT YOLO
→ alpha-beta tracking → control law (pole-placement PI + type-2 velocity feedforward)
→ merged with the real mouse (hid) / with the physical gamepad's right stick (pad, p5g)
→ USB raw_gadget userspace device: generic HID mouse, wired Xbox 360 pad, or P5 General pad
  (p5g additionally signs every report through the real dongle on a host port)
→ control tick 1 kHz → game
```

## No hand-tuned gains

Aim at a static background with texture, take both hands off the controls and hold both side keys for 5 seconds (the gamepad modes: L3+R3, or the webui's 「开始标定」 button). The program excites the loop — per axis, alternating deflections that stop the moment the picture has travelled far enough, each followed by a quiet pause — measures the background motion with block phase correlation, and estimates the **loop delay `L` (ms)**, the one calibrated quantity, from three independent readings of that same observation stream (a sub-frame-exact tail sum plus two frame-quantized edges). Success is a nod and writes back that output mode's delay VAR (`HID_L_EST` for hid, `PAD_L_EST` for pad, `P5G_L_EST` for p5g) into the per-game launch script; a failure is a shake that writes nothing and states its reason and evidence in the log. The control-law bandwidth is then derived from `L` via phase margin (`wn=(90°−PM)π/180/L`, PM=50°) and the per-game speed feel is dialled by four per-axis **pull-speed ratios** (`--spd`, `--ads-spd`: effective sensitivity = baseline / (ratio/100), so `100` is the baseline and a larger ratio means a faster pull) — no hand-tuned magic numbers, adapts to PC/PS5 and 60/120fps.

## Control law

The single binary `bin/aimbot` runs **ff_pi_acc**: pole-placement PI + type-2 velocity feedforward with direction-contradiction CUSUM velocity reset and an innovation-mean acceleration channel, plus optional training-data collection (`-o`, otherwise pure aimbot). The law was selected and tuned in `arena/`, a neutral pure-Python plant+sensor simulator that also hosts the alternative laws (ballistic, sliding, MPC, …) kept as Pareto points in speed/robustness. `AGENTS.md` is the full design document; `arena/AUTHORING.md` is the law-author guide.

## Repository layout

```
src/       CUDA/C++ source — main.cu (entry) + core/ (shared state, control law, estimator,
           calibration, TensorRT helpers) + io/ (capture, mouse input, gamepad input/merge,
           USB device stack + the mouse, Xbox 360 pad and P5 General pad device definitions,
           hot params)
scripts/   compile.sh / convert.sh (ONNX→engine) / setup_mouse.sh (raw_gadget setup) /
           game/template.sh.example / test/ (uinput pad e2e, Windows XInput probe, key probe,
           P5 General dongle bring-up probe)
docs/      p5general/ — the P5 General wire protocol (zh-CN/en), transcribed from the
           GP2040-CE reference firmware and hardware-verified against the real dongle
arena/     pure-Python control-law simulator + benchmark suite
webui/     browser control panel (FastAPI + vanilla JS): profile params, start/stop,
           hot-param push, calibration button, model/compile tasks, log stream
build/     per-TU object files (not committed)
engine/    TensorRT engines (not committed)
onnx/      ONNX models (not committed)
```

## Parameter face

Every per-game value lives in the launcher script (`scripts/game/<game>.sh`, a copy of
`template.sh.example`), and **the script is the single source of truth** — the WebUI only rewrites
the values of the VARs it knows and leaves every other line byte-identical. Each hand-edited value
carries a guard default (`CLASS_ID="${CLASS_ID:-0}"`), so a script missing any single line still
launches; the WebUI reads the guard's effective value, not its literal text.

| Script VAR | CLI | Meaning |
|---|---|---|
| `OUTPUT_MODE` | `-M` | `hid` / `pad` / `p5g` — mutually exclusive, each owns the UDC |
| `MOUSE_KEYWORD` | `-D` | hid: mouse `by-id` match substring (empty = lexicographically first `*-event-mouse`) |
| `PAD_KEYWORD` | `-P` | pad/p5g: gamepad match substring (empty = any `*-event-joystick`; the dongle's own node is excluded) |
| `PAD_TRIG_THR` | `-T` | pad/p5g: trigger threshold in % of full scale, shared by RT and LT; it gates the aim-trigger decision only — the analog value passes through 1:1 (hot param `padthr`) |
| `PAD_DUMP` | `--pad-dump` | pad/p5g: log the merged logical state every ≥50 ms (injection debugging) |
| `HID_L_EST` / `PAD_L_EST` / `P5G_L_EST` | `-l` | the calibrated loop delay, **one slot per output mode** — the only quantity the firmware ever writes back (each is hand-editable too) |
| `HID_SPDX` `HID_SPDY` `HID_ADS_SPDX` `HID_ADS_SPDY` | `--spd` / `--ads-spd` | hid slot: pull-speed ratios, per axis (integer; `100` = baseline, larger = faster, meaningful band `5..2000`) |
| `PAD_SPDX` `PAD_SPDY` `PAD_ADS_SPDX` `PAD_ADS_SPDY` | `--spd` / `--ads-spd` | pad slot: the same four in the gamepad channel's units |
| `P5G_SPDX` `P5G_SPDY` `P5G_ADS_SPDX` `P5G_ADS_SPDY` | `--spd` / `--ads-spd` | p5g slot: the same four for the PS5 channel |
| `MAX_SPEED` | `-x` | crosshair speed cap px/s (derivation in the template) |
| `AIM_KEY` / `AIM_ENABLED` / `FOV_R` / `PREVIEW` | `-k` / `-a` / `-r` / `-v` | trigger key, mouse takeover, FOV radius, preview |
| `CAPTURE` `CAP_FIRE` `CAP_DET` `CAP_AUTO` `OUT_DIR` `FIRE_MS` `AUTO_S` `COOLDOWN_MS` `JPEG_Q` | `-o` `-e` `-F` `-A` `-C` `-q` | training-data collection |

**Each output mode owns a slot of five values** (one delay + four ratios), and switching
`OUTPUT_MODE` switches the whole slot: the launcher's `case` takes that slot's delay and ratios, the
firmware writes a calibration back only into that slot's delay VAR, and the WebUI shows, edits and
saves only the current mode's slot. The slots are not interchangeable, which is why they exist: the
same ratio number means different speeds under the two bases below, and each mode's loop delay is
its own physical quantity (`p5g` adds the dongle's signing round trip to it).

The per-game feel is the four integer ratios of the active slot — there is no per-axis sensitivity
entry, no second gain and no curve coefficient. Each ratio is an inverse factor on the effective gain
(`base × 100 / ratio`), and both bases are compile-time constants (`S_HID_BASE` = 1.0 px/count,
`GAIN_PAD_BASE` = 3000 px/s full deflection). The base only decides how close a ratio's first guess
is; the ratios are what gets dialled, and the injection, the ledger→pixel scale and the own-motion
compensation all consume the same per-axis effective gain, so they cannot desynchronize.

## Hot-parameter channel

The binary opens a **localhost-only** UDP channel (`127.0.0.1:47700`, datagrams of
`key=value;key=value`) so a running instance can be retuned without a restart. Whitelisted keys —
`t` `y` `x` `fov` `padthr` `spdx` `spdy` `adsspdx` `adsspdy` `k` `aim` `cap_fire` `cap_det`
`cap_auto` — are clamped by the firmware as well, so the sender is never trusted; `padcalib=1`
requests one calibration round (consumed once; a request arriving mid-run is logged and dropped,
never re-entrant). Structural constants (PM/ζ/FF gain/CUSUM thresholds) are compile-time and
deliberately absent from the wire. Every applied key is echoed as a `[热参] …` log line; the full
table with clamps is in `webui/README.md`.

## WebUI

```bash
cd <deploy-root>/webui
sudo python3 -m pip install -r requirements.txt
sudo bash deploy/install.sh           # systemd unit + autostart; token printed in the journal
sudo journalctl -u aimbot-webui -n 20 --no-pager   # first-run token → set a password in the browser
```

It is an **orchestrator, not a replacement**: it runs the same three steps a launcher does
(`jetson_clocks` → `setup_mouse.sh` → `bin/aimbot` with the full parameter set, `-S` pointing at the
profile script so calibration writes back to the same place). Adding a game means copying
`scripts/game/template.sh.example` → `<game>.sh` (the WebUI has a "copy profile" button too); the new
script shows up in the profile list on the next scan. The output mode is a field of the profile, and
the rows a mode owns (`-D` for hid, `-P`/`-T`/`--pad-dump` for pad/p5g) and the mode's five-value
slot (delay + four ratios) are shown only for that mode — switching it changes which device VARs
reach the generated command line, and the delay plus all four ratios switch with it
(`HID_L_EST`/`HID_SPDX`… ↔ `PAD_L_EST`/`PAD_SPDX`… ↔ `P5G_L_EST`/`P5G_SPDX`…); a save writes back
only the current mode's slot. Calibration runs from the launcher's key hold (both side
keys for hid, L3+R3 for the gamepad modes) or, for pad/p5g, from the WebUI's 「开始标定」 button (hot
param `padcalib=1`). The `[标定]` lines, the per-segment readings and the verdict stay visible in the
page's log stream; the 60 s report-rate lines (`[USB-HID]`/`[PAD-USB]`), `[AI FPS]` and `[SAVE]` are
filtered out of it (they appear as cards and history instead, and the raw ring is still in the
download). `webui/README.md` is the operator manual (permissions, security, discovery model,
hot-param table, acceptance checklist).

## Build & run (on the Jetson)

```bash
scripts/compile.sh            # → bin/
scripts/convert.sh            # onnx/*.onnx → engine/*.engine (TensorRT 10)
scripts/setup_mouse.sh        # load raw_gadget, free the UDC, set /dev/raw-gadget permissions
cp scripts/game/template.sh.example scripts/game/<game>.sh   # one launcher per game
chmod +x scripts/game/<game>.sh
scripts/game/<game>.sh        # calibrate once; the mode's delay VAR is written back into it
```

The launcher's `OUTPUT_MODE` picks the channel (`hid` mouse / `pad` Xbox 360 pad / `p5g` P5 General pad, the two gamepad modes sharing `PAD_KEYWORD`, `PAD_TRIG_THR` and `PAD_DUMP`). Before a `p5g` run, `sudo python3 scripts/test/p5g_dongle_probe.py` dumps the real dongle's descriptors, exercises the three feature transfers and times the signing round trip (that round trip is the hardware bound on a continuously-updated report stream). The gamepad end-to-end check on the Jetson is `sudo python3 scripts/test/uinput_pad_test.py --mode pad|p5g` (synthesizes a virtual pad and asserts the whole input/merge chain through `--pad-dump`; the dump reads the publish point, which is backend-independent, so the same assertions cover the P5G backend); the calibration round has its own driver, `sudo python3 scripts/test/uinput_calib_test.py --mode hid|pad` (synthesizes the mode's trigger device, drives one round against the capture card and asserts the trigger, the excitation waveform, the measured sampling rate and the write-back rule); the host-side verdict for the XInput backend is `powershell -ExecutionPolicy Bypass -File scripts/test/xinput_probe.ps1` (`XInputGetState` rc / packet number / decoded fields).

Requires JetPack with TensorRT 10, CUDA, OpenCV 4, GStreamer, the kernel's `raw_gadget` module (distro package, or built out-of-tree per `Documentation/usb/raw_gadget.rst`), and a UVC capture card supporting 1080p NV12 @ 120 Hz.

## arena (control-law development)

```bash
python3 -m venv .venv
.venv/Scripts/python.exe -m pip install -r requirements.txt   # Windows dev machine
.venv/Scripts/python.exe -m arena.selftest                    # validate the simulator
.venv/Scripts/python.exe -m arena.eval ff_pi_acc              # standard test suite for the main law
.venv/Scripts/python.exe -m arena.integrate                   # all-law leaderboard + robustness sweeps
```
