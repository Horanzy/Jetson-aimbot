# arena control-law author guide

All laws are evaluated on **exactly the same scenario suite**. This guide defines the interface and the evaluation method. Run `python3 -m ...` from the repository root.

## Interface (must be followed strictly)

Your file: `arena/laws/<name>.py` (imported via try/except by `__init__.py`, so it is active as soon as it exists).

```python
from __future__ import annotations
import math
from typing import Optional, Tuple
from arena.core import Observation, LawConfig
from arena.laws.base import Law, register

@register("<law_name>")
class MyLaw(Law):
    def __init__(self, <tunable parameters, with defaults>): ...
    def reset(self, cfg: LawConfig):
        self.cfg = cfg
        # reset ALL internal state (filters/integrators/history)
    def step(self, t: float, obs: Optional[Observation]) -> Tuple[int, int]:
        # return integer counts (cx, cy)
        ...
        return cx, cy
```

### Observation (arena → law)
- `obs.t`: frame timestamp (ms) = capture time + L_true.
- `obs.dx, obs.dy`: measured (target − crosshair) in px, reflecting the world as of `(obs.t − L_true)`, noise included.
- `obs.new`: True = new frame since the last tick; False = same frame (inter-frame extrapolation is up to you).
- `obs` may be None (no frame yet at startup).

### LawConfig
`cfg.s` (px/count, believed value), `cfg.L` (ms, believed delay), `cfg.h` (2ms), `cfg.frame_dt`
(8.33ms @120fps), `cfg.max_v` (px/ms, ~1.5), `cfg.count_limit` (120), `cfg.fov_radius` (150).

## Plant/sensor ground truth (arena is neutral; these are reliable facts)
- Crosshair integrates: every tick `crosshair += s_true × counts_you_sent`.
- **The only delay is on the observation side**: a frame stamped `obs.t` reflects the world as of `obs.t − L_true`.
- Control runs at 500Hz (2ms), observations at 120fps (8.33ms) — about 4 ticks per frame.
- You keep your own command history and detection history; arena provides nothing else.
- `cfg.s`/`cfg.L` are the values you **believe**; arena's ground truth may differ (mismatch testing).
- Units are px / px/ms. To command velocity v (px/ms): `counts = v * cfg.h / cfg.s`.
- Use remainder-accumulation quantization so small corrections don't truncate to 0.

## Evaluation (standard battery, identical for everyone)
CLI: `python3 -m arena.eval <law_name>`
or:
```python
from arena.laws.base import get_law
from arena.eval import battery
battery(lambda: get_law("<law_name>")())
```
The battery: (A) standard multi-scenario suite @ matched L=50 / 120fps; (B) delay-mismatch
sweep L_true ∈ {30,40,50,60,70}, belief=50; (C) 60 vs 120fps. Prints composite + OVERALL
(lower is better). **Divergence is heavily penalized** — a law that diverges under mismatch
loses no matter how fast it is.

## Reference baseline (ff_pi, the current main control law)
OVERALL=153.2, matched composite=171.3 (step settle 377ms / overshoot 3.5px / first reach 198ms;
const_vel rmse 0.9px; maneuver rmse 23.1), worst mismatch=122.9, zero divergence across wide
delay L20–80 and sensitivity s0.7–1.3. Goal: beat it across the board.

## Rules
- Only modify your own law file; don't touch core/runner/eval/scenarios/base.
- You **must actually run arena and iterate** — theory alone doesn't count. Tune the method to its own optimum before reporting.
- Report: best parameters + full battery output + mismatch-robustness profile (diverges or not / at which L_true) + known limitations.
