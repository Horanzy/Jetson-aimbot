# arena 控制律作者指南

所有 law 在**完全相同的多组场景**下评测。本指南定义接口与评测方法。在仓库根目录运行 `python3 -m ...`。

## 接口 (必须严格符合)

你的文件: `arena/laws/<文件名>.py` (已被 `__init__.py` try/except 导入, 建好即生效)。

```python
from __future__ import annotations
import math
from typing import Optional, Tuple
from arena.core import Observation, LawConfig
from arena.laws.base import Law, register

@register("<律名>")
class MyLaw(Law):
    def __init__(self, <可调参数, 给默认值>): ...
    def reset(self, cfg: LawConfig):
        self.cfg = cfg
        # 重置全部内部状态 (滤波器/积分器/历史)
    def step(self, t: float, obs: Optional[Observation]) -> Tuple[int, int]:
        # 返回整数 counts (cx, cy)
        ...
        return cx, cy
```

### Observation (arena → law)
- `obs.t`: 帧时间戳 (ms) = 采集时刻 + L_真。
- `obs.dx, obs.dy`: 测到的 (目标 − 准星) px, 反映 `(obs.t − L_真)` 那一刻的世界, 含噪声。
- `obs.new`: True=自上一拍以来的新帧; False=同一帧 (帧间外推由你决定)。
- `obs` 可能为 None (开局还没有帧)。

### LawConfig
`cfg.s` (px/count, 相信值), `cfg.L` (ms, 相信延迟), `cfg.h` (2ms), `cfg.frame_dt`
(8.33ms@120fps), `cfg.max_v` (px/ms, ~1.5), `cfg.count_limit` (120), `cfg.fov_radius` (150)。

## 植物/传感器真值 (arena 中立, 这些是可靠事实)
- 准星积分: 每拍 `crosshair += s_真 × 你发的counts`。
- **唯一延迟在观测侧**: 时间戳 `obs.t` 的帧反映 `obs.t − L_真` 的世界。
- 控制 500Hz (2ms), 观测 120fps (8.33ms) —— 约 4 拍一帧。
- 你自己保管指令历史与检测历史, arena 不给别的。
- `cfg.s`/`cfg.L` 是你**相信**的值; arena 真值可能不同 (失配测试)。
- 单位 px / px/ms。发速度 v (px/ms): `counts = v * cfg.h / cfg.s`。
- 用余数累加量化, 避免小修正被截断为 0。

## 评测 (标准化电池, 所有人相同)
CLI: `python3 -m arena.eval <律名>`
或:
```python
from arena.laws.base import get_law
from arena.eval import battery
battery(lambda: get_law("<律名>")())
```
电池: (A) 标准多组场景 @匹配 L=50/120fps; (B) 延迟失配扫描 L_真∈{30,40,50,60,70},
相信=50; (C) 60 vs 120fps。打印 composite + OVERALL (越小越好)。
**发散重罚** —— 失配下发散的律无论多快都输。

## 参考基线 (ff_pi, 现役主控制律)
OVERALL=153.2, 匹配 composite=171.3 (step settle 377ms / 过冲 3.5px / 首达 198ms;
const_vel rmse 0.9px; maneuver rmse 23.1), 最坏失配=122.9, 宽延迟 L20–80 与
灵敏度 s0.7–1.3 全程零发散。目标: 全面超过它。

## 规则
- 只改你自己的 law 文件; 不要动 core/runner/eval/scenarios/base。
- 必须**真跑 arena 迭代调参**, 不能只给理论。调到该方法自身最优再报告。
- 报告: 最优参数 + 完整电池输出 + 失配鲁棒性剖面 (是否发散/在哪个 L_真) + 已知局限。
