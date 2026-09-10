"""arena/laws/base.py — law 接口 + 注册表 + 共享工具。

law 是黑盒: 自己保管检测历史与指令历史, 自己估计/预测/算指令/量化。
实现 Law 接口 + @register 即可接入; 多种 law 可并排测试。
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
from arena.core import Observation, LawConfig

_REGISTRY: dict[str, type] = {}


def register(name: str):
    def deco(cls):
        cls.name = name
        _REGISTRY[name] = cls
        return cls
    return deco


def get_law(name: str) -> type:
    return _REGISTRY[name]


def all_laws() -> dict[str, type]:
    return dict(_REGISTRY)


class CountsHist:
    """累计 counts 历史: (t, 累计cx, 累计cy) 环形缓冲 + 任意时刻线性插值。

    各 law 的共用基础设施: Smith 预测的"在途 counts"扣除与指令历史回放
    都靠它按时间戳取值。"""
    __slots__ = ("t", "cx", "cy", "cumx", "cumy")

    MAX_LEN = 2000

    def __init__(self):
        self.t: list[float] = []
        self.cx: list[float] = []
        self.cy: list[float] = []
        self.cumx = 0.0
        self.cumy = 0.0

    def add(self, t, dx, dy):
        self.cumx += dx
        self.cumy += dy
        self.t.append(t)
        self.cx.append(self.cumx)
        self.cy.append(self.cumy)
        if len(self.t) > self.MAX_LEN:
            self.t.pop(0); self.cx.pop(0); self.cy.pop(0)

    def at(self, t):
        ts = self.t
        n = len(ts)
        if n == 0:
            return 0.0, 0.0
        if t <= ts[0]:
            return self.cx[0], self.cy[0]
        if t >= ts[-1]:
            return self.cx[-1], self.cy[-1]
        lo, hi = 0, n - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ts[mid] <= t:
                lo = mid
            else:
                hi = mid
        span = ts[hi] - ts[lo]
        f = (t - ts[lo]) / span if span > 0 else 0.0
        return (self.cx[lo] + (self.cx[hi] - self.cx[lo]) * f,
                self.cy[lo] + (self.cy[hi] - self.cy[lo]) * f)

    def cum(self):
        return self.cumx, self.cumy


class Law:
    name = "base"

    def reset(self, cfg: LawConfig) -> None:
        self.cfg = cfg

    def step(self, t: float, obs: Optional[Observation]) -> Tuple[int, int]:
        raise NotImplementedError

    @staticmethod
    def _counts(rem_x: float, rem_y: float, s: float, limit: int):
        """余数累加 + 量化 + 限幅。返回 (cx, cy, 新rem_x, 新rem_y)。"""
        sx = math.trunc(rem_x)
        sy = math.trunc(rem_y)
        sx = max(-limit, min(limit, sx))
        sy = max(-limit, min(limit, sy))
        return sx, sy, rem_x - sx, rem_y - sy
