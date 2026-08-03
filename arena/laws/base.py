"""arena/laws/base.py — law 接口 + 注册表。

law 是黑盒: 自己保管检测历史与指令历史, 自己估计/预测/算指令/量化。
实现 Law 接口 + @register 即可接入; 多种 law 可并排测试。
"""
from __future__ import annotations
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


class Law:
    name = "base"

    def reset(self, cfg: LawConfig) -> None:
        self.cfg = cfg

    def step(self, t: float, obs: Optional[Observation]) -> Tuple[int, int]:
        raise NotImplementedError

    @staticmethod
    def _counts(rem_x: float, rem_y: float, s: float, limit: int):
        """余数累加 + 量化 + 限幅。返回 (cx, cy, 新rem_x, 新rem_y)。"""
        import math
        sx = math.trunc(rem_x)
        sy = math.trunc(rem_y)
        sx = max(-limit, min(limit, sx))
        sy = max(-limit, min(limit, sy))
        return sx, sy, rem_x - sx, rem_y - sy
