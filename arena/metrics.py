"""arena/metrics.py — 由真值序列计算指标。

step 场景: 拉枪稳定时间 (快) + 过冲 (小) + 首次到达时间。
track 场景: 稳态 RMSE / 平均|e| / 误差<阈值时间占比。
发散: 单独标记, 综合评分重罚。
"""
from __future__ import annotations
import math


def _abs_err(ex, ey):
    return [math.hypot(a, b) for a, b in zip(ex, ey)]


def step_metrics(res, scenario):
    t, ex, ey = res["t"], res["ex"], res["ey"]
    if res["diverged"]:
        return {"diverged": True, "settle_ms": float("inf"),
                "overshoot_px": float("inf"), "first_reach_ms": float("inf"),
                "final_err_px": float("inf")}
    band = scenario.settle_band
    e = _abs_err(ex, ey)
    first_reach = float("inf")
    first_reach_i = None
    for i, v in enumerate(e):
        if v <= band:
            first_reach = t[i]
            first_reach_i = i
            break
    last_out = 0.0
    for i, v in enumerate(e):
        if v > band:
            last_out = t[i]
    settle = last_out if e[-1] <= band else float("inf")
    # direction-agnostic rebound: after first reaching the target, max |e| deviation (any direction)
    overshoot = max(e[first_reach_i:]) if first_reach_i is not None else float("inf")
    return {"diverged": False, "settle_ms": settle, "overshoot_px": overshoot,
            "first_reach_ms": first_reach, "final_err_px": e[-1]}


def track_metrics(res, scenario):
    t, ex, ey = res["t"], res["ex"], res["ey"]
    if res["diverged"]:
        return {"diverged": True, "rmse_px": float("inf"),
                "mean_err_px": float("inf"), "in_band_frac": 0.0,
                "max_err_px": float("inf")}
    sf = scenario.steady_from
    band = scenario.settle_band
    xs, ys = [], []
    for i in range(len(t)):
        if t[i] >= sf:
            xs.append(ex[i]); ys.append(ey[i])
    if not xs:
        xs, ys = ex, ey
    e = _abs_err(xs, ys)
    n = len(e)
    rmse = math.sqrt(sum(v * v for v in e) / n)
    mean = sum(e) / n
    in_band = sum(1 for v in e if v <= band) / n
    return {"diverged": False, "rmse_px": rmse, "mean_err_px": mean,
            "in_band_frac": in_band, "max_err_px": max(e)}


def compute(res, scenario):
    if scenario.kind == "step":
        return step_metrics(res, scenario)
    return track_metrics(res, scenario)
