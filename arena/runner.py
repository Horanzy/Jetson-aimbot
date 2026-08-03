"""arena/runner.py — 在标准多组场景下运行 law, 计算指标 / 综合评分 / 排行榜。

综合目标: 同时权衡 速度(稳定时间/首达) / 过冲 / 跟踪误差 / 鲁棒性(失配发散重罚)。
"""
from __future__ import annotations
import math
import random
from arena.core import Arena, ArenaConfig
from arena import metrics as M
from arena.scenarios import standard_suite


def run_one(law, scenario, arena_cfg, s_belief, L_belief, max_v, seed):
    rng = random.Random(seed)
    tgt = scenario.make_target(rng)
    ar = Arena(arena_cfg, tgt, rng, cross0=scenario.cross0)
    res = ar.run(law, s_belief, L_belief, max_v)
    return M.compute(res, scenario), res


def run_suite(law_factory, scenarios=None, arena_cfg=None, s_belief=1.0,
              L_belief=50.0, max_v=1.5, seeds=(1, 2, 3), s_true=1.0,
              L_true=50.0):
    """law_factory: callable() -> Law (每次新建, 避免状态串扰)。
    返回 {scenario_name: {"metrics": [...per seed], "agg": {...}}}。"""
    scenarios = scenarios or standard_suite()
    arena_cfg = arena_cfg or ArenaConfig()
    out = {}
    for sc in scenarios:
        per = []
        for sd in seeds:
            ac = ArenaConfig(s_true=s_true, L_true=L_true, h=arena_cfg.h,
                             fps=arena_cfg.fps, noise_std=arena_cfg.noise_std,
                             duration=sc.duration, count_limit=arena_cfg.count_limit,
                             fov_radius=arena_cfg.fov_radius)
            m, _ = run_one(law_factory(), sc, ac, s_belief, L_belief, max_v, sd)
            per.append(m)
        out[sc.name] = {"metrics": per, "agg": _aggregate(per, sc.kind)}
    return out


def _aggregate(per, kind):
    div = any(m["diverged"] for m in per)
    agg = {"diverged": div}
    if kind == "step":
        keys = ["settle_ms", "overshoot_px", "first_reach_ms", "final_err_px"]
    else:
        keys = ["rmse_px", "mean_err_px", "in_band_frac", "max_err_px"]
    if div:
        for k in keys:
            agg[k] = 0.0 if k == "in_band_frac" else float("inf")
        return agg
    for k in keys:
        vals = [m[k] for m in per]
        agg[k] = sum(vals) / len(vals)
    return agg


def composite_score(suite_result, weights=None):
    """越小越好。发散 → 巨大罚分。"""
    w = weights or {}
    w_settle = w.get("settle", 1.0)
    w_over = w.get("over", 1.5)
    w_rmse = w.get("rmse", 2.0)
    w_inband = w.get("inband", 1.0)
    DIV_PENALTY = 1e6

    score = 0.0
    n_step = n_track = 0
    for name, d in suite_result.items():
        a = d["agg"]
        if a["diverged"]:
            score += DIV_PENALTY
            continue
        if "settle_ms" in a:
            score += w_settle * a["settle_ms"] / 100.0
            score += w_over * a["overshoot_px"]
            score += 0.3 * a["first_reach_ms"] / 100.0
            n_step += 1
        else:
            score += w_rmse * a["rmse_px"]
            score += w_inband * (1.0 - a["in_band_frac"]) * 50.0
            n_track += 1
    return score


def print_suite(name, suite_result, score):
    print(f"\n=== {name}  (composite={score:.2f}) ===")
    for sc, d in suite_result.items():
        a = d["agg"]
        if a["diverged"]:
            print(f"  {sc:18s} DIVERGED")
            continue
        if "settle_ms" in a:
            print(f"  {sc:18s} settle={a['settle_ms']:7.1f}ms over="
                  f"{a['overshoot_px']:6.2f}px first={a['first_reach_ms']:7.1f}ms "
                  f"final={a['final_err_px']:.2f}px")
        else:
            print(f"  {sc:18s} rmse={a['rmse_px']:6.2f}px mean={a['mean_err_px']:6.2f}px "
                  f"in_band={a['in_band_frac']*100:5.1f}% max={a['max_err_px']:.1f}px")
