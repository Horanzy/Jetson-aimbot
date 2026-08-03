"""arena/integrate.py — 阶段4 统合评测。

所有 law 在完全相同的多组场景下:
  1. 标准电池 (arena.eval.battery): 标准多组场景 + 失配扫描 + 帧率。
  2. 重新锁定: 目标瞬间大跳变 (切目标/丢失重锁), 测二次拉枪稳定时间与过冲。
  3. 宽延迟扫描: L_真 ∈ {20..80}, 找发散边界。
  4. 灵敏度失配: s_真=1, s_相信 ∈ {0.7..1.3} (标定误差)。

用法: python3 -m arena.integrate [律名 ...]   (不给名字 = 全部已注册 law)
"""
from __future__ import annotations
import sys
import math
import random
from arena.core import Arena, ArenaConfig
from arena.laws.base import all_laws
from arena.scenarios import Scenario, JumpTarget
from arena import metrics as M
from arena import runner
from arena.eval import battery, NOMINAL_L, NOISE, SEEDS

RELOCK_JUMP_T = 700.0


def relock_scenario():
    return Scenario("relock", 2000.0,
                    lambda rng: JumpTarget(80.0, 0.0, -60.0, 30.0, RELOCK_JUMP_T),
                    (0.0, 0.0), "step", (-1.0, 0.3), settle_band=3.0)


def _relock_metrics(res, sc):
    t, ex, ey = res["t"], res["ex"], res["ey"]
    if res["diverged"]:
        return {"diverged": True, "relock_settle": float("inf"),
                "relock_over": float("inf")}
    ji = 0
    for i in range(len(t)):
        if t[i] >= RELOCK_JUMP_T:
            ji = i
            break
    band = sc.settle_band
    e = [math.hypot(ex[i], ey[i]) for i in range(ji, len(t))]
    tt = t[ji:]
    last_out = tt[0]
    for i, v in enumerate(e):
        if v > band:
            last_out = tt[i]
    relock_settle = last_out - RELOCK_JUMP_T if e[-1] <= band else float("inf")
    # direction-agnostic rebound (consistent with metrics.step_metrics): max |e| after first reach
    first_reach_i = None
    for i, v in enumerate(e):
        if v <= band:
            first_reach_i = i
            break
    relock_over = max(e[first_reach_i:]) if first_reach_i is not None else float("inf")
    return {"diverged": False, "relock_settle": relock_settle,
            "relock_over": relock_over}


def run_relock(law_factory, L_belief=NOMINAL_L, max_v=1.5, seeds=SEEDS,
               L_true=NOMINAL_L):
    sc = relock_scenario()
    out = []
    for sd in seeds:
        ac = ArenaConfig(noise_std=NOISE, fps=120, duration=sc.duration,
                         s_true=1.0, L_true=L_true)
        rng = random.Random(sd)
        tgt = sc.make_target(rng)
        ar = Arena(ac, tgt, rng, cross0=sc.cross0)
        res = ar.run(law_factory(), 1.0, L_belief, max_v)
        out.append(_relock_metrics(res, sc))
    div = any(m["diverged"] for m in out)
    settle = sum(m["relock_settle"] for m in out) / len(out)
    over = sum(m["relock_over"] for m in out) / len(out)
    return {"diverged": div, "relock_settle": settle, "relock_over": over}


def wide_delay_sweep(law_factory, L_belief=NOMINAL_L, max_v=1.5, seeds=(1, 2)):
    suite = [s for s in runner.standard_suite() if s.name in ("step_80px", "maneuver")]
    rows = {}
    for Lt in (20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0):
        r = runner.run_suite(law_factory, suite, ArenaConfig(noise_std=NOISE),
                             s_belief=1.0, L_belief=L_belief, max_v=max_v,
                             seeds=seeds, s_true=1.0, L_true=Lt)
        sc = runner.composite_score(r)
        div = any(d["agg"]["diverged"] for d in r.values())
        rows[Lt] = (sc, div)
    return rows


def s_mismatch(law_factory, L_belief=NOMINAL_L, max_v=1.5, seeds=(1, 2)):
    suite = [s for s in runner.standard_suite() if s.name in ("step_80px", "maneuver")]
    rows = {}
    for sb in (0.7, 0.85, 1.0, 1.15, 1.3):
        r = runner.run_suite(law_factory, suite, ArenaConfig(noise_std=NOISE),
                             s_belief=sb, L_belief=L_belief, max_v=max_v,
                             seeds=seeds, s_true=1.0, L_true=NOMINAL_L)
        sc = runner.composite_score(r)
        div = any(d["agg"]["diverged"] for d in r.values())
        rows[sb] = (sc, div)
    return rows


def full(name, law_factory):
    print(f"\n################## {name} ##################")
    b = battery(law_factory)
    rl = run_relock(law_factory)
    print(f"\n--- 重新锁定 (目标 +80→(-60,30) @700ms) ---")
    if rl["diverged"]:
        print("  DIVERGED")
    else:
        print(f"  二次稳定时间={rl['relock_settle']:.1f}ms  过冲={rl['relock_over']:.2f}px")
    print(f"\n--- 宽延迟扫描 (belief=50) ---")
    wd = wide_delay_sweep(law_factory)
    for Lt, (sc, div) in wd.items():
        print(f"  L_true={Lt:4.0f}  composite={sc:8.2f}  {'DIVERGED' if div else ''}")
    print(f"\n--- 灵敏度失配 (s_true=1.0) ---")
    sm = s_mismatch(law_factory)
    for sb, (sc, div) in sm.items():
        print(f"  s_belief={sb:.2f}  composite={sc:8.2f}  {'DIVERGED' if div else ''}")
    return {"battery": b, "relock": rl, "wide_delay": wd, "s_mismatch": sm}


def main():
    laws = all_laws()
    names = sys.argv[1:] or list(laws.keys())
    results = {}
    for nm in names:
        if nm not in laws:
            print(f"未知 law: {nm}")
            continue
        cls = laws[nm]
        results[nm] = full(nm, lambda c=cls: c())

    print("\n\n================== 总排行榜 (OVERALL, 越小越好) ==================")
    ranked = sorted(results.items(), key=lambda kv: kv[1]["battery"]["overall"])
    print(f"{'law':12s} {'OVERALL':>9s} {'matched':>9s} {'worst_MM':>9s} "
          f"{'relock':>8s} {'fpsΔ%':>6s}")
    for nm, r in ranked:
        b = r["battery"]
        rl = r["relock"]
        rls = f"{rl['relock_settle']:.0f}" if not rl["diverged"] else "DIV"
        print(f"{nm:12s} {b['overall']:9.2f} {b['matched_120']['score']:9.2f} "
              f"{b['mismatch_worst']:9.2f} {rls:>8s} {b['fps_delta']*100:6.1f}")


if __name__ == "__main__":
    main()
