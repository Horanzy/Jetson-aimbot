"""arena/fps_eval.py — FPS 行为库评测电池 (急停/跳跃落地/蹬墙跳/变向/…)。

与 arena.eval (标准电池) 互补: 标准电池覆盖跟踪/阶跃/延迟失配/帧率,
本电池覆盖"FPS 角色行为"维度的瞬态, 核心指标是事件后过冲 over 与
恢复时间 rec (急停/落地/折返正是真实设备上暴露问题的工况)。

每 law 两档传感器:
  clean  drop_p=0
  flaky  drop_p=0.12 (检测闪烁, 快目标常见; 复现"丢帧撞急停"放大器)

用法: python -m arena.fps_eval [law ...]     (默认 ff_pi reference)
"""
from __future__ import annotations
import sys
import random
from arena.core import Arena, ArenaConfig
from arena import metrics as M
from arena import runner
from arena.fps import fps_suite
from arena.laws.base import get_law

NOISE = 0.5
SEEDS = (1, 2, 3)
NOMINAL_L = 50.0
MAX_V = 1.5
VARIANTS = (("clean", 0.0), ("flaky", 0.12))


def run_variant(law_factory, drop_p):
    out = {}
    for sc in fps_suite():
        per, evs = [], []
        for sd in SEEDS:
            rng = random.Random(sd)
            tgt = sc.make_target(rng)
            cfg = ArenaConfig(noise_std=NOISE, fps=120, duration=sc.duration,
                              drop_p=drop_p)
            ar = Arena(cfg, tgt, rng, cross0=sc.cross0)
            res = ar.run(law_factory(), s_belief=1.0, L_belief=NOMINAL_L,
                         max_v=MAX_V)
            per.append(M.compute(res, sc))
            evs.extend(M.event_metrics(res, sc.events))
        out[sc.name] = {"track": runner._aggregate(per, "track"), "events": evs}
    return out


def _ev_stats(events):
    fin = [e for e in events if e["over"] != float("inf")]
    if not fin:
        return float("inf"), float("inf"), float("inf")
    over = [e["over"] for e in fin]
    rec = [e["rec"] for e in fin if e["rec"] != float("inf")]
    return (sum(over) / len(over), max(over),
            sum(rec) / len(rec) if rec else float("inf"))


def battery(law_factory, verbose=True):
    summary = {}
    for tag, drop_p in VARIANTS:
        summary[tag] = run_variant(law_factory, drop_p)

    def tot(res):
        evs = [e for sc in res.values() for e in sc["events"]]
        mo, wo, mr = _ev_stats(evs)
        rmse = sum(sc["track"]["rmse_px"] for sc in res.values()) / len(res)
        div = any(sc["track"]["diverged"] for sc in res.values())
        return {"mean_over": mo, "worst_over": wo, "mean_rec": mr,
                "rmse": rmse, "diverged": div}

    summary["totals"] = {tag: tot(summary[tag]) for tag, _ in VARIANTS}
    if verbose:
        print_battery(law_factory.__name__ if hasattr(law_factory, "__name__")
                      else "law", summary)
    return summary


def print_battery(tag, s):
    print(f"\n########## {tag} ##########")
    for vtag, _ in VARIANTS:
        print(f"\n=== FPS suite [{vtag}] ===")
        for sc, d in s[vtag].items():
            a = d["track"]
            if a["diverged"]:
                print(f"  {sc:20s} DIVERGED")
                continue
            mo, wo, mr = _ev_stats(d["events"])
            ev_s = (f"ev_over={mo:6.1f}px worst={wo:6.1f}px rec={mr:6.0f}ms"
                    if mo == mo and mo != float("inf") else "ev: n/a")
            print(f"  {sc:20s} rmse={a['rmse_px']:6.2f}px max={a['max_err_px']:6.1f}px  {ev_s}")
    for vtag, _ in VARIANTS:
        t = s["totals"][vtag]
        print(f"\n[{vtag}] EVENT_OVER mean={t['mean_over']:.1f}px "
              f"worst={t['worst_over']:.1f}px  REC mean="
              f"{t['mean_rec']:.0f}ms  RMSE mean={t['rmse']:.2f}px"
              + ("  [DIVERGED SOMEWHERE]" if t["diverged"] else ""))


def main():
    names = sys.argv[1:] or ["ff_pi", "reference"]
    for name in names:
        cls = get_law(name)
        battery(lambda: cls())


if __name__ == "__main__":
    main()
