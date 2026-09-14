"""arena/fps_eval.py — FPS 行为库评测组 (急停/跳跃落地/蹬墙跳/变向/…)。

与 arena.eval (标准评测组) 互补: 标准评测组覆盖跟踪/阶跃/延迟失配/帧率,
本组覆盖"FPS 角色行为"维度的瞬态, 核心指标是事件后过冲 over 与
恢复时间 rec (急停/落地/折返正是真实设备上暴露问题的工况)。

每 law 两档传感器:
  clean  drop_p=0
  flaky  drop_p=0.12 (检测闪烁, 快目标常见; 复现"丢帧撞急停"放大器)

输出除事件指标外还有**阶段指标** (metrics.phase_metrics): 滞空/冲刺/滑铲等
持续段的 rmse/mean/p95/max/on_body(±25px)。事件指标 over 是相对口径 (事件前
200ms 中位 |e|), 对"事件前就在持续拖尾"的工况会失明 (典型: 跳跃滞空期准星
一直落后 → pre 已经很大 → over 被压小), 阶段指标补上这一段。

用法: python -m arena.fps_eval [law ...]     (默认 ff_pi_acc reference)
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
        per, evs, phs = [], [], []
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
            phs.extend(M.phase_metrics(res, sc.phases, dist_m=sc.dist_m))
        out[sc.name] = {"track": runner.aggregate(per, "track"), "events": evs,
                        "phases": phs}
    return out


def _ev_stats(events):
    fin = [e for e in events if e["over"] != float("inf")]
    if not fin:
        return float("inf"), float("inf"), float("inf")
    over = [e["over"] for e in fin]
    rec = [e["rec"] for e in fin if e["rec"] != float("inf")]
    return (sum(over) / len(over), max(over),
            sum(rec) / len(rec) if rec else float("inf"))


def test_suite(law_factory, verbose=True):
    summary = {}
    for tag, drop_p in VARIANTS:
        summary[tag] = run_variant(law_factory, drop_p)

    def tot(res):
        evs = [e for sc in res.values() for e in sc["events"]]
        phs = [p for sc in res.values() for p in sc["phases"]]
        mo, wo, mr = _ev_stats(evs)
        rmse = sum(sc["track"]["rmse_px"] for sc in res.values()) / len(res)
        div = any(sc["track"]["diverged"] for sc in res.values())
        air = M.pooled_phase(phs, "air")
        return {"mean_over": mo, "worst_over": wo, "mean_rec": mr,
                "rmse": rmse, "diverged": div, "air": air}

    summary["totals"] = {tag: tot(summary[tag]) for tag, _ in VARIANTS}
    if verbose:
        print_test_suite(law_factory.__name__ if hasattr(law_factory, "__name__")
                      else "law", summary)
    return summary


def print_test_suite(tag, s):
    print(f"\n########## {tag} ##########")
    for vtag, _ in VARIANTS:
        print(f"\n=== FPS suite [{vtag}] ===")
        print(f"  {'scenario':20s} {'rmse':>7} {'max':>7}  {'events':38s} "
              f"{'phase':>34s}")
        for sc, d in s[vtag].items():
            a = d["track"]
            if a["diverged"]:
                print(f"  {sc:20s} DIVERGED")
                continue
            mo, wo, mr = _ev_stats(d["events"])
            ev_s = (f"over={mo:6.1f} worst={wo:6.1f} rec={mr:5.0f}ms"
                    if mo == mo and mo != float("inf") else "ev: n/a")
            ph_s = _phase_str(d.get("phases"))
            print(f"  {sc:20s} rmse={a['rmse_px']:6.2f}px max={a['max_err_px']:6.1f}px  "
                  f"{ev_s:38s} {ph_s:>34s}")
    for vtag, _ in VARIANTS:
        t = s["totals"][vtag]
        air = t.get("air")
        air_s = ("  [air] rmse={:.2f}px mean={:.2f}px max={:.1f}px "
                 "on_body={:.0f}%".format(air["rmse"], air["mean"], air["max"],
                                          air["on_body"] * 100)) if air else ""
        print(f"\n[{vtag}] EVENT_OVER mean={t['mean_over']:.1f}px "
              f"worst={t['worst_over']:.1f}px  REC mean="
              f"{t['mean_rec']:.0f}ms  RMSE mean={t['rmse']:.2f}px"
              + ("  [DIVERGED SOMEWHERE]" if t["diverged"] else "")
              + air_s)


def _phase_str(phases):
    """阶段表: '<kind>: rmse=.. mean=.. max=.. onbody=..%' (多段取段平均)。"""
    if not phases:
        return ""
    kinds = []
    for p in phases:
        if p["kind"] not in kinds:
            kinds.append(p["kind"])
    parts = []
    for kd in kinds:
        agg = M.pooled_phase(phases, kd)
        if agg is None:
            continue
        parts.append(f"{kd}: rmse={agg['rmse']:.1f} mean={agg['mean']:.1f} "
                     f"p95={agg['p95']:.1f} max={agg['max']:.1f} "
                     f"onbody={agg['on_body']*100:.0f}% (n={agg['n_seg']})")
    return "  |  ".join(parts)


def main():
    names = sys.argv[1:] or ["ff_pi_acc", "reference"]
    for name in names:
        cls = get_law(name)
        test_suite(lambda: cls())


if __name__ == "__main__":
    main()
