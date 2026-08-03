"""arena/eval.py — 标准化评测电池 (所有 law 用完全相同的多组场景)。

电池:
  A. 标准多组场景 (step x2 / 匀速 / 匀加速 / 随机机动), 匹配延迟 L=50, 120fps, 噪声。
  B. 延迟失配扫描: 固定 law 相信 L_belief=50 (模拟标定读数), 真值 L_true ∈ {30..70};
     跑 step + maneuver 代表场景, 报告综合分与是否发散 → 最坏表现 = 鲁棒性。
  C. 帧率无关性: 60 vs 120fps 匹配延迟, 报告差异。

用法: python3 -m arena.eval <law名字> [L_belief]
"""
from __future__ import annotations
import sys
from arena.core import ArenaConfig
from arena.laws.base import get_law
from arena.scenarios import standard_suite, Scenario
from arena import runner

NOMINAL_L = 50.0
NOISE = 0.5
SEEDS = (1, 2, 3)
L_SWEEP = (30.0, 40.0, 50.0, 60.0, 70.0)


def _suite_cfg(fps=120):
    return ArenaConfig(noise_std=NOISE, fps=fps)


def battery(law_factory, L_belief=NOMINAL_L, max_v=1.5, verbose=True):
    suite = standard_suite()
    rep = [s for s in suite if s.name in ("step_80px", "maneuver")]
    summary = {}

    # A. 匹配延迟, 120fps, 全场景
    resA = runner.run_suite(law_factory, suite, _suite_cfg(120),
                            s_belief=1.0, L_belief=L_belief, max_v=max_v,
                            seeds=SEEDS, s_true=1.0, L_true=NOMINAL_L)
    scoreA = runner.composite_score(resA)
    summary["matched_120"] = {"score": scoreA, "result": resA}

    # B. 延迟失配扫描 (代表场景)
    sweep = {}
    worst = -1e9
    for Lt in L_SWEEP:
        r = runner.run_suite(law_factory, rep, _suite_cfg(120),
                             s_belief=1.0, L_belief=L_belief, max_v=max_v,
                             seeds=SEEDS, s_true=1.0, L_true=Lt)
        sc = runner.composite_score(r)
        div = any(d["agg"]["diverged"] for d in r.values())
        sweep[Lt] = {"score": sc, "diverged": div, "result": r}
        worst = max(worst, sc)
    summary["mismatch_sweep"] = sweep
    summary["mismatch_worst"] = worst

    # C. 帧率无关性
    res60 = runner.run_suite(law_factory, suite, _suite_cfg(60),
                             s_belief=1.0, L_belief=L_belief, max_v=max_v,
                             seeds=SEEDS, s_true=1.0, L_true=NOMINAL_L)
    score60 = runner.composite_score(res60)
    summary["matched_60"] = {"score": score60, "result": res60}
    summary["fps_delta"] = abs(score60 - scoreA) / max(scoreA, 1e-6)

    # 总评: 匹配性能 + 失配最坏 (重罚) 的折中
    summary["overall"] = 0.5 * scoreA + 0.5 * worst + 200.0 * summary["fps_delta"]
    if verbose:
        print_battery(law_factory.__name__ if hasattr(law_factory, "__name__") else "law",
                      summary)
    return summary


def print_battery(tag, s):
    print(f"\n########## {tag} ##########")
    runner.print_suite("A: matched L=50, 120fps", s["matched_120"]["result"],
                       s["matched_120"]["score"])
    print(f"\n--- B: 延迟失配扫描 (belief=50) ---")
    for Lt, d in s["mismatch_sweep"].items():
        flag = "DIVERGED" if d["diverged"] else ""
        print(f"  L_true={Lt:4.0f}ms  composite={d['score']:8.2f}  {flag}")
    print(f"  最坏 composite = {s['mismatch_worst']:.2f}")
    print(f"\n--- C: 帧率无关性 ---")
    print(f"  60fps composite={s['matched_60']['score']:.2f}  "
          f"120fps={s['matched_120']['score']:.2f}  相对差={s['fps_delta']*100:.1f}%")
    print(f"\n  >>> OVERALL = {s['overall']:.2f}  (越小越好)")


def main():
    if len(sys.argv) < 2:
        print("用法: python3 -m arena.eval <law名字> [L_belief]")
        return
    name = sys.argv[1]
    Lb = float(sys.argv[2]) if len(sys.argv) > 2 else NOMINAL_L
    cls = get_law(name)
    battery(lambda: cls(), L_belief=Lb)


if __name__ == "__main__":
    main()
