"""arena/recoil_eval.py — 后坐力扰动评测组 (相机被推动: 周期性脉冲串)。

与 arena.eval / arena.fps_eval 并列的第三套工况: 被控对象一侧多一个**外部扰动**
(arena/recoil.py), 目标本身静止 —— 于是每一段指令位移都是对后坐力的补偿, 可以
按发对齐地看"这一段位移是怎么发出去的"。

报告的核心不是"准星离目标多远" (那是标准口径), 而是:
    fill90 / iqr    每发交付位移的时间分位点: 90% 分位占间隔的比例 (匀速摊满 =
                    0.90) 与 25↔75 跨度 (匀速 = 0.50)。脉冲式压枪两个数都塌下来
                    —— "一顿一顿"的直接量化 (分位点口径, 无阈值)
    steps_per_shot  每发引起多少次指令速度台阶 (阈值 = 帧分辨率下限与信号稳健
                    尺度取大, 见 metrics.recoil_metrics)
    rev_per_s       实际交付位移的方向翻转次数/秒 (压过头再拉回来的往复)
    acc_rms         指令速度变化率 RMS (px/ms², 高频含量)
    over_y          每发间隔内 ey 的正峰值 = 压过头多少 px
    on_body/rmse_y  精度代价 (躯干半宽随距离缩放, 与 phase_metrics 同口径)

每 law 两档传感器 (与 fps_eval 同口径):
  clean  drop_p=0
  flaky  drop_p=0.12 (检测闪烁)

用法: python -m arena.recoil_eval [law ...]        (默认 ff_pi_acc reference)
      python -m arena.recoil_eval --png out.png [--png-scenario 名字]
"""
from __future__ import annotations
import sys
import random
from arena.core import Arena, ArenaConfig
from arena import metrics as M
from arena.recoil import recoil_suite
from arena.laws.base import get_law

NOISE = 0.5
SEEDS = (1, 2, 3)
NOMINAL_L = 50.0
MAX_V = 1.5
VARIANTS = (("clean", 0.0), ("flaky", 0.12))


def run_one(law_factory, sc, drop_p, seed):
    rng = random.Random(seed)
    tgt = sc.make_target(rng)
    cfg = ArenaConfig(noise_std=NOISE, fps=120, duration=sc.duration,
                      drop_p=drop_p, disturbance=sc.recoil)
    ar = Arena(cfg, tgt, rng, cross0=sc.cross0)
    return ar.run(law_factory(), s_belief=1.0, L_belief=NOMINAL_L, max_v=MAX_V)


def aggregate(per):
    keys = sorted(per[0])
    agg = {}
    for k in keys:
        vals = [m[k] for m in per]
        agg[k] = (float("inf") if any(v == float("inf") for v in vals)
                  else sum(vals) / len(vals))
    return agg


def run_variant(law_factory, drop_p):
    out = {}
    for sc in recoil_suite():
        per = [M.recoil_metrics(run_one(law_factory, sc, drop_p, sd), sc)
               for sd in SEEDS]
        out[sc.name] = {"agg": aggregate(per), "sc": sc}
    return out


def _fmt(v, w=7, p=2):
    if v == float("inf"):
        return f"{'inf':>{w}}"
    return f"{v:{w}.{p}f}"


def print_variant(tag, res):
    print(f"\n=== 后坐力族 [{tag}] ===")
    print(f"  {'combo':22s}{'n':>3}{'rmse_y':>8}{'mean|y|':>8}{'over_y':>8}"
          f"{'on_body':>8}{'steps/s':>9}{'st/shot':>9}{'rev/s':>8}"
          f"{'acc_rms':>9}{'fill90':>8}{'iqr':>7}{'rec_ms':>8}")
    for name, d in res.items():
        a = d["agg"]
        print(f"  {name:22s}{int(a['n_shots']):>3}"
              + "".join(_fmt(a[k]) for k in
                        ("rmse_y", "mean_abs_y", "over_y"))
              + f"{a['on_body'] * 100:7.0f}%"
              + _fmt(a["steps_per_s"], 9)
              + _fmt(a["steps_per_shot"], 9)
              + _fmt(a["rev_per_s"], 8) + _fmt(a["acc_rms"], 9)
              + _fmt(a["fill90"], 8, 3) + _fmt(a["iqr"], 7, 3)
              + _fmt(a["rec_ms"], 8, 0))


def totals(res):
    """逐组合等权平均 (发散组合 → inf, 汇总随之 inf/带标记)。"""
    rows = [d["agg"] for d in res.values()]
    t = M.pooled_recoil(rows)
    t["diverged"] = any(a["rmse_y"] == float("inf") for a in rows)
    return t


def test_suite(law_factory, tag=None, verbose=True):
    tag = tag or (law_factory.__name__ if hasattr(law_factory, "__name__")
                  else "law")
    summary = {"law": tag}
    for vtag, dp in VARIANTS:
        summary[vtag] = run_variant(law_factory, dp)
    if verbose:
        print(f"\n########## {tag} ##########")
        for vtag, _ in VARIANTS:
            print_variant(vtag, summary[vtag])
        for vtag, _ in VARIANTS:
            t = totals(summary[vtag])
            print(f"[{vtag}] 组合均值: fill90={t['fill90']:.3f} iqr={t['iqr']:.3f} "
                  f"st/shot={t['steps_per_shot']:.2f} rev/s={t['rev_per_s']:.2f} "
                  f"acc_rms={t['acc_rms']:.3f} over_y={t['over_y']:.2f}px "
                  f"on_body={t['on_body']*100:.0f}% "
                  f"rmse_y={t['rmse_y']:.2f}px rec={t['rec_ms']:.0f}ms"
                  + ("  [DIVERGED SOMEWHERE]" if t["diverged"] else ""))
    return summary


def _aligned(res, sc, grid):
    """每发对齐的累计补偿位移 (向下压为正), 重采样到统一网格。"""
    import numpy as np
    t, cy = res["t"], res["sent_cy"]
    s = res.get("s_true", 1.0)
    interval = sc.interval_ms
    curves = []
    for ts in sc.shots:
        xs, ys, a = [], [], 0.0
        for i, tt in enumerate(t):
            if ts <= tt < ts + interval:
                a += cy[i] * s
                xs.append(tt - ts)
                ys.append(-a)
        if len(xs) > 1:
            curves.append(np.interp(grid, xs, ys))
    return curves


def write_png(path, law_names, png_scenario=None):
    """每发对齐的响应曲线叠加图 (每个 law 一色, 细线 = 单发, 粗线 = 发间中位)。
    需要 matplotlib; 缺失时跳过 (不为此新增依赖)。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("  (无 matplotlib, 跳过 PNG; 文本表格已足够)")
        return False
    suite = {sc.name: sc for sc in recoil_suite()}
    if png_scenario is None:
        # 数据驱动选默认场景: clean 档下 fill90 最低 (响应最集中 = 最"一顿一顿")
        best, best_v = None, float("inf")
        for nm, sc in suite.items():
            vals = [M.recoil_metrics(run_one(get_law(law_names[0]), sc, 0.0, sd),
                                     sc)["fill90"] for sd in SEEDS]
            v = sum(vals) / len(vals)
            if v < best_v:
                best, best_v = nm, v
        png_scenario = best
    sc = suite[png_scenario]
    grid = np.linspace(0.0, sc.interval_ms, 60)
    fig, ax = plt.subplots(figsize=(9, 5.5))
    for name in law_names:
        cls = get_law(name)
        curves = _aligned(run_one(cls, sc, 0.0, SEEDS[0]), sc, grid)
        if not curves:
            continue
        for c in curves:
            ax.plot(grid, c, lw=0.6, alpha=0.25)
        med = np.median(np.array(curves), axis=0)
        ax.plot(grid, med, lw=2.2, label=f"{name} (n={len(curves)})")
    ax.axhline(0.0, color="k", lw=0.5, alpha=0.4)
    ax.set_xlabel("距该发的时刻 (ms)")
    ax.set_ylabel("累计补偿位移 (px, 向下压为正)")
    ax.set_title(f"每发对齐的压枪响应 @ {sc.name} "
                 f"(每发 {sc.kick_eff_px:.0f}px / 间隔 {sc.interval_ms:.0f}ms, clean)")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print(f"  PNG: {path}  (场景 {sc.name})")
    return True


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    png = None
    png_scenario = None
    if "--png" in argv:
        i = argv.index("--png")
        png = argv[i + 1] if len(argv) > i + 1 else "recoil_aligned.png"
        argv = argv[:i] + argv[i + 2:]
    if "--png-scenario" in argv:
        i = argv.index("--png-scenario")
        png_scenario = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]
    names = argv or ["ff_pi_acc", "reference"]
    if any(n.startswith("-") for n in names):
        raise SystemExit("用法: python -m arena.recoil_eval [law ...] "
                         "[--png out.png] [--png-scenario 名字]")
    summary = {}
    for name in names:
        cls = get_law(name)
        summary[name] = test_suite(lambda: cls(), tag=name)
    if len(names) > 1:
        print("\n########## 汇总 (逐组合等权平均) ##########")
        print(f"  {'law':14s}{'fill90':>8}{'iqr':>8}{'st/shot':>9}{'rev/s':>8}"
              f"{'acc_rms':>9}{'over_y':>8}{'on_body':>9}{'rmse_y':>8}"
              f"{'rec_ms':>8}")
        for vtag, _ in VARIANTS:
            print(f"  -- [{vtag}] --")
            for name in names:
                t = totals(summary[name][vtag])
                print(f"  {name:14s}{_fmt(t['fill90'], 8, 3)}{_fmt(t['iqr'], 8, 3)}"
                      f"{_fmt(t['steps_per_shot'], 9)}{_fmt(t['rev_per_s'], 8)}"
                      f"{_fmt(t['acc_rms'], 9)}{_fmt(t['over_y'], 8)}"
                      f"{t['on_body'] * 100:8.0f}%"
                      f"{_fmt(t['rmse_y'], 8)}{_fmt(t['rec_ms'], 8, 0)}")
    if png:
        write_png(png, names, png_scenario)


if __name__ == "__main__":
    main()
