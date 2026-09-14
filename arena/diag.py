"""arena/diag.py — 单场景"误差归因"诊断 (纯观察, 与 trace 同一性质)。

动机: 聚合终值只说"差多少", 不说"差在哪一层"。本模块跑**同一条 law 的两个版本**
做对照, 把误差拆成两个可操作的部分:

    law      原样
    oracle   只把 law 的**速度状态**换成真实速度 (取采集时刻 t-L_true 的真值,
             即该 law 信息集里最"新"的世界状态 —— 完美**因果**估计器能推断出的
             最好结果; 取 t 时刻真值等于偷看未来机动, 不是因果下界), 其余一切
             (估计器其余部分 / 门控 / 控制律 / 量化) 完全不动

于是同一工况下:
    oracle 残差  = 延迟 + 环路带宽 + 前馈结构决定的下界 (估计器再好也去不掉)
    law − oracle = **速度估计质量**贡献的可改进部分 (est_share = 占比)

est_share 高 → 瓶颈在速度估计, 换估计器有空间; est_share 低而 oracle 残差高 →
瓶颈在延迟/环路/执行器饱和, 改估计器没用 (例如冲刺段的爆发速度超过速度帽时,
oracle 也一样贴不上)。

覆写的字段默认取 ff_pi 家族的 α-β 速度估计 (fvx/fvy), 用 --v-fields 改。

中立性: 只在 diag 路径生效 —— 真值用代理包住 Target 记录 (core/runner 一行未动),
monkeypatch 只发生在本模块新建的 law 实例上。eval/integrate/fps_eval 不 import 本模块。

用法:
  python -m arena.diag <law> [场景 ...] [--L-true 50] [--noise 0.5] [--drop 0]
  python -m arena.diag ff_pi_acc fps_jump_land_stop fps_jump_3m
  python -m arena.diag ff_pi_acc --suite       # 标准评测组的 composite 对照 (慢)
  python -m arena.diag ff_pi_acc --list        # 场景名列表
"""
from __future__ import annotations
import argparse
import math
import random
import sys

from arena.core import Arena, ArenaConfig
from arena import metrics as M
from arena.laws.base import get_law
from arena.trace import find_scenario, scenario_names

DEFAULT_SCENARIOS = ("fps_jump_land_stop", "fps_jump_5m", "fps_jump_3m",
                     "fps_wall_bounce", "fps_bhop", "fps_dash", "maneuver")
NOISE = 0.5
NOMINAL_L = 50.0
MAX_V = 1.5
SEEDS = (1, 2, 3)
FPS = 120


class _TruthTarget:
    """真值代理: 转发 advance 并记录轨迹 (不改变被控对象行为)。"""

    def __init__(self, inner):
        self.inner = inner
        self.samples = []

    @property
    def x(self):
        return self.inner.x

    @property
    def y(self):
        return self.inner.y

    def advance(self, h, t):
        self.inner.advance(h, t)
        self.samples.append((t + h, self.inner.x, self.inner.y))


def _sample(samples, idx, t):
    if t <= samples[0][0]:
        return samples[0][idx]
    if t >= samples[-1][0]:
        return samples[-1][idx]
    lo, hi = 0, len(samples) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if samples[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    s0, s1 = samples[lo], samples[hi]
    span = s1[0] - s0[0]
    f = (t - s0[0]) / span if span > 0 else 0.0
    return s0[idx] + (s1[idx] - s0[idx]) * f


def truth_vel(samples, axis, t, half_ms):
    """真值速度 (px/ms): 位置中心差分, 差分宽度取一个帧周期 (half_ms = T/2),
    与传感器自身的积分尺度一致。

    必须差分位置而不是读 Target 的 .vx/.vy —— fps 目标类的 .vx/.vy 只含 strafe
    分量, 跳跃弧在位置偏移里。
    """
    i = 1 if axis == "x" else 2
    return (_sample(samples, i, t + half_ms)
            - _sample(samples, i, t - half_ms)) / (2.0 * half_ms)


class _OracleLaw:
    """诊断代理: 每帧把 law 的速度状态覆写成真实延迟速度。

    只 monkeypatch 本代理新建的 law 实例的 _update_filter, 因此评测路径里的
    law 完全不受影响。
    """

    def __init__(self, law, truth, v_fields, delay, half_ms):
        self.law = law
        self.truth = truth
        self.delay = delay
        self.half_ms = half_ms
        orig = law._update_filter

        def patched(det):
            orig(det)
            if det.t - self.delay < self.truth[0][0]:
                return
            setattr(law, v_fields[0],
                    truth_vel(self.truth, "x", det.t - self.delay, self.half_ms))
            setattr(law, v_fields[1],
                    truth_vel(self.truth, "y", det.t - self.delay, self.half_ms))

        law._update_filter = patched

    def reset(self, cfg):
        self.law.reset(cfg)

    def step(self, t, obs):
        return self.law.step(t, obs)


def _run(law_name, sc, *, L_true, L_belief, noise, drop_p, s_true, s_belief,
         max_v, seed, oracle, v_fields):
    rng = random.Random(seed)
    tgt = sc.make_target(rng)
    rec = _TruthTarget(tgt)
    cfg = ArenaConfig(s_true=s_true, L_true=L_true, fps=FPS, noise_std=noise,
                      duration=sc.duration, drop_p=drop_p)
    ar = Arena(cfg, rec, rng, cross0=sc.cross0)
    law = get_law(law_name)()
    if oracle:
        # 真值取 **采集时刻** t-L_true: 完美因果估计器的信息边界。L̂≠L_true
        # 时也按 L_true 取, 与 belief 无关。
        law = _OracleLaw(law, rec.samples, v_fields, L_true, 500.0 / FPS)
    res = ar.run(law, s_belief, L_belief, max_v)
    return res, rec.samples


def _seg_stats(res, t0, t1, band, dist_m):
    t, ex, ey = res["t"], res["ex"], res["ey"]
    e = [math.hypot(a, b) for a, b in zip(ex, ey)]
    seg = [v for tt, v in zip(t, e) if t0 <= tt <= t1]
    if res["diverged"]:
        return {"n": len(seg), "rmse": float("inf"), "mean": float("inf"),
                "p95": float("inf"), "max": float("inf"), "on_body": 0.0}
    if not seg:
        return {"n": 0, "rmse": 0.0, "mean": 0.0, "p95": 0.0, "max": 0.0,
                "on_body": 0.0}
    n = len(seg)
    ss = sorted(seg)
    return {"n": n, "rmse": math.sqrt(sum(v * v for v in seg) / n),
            "mean": sum(seg) / n, "p95": ss[min(n - 1, int(0.95 * n))],
            "max": ss[-1],
            "on_body": sum(1 for v in seg if v <= M.body_px(dist_m)) / n}


def diag_scenario(law_name, sc, **kw):
    v_fields = kw.pop("v_fields", ("fvx", "fvy"))
    band = sc.settle_band
    acc = {}
    for tag, oracle in (("law", False), ("oracle", True)):
        rows = []
        for sd in SEEDS:
            try:
                rows.append(_run(law_name, sc, oracle=oracle, v_fields=v_fields,
                                 seed=sd, **kw))
            except AttributeError:
                rows = None
                break
        acc[tag] = rows
    if acc["oracle"] is None:
        del acc["oracle"]
    segs = [(kd, t0, t1) for t0, t1, kd in (sc.phases or ())]
    segs.append(("all", 0.0, sc.duration))
    out = []
    for name, t0, t1 in segs:
        row = {"phase": name, "t0": t0, "t1": t1}
        for tag, rows in acc.items():
            if rows is None:
                continue
            st = [_seg_stats(r, t0, t1, band, sc.dist_m) for r, _ in rows]
            row[tag] = {k: sum(s[k] for s in st) / len(st) for k in st[0]}
        if "oracle" in row and row["law"]["rmse"] > 0:
            row["est_share"] = max(0.0, 1.0 - row["oracle"]["rmse"]
                                   / row["law"]["rmse"])
        out.append(row)
    return out


def print_diag(law_name, sc, rows, kw):
    print(f"\n=== diag {law_name} @ {sc.name} (d={sc.dist_m:g}m, "
          f"命中带 {M.body_px(sc.dist_m):.0f}px)  "
          f"(L_true={kw['L_true']:.0f} L_belief={kw['L_belief']:.0f} "
          f"noise={kw['noise']} drop={kw['drop_p']} seeds={len(SEEDS)}) ===")
    has_or = any("oracle" in r for r in rows)
    if has_or:
        print(f"  {'phase':8s} {'n':>5s} {'law rmse':>9s} {'oracle':>8s} "
              f"{'est_share':>10s} {'law max':>8s} {'orc max':>8s} "
              f"{'law onbody':>11s} {'orc onbody':>11s}")
    else:
        print(f"  {'phase':8s} {'n':>5s} {'rmse':>9s} {'mean':>8s} {'p95':>8s} "
              f"{'max':>8s} {'onbody':>8s}")
    for r in rows:
        la = r.get("law")
        if la is None:
            continue
        if has_or and "oracle" in r:
            orc = r["oracle"]
            print(f"  {r['phase']:8s} {int(la['n']):5d} {la['rmse']:9.2f} "
                  f"{orc['rmse']:8.2f} {r.get('est_share', 0.0)*100:9.0f}% "
                  f"{la['max']:8.1f} {orc['max']:8.1f} "
                  f"{la['on_body']*100:10.0f}% {orc['on_body']*100:10.0f}%")
        else:
            print(f"  {r['phase']:8s} {int(la['n']):5d} {la['rmse']:9.2f} "
                  f"{la['mean']:8.2f} {la['p95']:8.2f} {la['max']:8.1f} "
                  f"{la['on_body']*100:7.0f}%")
    if has_or:
        print("  读法: oracle = 只把速度状态换成真实延迟速度后的残差 (延迟/环路"
              "下界); est_share = 速度估计在误差里的占比 (可改进部分)。")


def suite_headroom(law_name, v_fields, **kw):
    """标准评测组的同一套归因: matched (L=50) 与最坏失配 (L 扫描) 的 composite,
    law 与 oracle 各一份。回答的是"若速度估计换成真实的, 综合分会到哪" ——
    这是"还有多少空间"的总量口径 (逐场景口径见 diag_scenario)。"""
    from arena import runner
    from arena.eval import L_SWEEP
    from arena.scenarios import standard_suite
    suite = standard_suite()
    rep = [s for s in suite if s.name in ("step_80px", "maneuver")]

    def run(scenarios, L_true, oracle):
        out = {}
        for sc in scenarios:
            per = []
            for sd in SEEDS:
                res, _ = _run(law_name, sc, oracle=oracle, v_fields=v_fields,
                              seed=sd, **{**kw, "L_true": L_true})
                per.append(M.compute(res, sc))
            out[sc.name] = {"metrics": per,
                            "agg": runner.aggregate(per, sc.kind)}
        return runner.composite_score(out)

    res = {}
    for tag, oracle in (("law", False), ("oracle", True)):
        matched = run(suite, kw["L_true"], oracle)
        worst = max(run(rep, Lt, oracle) for Lt in L_SWEEP)
        res[tag] = {"matched": matched, "worst_mismatch": worst}
    return res


def print_headroom(law_name, hr):
    print(f"\n=== headroom {law_name} (标准评测组 composite, 越小越好) ===")
    print(f"  {'':8s} {'matched':>9s} {'worst_mismatch':>15s}")
    for tag in ("law", "oracle"):
        r = hr[tag]
        who = "原样" if tag == "law" else "只换真实速度"
        print(f"  {who:8s} {r['matched']:9.2f} {r['worst_mismatch']:15.2f}")
    la, orc = hr["law"], hr["oracle"]
    print(f"  可改进占比: matched {100*(1-orc['matched']/la['matched']):.0f}%  "
          f"worst_mismatch {100*(1-orc['worst_mismatch']/la['worst_mismatch']):.0f}%")


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m arena.diag",
                                 description="单场景误差归因诊断 (纯观察)。")
    ap.add_argument("law", nargs="?", help="law 注册名")
    ap.add_argument("scenarios", nargs="*", help="场景名 (缺省用推荐组)")
    ap.add_argument("--list", action="store_true", help="列出场景名")
    ap.add_argument("--L-true", type=float, default=NOMINAL_L)
    ap.add_argument("--L-belief", type=float, default=NOMINAL_L)
    ap.add_argument("--s-true", type=float, default=1.0)
    ap.add_argument("--s-belief", type=float, default=1.0)
    ap.add_argument("--noise", type=float, default=NOISE)
    ap.add_argument("--drop", type=float, default=0.0)
    ap.add_argument("--max-v", type=float, default=MAX_V)
    ap.add_argument("--v-fields", default="fvx,fvy",
                    help="oracle 覆写的速度字段 (逗号分隔)")
    ap.add_argument("--suite", action="store_true",
                    help="只跑标准评测组的 composite 对照 (比逐场景慢)")
    args = ap.parse_args(argv)
    if args.list:
        print("scenarios:", ", ".join(scenario_names()))
        return
    if not args.law:
        ap.error("需要 law 名 (或 --list)")
    names = args.scenarios or list(DEFAULT_SCENARIOS)
    vf = tuple(x.strip() for x in args.v_fields.split(",") if x.strip())
    kw = dict(L_true=args.L_true, L_belief=args.L_belief, noise=args.noise,
              drop_p=args.drop, s_true=args.s_true, s_belief=args.s_belief,
              max_v=args.max_v)
    if args.suite:
        print_headroom(args.law, suite_headroom(args.law, vf, **kw))
        return
    for nm in names:
        try:
            sc = find_scenario(nm)
        except SystemExit as e:
            print(e)
            continue
        rows = diag_scenario(args.law, sc, v_fields=vf, **kw)
        print_diag(args.law, sc, rows, kw)


if __name__ == "__main__":
    main()
