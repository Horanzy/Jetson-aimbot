"""arena/trace.py — 单场景逐拍过程追踪 (纯观察)。

动机: eval/integrate/fps_eval 只给聚合终值, law 坏在哪里 (振荡 / 尾迹慢 /
v̂ 重建卡住 / 噪声直通 / 积分饱和) 看不见。本模块把一次运行逐拍落盘 +
给终端过程摘要, 让"坏在哪"一眼可见。

中立性 (不可违反): 本模块只读地记录 被控对象/传感器 与 law 的交互, 不向回路
回馈任何信息, 不改变任何评测数字。实现方式是 law 代理 (转发 reset/step,
逐拍记录), core/runner/eval 一行未动; 默认三套评测的输出与本模块存在与否
无关。law 不实现 debug() 钩子时, trace 只是少了 dbg_* 列, 一切照旧。

law 可选调试协议:
    def debug(self) -> dict[str, float]:
        返回内部状态标量 (如 Kp/Ki/v̂/S_cusum/σ̂ …), trace 逐拍采集为
        dbg_<key> 列。字段名与语义由 law 自己的 docstring 定义, arena 不
        解释。要求无副作用 (只在 trace 路径被调用, 不进入任何评测路径)。

用法:
  python -m arena.trace <law> [场景] [选项]
  python -m arena.trace --list

常用:
  python -m arena.trace ff_pi_acc step_80px --csv out.csv
  python -m arena.trace ff_pi_acc maneuver --L-true 30 --auto-window
  python -m arena.trace ff_pi_acc fps_stop_hard --event 0 --csv stop.csv

输出: 终端摘要 (十几行) + 可选 CSV (逐拍全量) + 可选 PNG (装了 matplotlib
才有, 没有则跳过, 不为此新增依赖)。默认输出目录 arena/trace_out/ (gitignore)。
"""
from __future__ import annotations
import argparse
import csv
import math
import os
import random
import sys
from arena.core import Arena, ArenaConfig
from arena.scenarios import standard_suite, Scenario
from arena.fps import fps_suite
from arena import metrics as M
from arena import runner
from arena.laws.base import get_law, all_laws

TRACE_OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trace_out")

# 与 eval/fps_eval 一致的标准传感器工况 (默认值)
NOISE = 0.5
NOMINAL_L = 50.0
MAX_V = 1.5
# 终端摘要参数 (纯展示用启发值, 不进任何指标)
LADDER_BANDS = (10.0, 5.0, 3.0, 1.0)   # 进入带宽阶梯 px
TAIL_MS = 500.0                        # 尾段振荡检测窗口
EVENT_SPAN_MS = 1000.0                 # --event 窗口长度
AUTO_WINDOW_MS = 1000.0                # 自动截窗长度


def find_scenario(name: str) -> Scenario:
    """按名字找场景: 标准套件 + FPS 套件 + relock (integrate 用)。"""
    from arena.integrate import relock_scenario
    for sc in list(standard_suite()) + list(fps_suite()) + [relock_scenario()]:
        if sc.name == name:
            return sc
    raise SystemExit(f"未知场景: {name}\n可用: " + ", ".join(scenario_names()))


def scenario_names():
    from arena.integrate import relock_scenario
    return ([sc.name for sc in standard_suite()]
            + [sc.name for sc in fps_suite()] + [relock_scenario().name])


class _TracingLaw:
    """law 代理: 逐拍转发并记录 (纯观察, 行为与裸 law 逐位一致)。"""

    def __init__(self, inner):
        self.inner = inner
        self.rows = []          # 每拍: (t, obs_t, obs_dx, obs_dy, obs_new, dbg: dict)
        self.dbg_keys: list[str] = []
        self._dbg_seen: set = set()
        self.has_debug = callable(getattr(inner, "debug", None))

    def reset(self, cfg):
        self.inner.reset(cfg)

    def step(self, t, obs):
        cx, cy = self.inner.step(t, obs)
        dbg = {}
        if self.has_debug:
            try:
                raw = self.inner.debug()
            except Exception:
                raw = None
            if raw:
                for k, v in raw.items():
                    if k not in self._dbg_seen:
                        self._dbg_seen.add(k)
                        self.dbg_keys.append(k)
                    try:
                        dbg[k] = float(v)
                    except (TypeError, ValueError):
                        dbg[k] = None
        if obs is not None:
            self.rows.append((t, obs.t, obs.dx, obs.dy, 1 if obs.new else 0, dbg))
        else:
            self.rows.append((t, None, None, None, 0, dbg))
        return cx, cy


class TraceResult:
    def __init__(self, law_name, sc, cfg_kwargs, rows, res, events):
        self.law_name = law_name
        self.scenario = sc
        self.cfg_kwargs = cfg_kwargs      # 记录本次工况
        self.rows = rows                  # 见 _TracingLaw.rows + 落盘时并真值
        self.res = res                    # Arena.result(): t/ex/ey/sent_cx/sent_cy
        self.events = events              # [(t_ms, kind), ...]
        self.diverged = res["diverged"]


def run_trace(law_name, scenario_name, *, L_belief=NOMINAL_L, L_true=NOMINAL_L,
              s_belief=1.0, s_true=1.0, fps=120, noise=NOISE, drop_p=0.0,
              seed=1, max_v=MAX_V, duration=None) -> TraceResult:
    """跑一次 单 law × 单场景, 返回逐拍记录 (不打印)。"""
    law_cls = get_law(law_name)
    sc = find_scenario(scenario_name)
    rng = random.Random(seed)
    tgt = sc.make_target(rng)
    cfg = ArenaConfig(s_true=s_true, L_true=L_true, fps=fps, noise_std=noise,
                      duration=sc.duration if duration is None else duration,
                      drop_p=drop_p)
    ar = Arena(cfg, tgt, rng, cross0=sc.cross0)
    tl = _TracingLaw(law_cls())
    res = ar.run(tl, s_belief, L_belief, max_v)
    events = list(getattr(sc, "events", ()) or ())
    if sc.kind == "step" and (not events or events[0][0] > 0.0):
        events = [(0.0, "step")] + events     # step 场景: t=0 即阶跃事件
    kw = dict(L_belief=L_belief, L_true=L_true, s_belief=s_belief, s_true=s_true,
              fps=fps, noise=noise, drop_p=drop_p, seed=seed, max_v=max_v,
              duration=duration)
    return TraceResult(law_name, sc, kw, tl.rows, res, events)


def _abs_e(res):
    return [math.hypot(a, b) for a, b in zip(res["ex"], res["ey"])]


def _first_reach(t, e, t0, band, t_end):
    """窗口 (t0, t_end) 内首次 |e|<=band 的偏移 ms; 没进过则 None。"""
    for tt, v in zip(t, e):
        if tt < t0:
            continue
        if tt > t_end:
            break
        if v <= band:
            return tt - t0
    return None


def tail_oscillation(t, ex, ey, tail_ms=TAIL_MS):
    """尾段振荡检测: 最后 tail_ms 内 ex/ey 的过零次数与幅度 (展示用启发)。"""
    t0 = t[-1] - tail_ms
    xs = [(tt, a, b) for tt, a, b in zip(t, ex, ey) if tt >= t0]
    if len(xs) < 3:
        return None
    def crossings(seq):
        last = 0.0
        n = 0
        for v in seq:
            if v != 0.0:
                if last != 0.0 and (v > 0) != (last > 0):
                    n += 1
                last = v
        return n
    cx_n = crossings([a for _, a, _ in xs])
    cy_n = crossings([b for _, _, b in xs])
    amp = max(max(abs(a) for _, a, _ in xs), max(abs(b) for _, _, b in xs))
    if cx_n + cy_n >= 6 and amp > 2.0:
        verdict = "明显振荡"
    elif cx_n + cy_n >= 3 and amp > 0.5:
        verdict = "轻度振荡"
    else:
        verdict = "收敛"
    return {"crossings": (cx_n, cy_n), "amp": amp, "verdict": verdict}


def summarize(tr: TraceResult, tail_ms=TAIL_MS):
    """终端过程摘要 (紧凑, 目标只有一个: 坏在哪一眼可见)。"""
    res, sc, t = tr.res, tr.scenario, tr.res["t"]
    ex, ey = res["ex"], res["ey"]
    e = _abs_e(res)
    kw = tr.cfg_kwargs
    out = []
    out.append(f"=== trace {tr.law_name} @ {sc.name} ===")
    out.append(f"  工况: L_true={kw['L_true']:.0f} L_belief={kw['L_belief']:.0f} "
               f"s_true={kw['s_true']:.2f} s_belief={kw['s_belief']:.2f} "
               f"fps={kw['fps']} noise={kw['noise']} drop={kw['drop_p']} "
               f"seed={kw['seed']} 时长={t[-1]:.0f}ms"
               + (f"  [截断于 {kw['duration']:.0f}ms]" if kw['duration'] else ""))
    if tr.diverged:
        out.append("  *** DIVERGED (发散, 以下数字仅示意) ***")
    # 全局
    pk = max(range(len(e)), key=lambda i: e[i])
    n_new = sum(1 for r in tr.rows if r[4])
    out.append(f"  |e|: 峰值 {e[pk]:.1f}px @ t={t[pk]:.0f}ms  "
               f"末值 {e[-1]:.2f}px  新帧 {n_new}/{len(tr.rows)}拍")
    # 场景事件窗口
    if tr.events:
        out.append("  事件窗口 (event_metrics 口径: pre=前200ms中位 |e|, "
                   "peak/rec=后1s):")
        evm = M.event_metrics(res, tr.events)
        for j, (te, kd) in enumerate(tr.events):
            m = evm[j]
            ladder = " ".join(
                f"{b:g}px:{'never' if (d := _first_reach(t, e, te, b, te + EVENT_SPAN_MS)) is None else f'{d:.0f}ms'}"
                for b in LADDER_BANDS)
            rec = m["rec"]
            rec_s = "inf" if rec == float("inf") else f"{rec:.0f}ms"
            out.append(f"    #{j} t={te:.0f}ms {kd}: pre={m['pre']:.1f} "
                       f"peak={m['peak']:.1f} over={m['over']:.1f} rec={rec_s}"
                       + f"  | 进带阶梯 {ladder}")
    elif sc.kind == "step":
        ladder = " ".join(
            f"{b:g}px:{'never' if (d := _first_reach(t, e, 0.0, b, t[-1])) is None else f'{d:.0f}ms'}"
            for b in LADDER_BANDS)
        out.append(f"  进带阶梯 (自 t=0): {ladder}")
    else:
        sf = sc.steady_from
        seg = [v for tt, v in zip(t, e) if tt >= sf]
        if seg:
            out.append(f"  稳态 (t>={sf:.0f}ms): rmse="
                       f"{math.sqrt(sum(v*v for v in seg)/len(seg)):.2f}px "
                       f"mean={sum(seg)/len(seg):.2f}px "
                       f"in_band(3px)={sum(1 for v in seg if v <= 3.0)/len(seg)*100:.1f}% "
                       f"max={max(seg):.1f}px")
    # 最差窗口 (track 场景: |e| 峰值所在 1s 窗)
    w0 = max(0.0, t[pk] - AUTO_WINDOW_MS / 2)
    w1 = min(t[-1], t[pk] + AUTO_WINDOW_MS / 2)
    out.append(f"  最差 1s 窗: [{w0:.0f}, {w1:.0f}]ms 峰值 {e[pk]:.1f}px "
               f"(--auto-window 可导出该窗)")
    # 尾段振荡
    to = tail_oscillation(t, ex, ey, tail_ms)
    if to:
        out.append(f"  尾段({tail_ms:.0f}ms): ex过零{to['crossings'][0]}次 "
                   f"ey过零{to['crossings'][1]}次 幅度{to['amp']:.2f}px → {to['verdict']}")
    return "\n".join(out)


def write_csv(tr: TraceResult, path, t0=None, t1=None):
    """逐拍 CSV。t0/t1 给定时只写窗口内行。列:
    t, ex, ey, abs_e, sent_cx, sent_cy, obs_t, obs_dx, obs_dy, obs_new[, dbg_*]"""
    res, rows = tr.res, tr.rows
    dbg_keys = []
    for r in rows:
        for k in r[5]:
            if k not in dbg_keys:
                dbg_keys.append(k)
    header = (["t", "ex", "ey", "abs_e", "sent_cx", "sent_cy",
               "obs_t", "obs_dx", "obs_dy", "obs_new"] + [f"dbg_{k}" for k in dbg_keys])
    n = min(len(rows), len(res["t"]))
    d = os.path.dirname(os.path.abspath(path))
    os.makedirs(d, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        for i in range(n):
            tt = res["t"][i]
            if t0 is not None and tt < t0:
                continue
            if t1 is not None and tt > t1:
                continue
            row_t, ot, odx, ody, onew, dbg = rows[i]
            e = math.hypot(res["ex"][i], res["ey"][i])
            w.writerow([f"{tt:.1f}", f"{res['ex'][i]:.3f}", f"{res['ey'][i]:.3f}",
                        f"{e:.3f}", res["sent_cx"][i], res["sent_cy"][i],
                        "" if ot is None else f"{ot:.1f}",
                        "" if odx is None else f"{odx:.3f}",
                        "" if ody is None else f"{ody:.3f}", onew]
                       + ["" if dbg.get(k) is None else f"{dbg[k]:.4g}" for k in dbg_keys])
    return n


def write_png(tr: TraceResult, path):
    """可选 PNG: matplotlib 存在才画, 否则跳过 (不新增依赖)。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("  (无 matplotlib, 跳过 PNG; CSV+终端摘要已足够)")
        return False
    res, t = tr.res, tr.res["t"]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    ax1.plot(t, res["ex"], label="ex", lw=0.8)
    ax1.plot(t, res["ey"], label="ey", lw=0.8)
    ax1.plot(t, _abs_e(res), label="|e|", lw=0.8, color="k", alpha=0.6)
    for te, kd in tr.events:
        ax1.axvline(te, color="r", ls=":", lw=0.8)
        ax1.annotate(kd, (te, ax1.get_ylim()[1]), fontsize=7, color="r")
    ax1.set_ylabel("px")
    ax1.legend(fontsize=8)
    ax1.set_title(f"{tr.law_name} @ {tr.scenario.name}")
    ax2.plot(t, res["sent_cx"], label="sent_cx", lw=0.8)
    ax2.plot(t, res["sent_cy"], label="sent_cy", lw=0.8)
    ax2.set_ylabel("counts/tick")
    ax2.set_xlabel("t (ms)")
    ax2.legend(fontsize=8)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return True


def _parse_window(s):
    a, b = s.split(":")
    return float(a), float(b)


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="python -m arena.trace",
        description="单 law × 单场景 逐拍过程追踪 (纯观察)。")
    ap.add_argument("law", nargs="?", help="law 注册名 (wip_* 也可以)")
    ap.add_argument("scenario", nargs="?", default="step_80px", help="场景名")
    ap.add_argument("--list", action="store_true", help="列出可用 law 与场景")
    ap.add_argument("--L-true", type=float, default=NOMINAL_L)
    ap.add_argument("--L-belief", type=float, default=NOMINAL_L)
    ap.add_argument("--s-true", type=float, default=1.0)
    ap.add_argument("--s-belief", type=float, default=1.0)
    ap.add_argument("--fps", type=int, default=120)
    ap.add_argument("--noise", type=float, default=NOISE)
    ap.add_argument("--drop", type=float, default=0.0, help="每帧丢失概率")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--max-v", type=float, default=MAX_V)
    ap.add_argument("--duration", type=float, default=None, help="覆盖场景时长 ms")
    ap.add_argument("--csv", default=None, help="逐拍 CSV 输出路径")
    ap.add_argument("--png", default=None, help="PNG 输出路径 (需 matplotlib)")
    ap.add_argument("--window", default=None, help="只导出时间窗 a:b (ms)")
    ap.add_argument("--event", type=int, default=None, help="只导出第 N 个事件窗")
    ap.add_argument("--auto-window", action="store_true",
                    help="只导出 |e| 峰值所在 1s 窗")
    ap.add_argument("--tail-ms", type=float, default=TAIL_MS)
    ap.add_argument("--outdir", default=TRACE_OUT)
    args = ap.parse_args(argv)

    if args.list:
        print("laws:", ", ".join(sorted(all_laws())))
        print("scenarios:", ", ".join(scenario_names()))
        return
    if not args.law:
        ap.error("需要 law 名 (或 --list)")

    tr = run_trace(args.law, args.scenario, L_belief=args.L_belief,
                   L_true=args.L_true, s_belief=args.s_belief,
                   s_true=args.s_true, fps=args.fps, noise=args.noise,
                   drop_p=args.drop, seed=args.seed, max_v=args.max_v,
                   duration=args.duration)
    print(summarize(tr, tail_ms=args.tail_ms))

    t0 = t1 = None
    if args.window:
        t0, t1 = _parse_window(args.window)
    elif args.event is not None:
        if not tr.events:
            ap.error("该场景无事件标注")
        te = tr.events[args.event][0]
        t0, t1 = te, te + EVENT_SPAN_MS
    elif args.auto_window:
        e = _abs_e(tr.res)
        pk = max(range(len(e)), key=lambda i: e[i])
        t0 = max(0.0, tr.res["t"][pk] - AUTO_WINDOW_MS / 2)
        t1 = tr.res["t"][pk] + AUTO_WINDOW_MS / 2

    if args.csv:
        path = args.csv if os.path.isabs(args.csv) else os.path.join(args.outdir, args.csv)
        n = write_csv(tr, path, t0, t1)
        scope = "全量" if t0 is None else f"窗口 [{t0:.0f},{t1:.0f}]ms"
        print(f"  CSV: {path} ({scope}, {n} 行)")
    if args.png:
        path = args.png if os.path.isabs(args.png) else os.path.join(args.outdir, args.png)
        if write_png(tr, path):
            print(f"  PNG: {path}")


if __name__ == "__main__":
    main()
