"""arena/metrics.py — 由真值序列计算指标。

step 场景: 拉枪稳定时间 (快) + 过冲 (小) + 首次到达时间。
track 场景: 稳态 RMSE / 平均|e| / 误差<阈值时间占比。
发散: 单独标记, 综合评分重罚。
阶段指标 (phase_metrics): 场景声明的时间段 (滞空/冲刺/滑铲…) 内的误差统计。

为什么需要阶段指标: 事件指标 over = 事件后峰值 − 事件前 200ms 中位 |e| 是**相对**
口径 —— 若事件前已经长时间拖尾 (典型: 跳跃滞空期), over 会被压小甚至为 0,
"滞空期准星不在人身上"这种**持续跟踪**失效在 over/全场景 rmse 里都看不见
(全场景 rmse 被地面段的正常跟踪稀释)。阶段指标把"某段时间的表现"单独算出来,
与事件指标互补: 事件指标看瞬态冲击, 阶段指标看持续段质量。
"""
from __future__ import annotations
import math

# "打在躯干上"的判据: 屏幕上的躯干尺寸随距离缩放, 所以命中带只能是距离的函数,
# 不能写死绝对值 —— 把 10m 的值固定下来, 在 3m 处会把 80px 的半宽当成 24px,
# 系统性少算命中率。换算基准与 fps.py 同一套投影 (fps.py docstring): 1080p/90°
# hFOV 的焦距 ≈ 960px; 躯干半宽 0.25m → 距离 d 处屏幕半宽 = 960·0.25/d px。
F_PX = 960.0          # 投影焦距 px (1080p / 90° hFOV)
BODY_HALF_M = 0.25    # 躯干半宽 m


def body_px(dist_m: float) -> float:
    """距离 dist_m 处的躯干屏幕半宽 px (命中带半径)。"""
    return F_PX * BODY_HALF_M / max(1e-6, dist_m)


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
    # 方向无关回弹: 首次到达目标后, 任意方向的最大 |e| 偏离
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


def event_metrics(res, events, pre_ms=200.0, window_ms=600.0, band=3.0):
    """事件后过冲/恢复 (急停/落地/折返/变向等, 事件由场景声明)。

    每个事件 (t, kind) 报告:
      pre   事件前 pre_ms 窗口 |e| 中位数 (基线拖尾)
      peak  事件后 window_ms 内最大 |e|
      over  peak − pre (事件注入的额外误差激励, ≥0) — 核心指标
      rec   事件后首次 |e| ≤ max(band, pre) 的时刻偏移 (ms, inf=未恢复)
    对所有 law 完全中立; 发散场景全部 inf。
    """
    if res["diverged"]:
        return [{"t": te, "kind": kd, "pre": float("inf"),
                 "peak": float("inf"), "over": float("inf"),
                 "rec": float("inf")} for te, kd in events]
    t, ex, ey = res["t"], res["ex"], res["ey"]
    e = [math.hypot(a, b) for a, b in zip(ex, ey)]
    out = []
    for te, kd in events:
        pre = [v for tt, v in zip(t, e) if te - pre_ms <= tt < te]
        pre_v = sorted(pre)[len(pre) // 2] if pre else 0.0
        seg = [(tt, v) for tt, v in zip(t, e) if te <= tt <= te + window_ms]
        peak_v = max(v for _, v in seg) if seg else float("inf")
        thr = max(band, pre_v)
        rec = float("inf")
        for tt, v in seg:
            if v <= thr:
                rec = tt - te
                break
        out.append({"t": te, "kind": kd, "pre": pre_v, "peak": peak_v,
                    "over": max(0.0, peak_v - pre_v), "rec": rec})
    return out


def phase_metrics(res, phases, band=3.0, dist_m=10.0):
    """场景声明的时间段内的误差统计 (与 event_metrics 互补)。

    每个阶段 (t0, t1, kind) 报告该段内的 |e| 分布:
      n        该段拍数 (0 = 段落在运行时长之外, 其余字段为 0)
      rmse/mean/p95/max   段内误差统计 (p95 = 分位数, 抗单帧毛刺)
      in_band  段内 |e| <= band 的时间占比 (与 settle_band 同口径)
      on_body  段内 |e| <= body_px(dist_m) 的时间占比 ("准星还在躯干上")
    发散场景全部 inf (口径与 event_metrics 一致, 便于混合排序)。
    """
    hit = body_px(dist_m)
    if res["diverged"]:
        return [{"t0": t0, "t1": t1, "kind": kd, "n": 0, "rmse": float("inf"),
                 "mean": float("inf"), "p95": float("inf"),
                 "max": float("inf"), "in_band": 0.0, "on_body": 0.0}
                for t0, t1, kd in phases]
    t, ex, ey = res["t"], res["ex"], res["ey"]
    e = _abs_err(ex, ey)
    out = []
    for t0, t1, kd in phases:
        seg = [v for tt, v in zip(t, e) if t0 <= tt <= t1]
        if not seg:
            out.append({"t0": t0, "t1": t1, "kind": kd, "n": 0, "rmse": 0.0,
                        "mean": 0.0, "p95": 0.0, "max": 0.0,
                        "in_band": 0.0, "on_body": 0.0})
            continue
        n = len(seg)
        ss = sorted(seg)
        out.append({
            "t0": t0, "t1": t1, "kind": kd, "n": n,
            "rmse": math.sqrt(sum(v * v for v in seg) / n),
            "mean": sum(seg) / n,
            "p95": ss[min(n - 1, int(0.95 * n))],
            "max": ss[-1],
            "in_band": sum(1 for v in seg if v <= band) / n,
            "on_body": sum(1 for v in seg if v <= hit) / n,
        })
    return out


def pooled_phase(phases_out, kind=None):
    """把多个同 kind 阶段汇总成一个 (取各段的平均; n=0 的段跳过)。
    段数不等时按段平均, 口径是"每段平均表现", 不是"时间加权"。"""
    sel = [p for p in phases_out if (kind is None or p["kind"] == kind)
           and p["n"] > 0]
    if not sel:
        return None
    keys = ("rmse", "mean", "p95", "max", "in_band", "on_body")
    agg = {k: sum(p[k] for p in sel) / len(sel) for k in keys}
    agg["n_seg"] = len(sel)
    return agg
