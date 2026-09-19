"""arena/metrics.py — 由真值序列计算指标。

step 场景: 拉枪稳定时间 (快) + 过冲 (小) + 首次到达时间。
track 场景: 稳态 RMSE / 平均|e| / 误差<阈值时间占比。
发散: 单独标记, 综合评分重罚。
阶段指标 (phase_metrics): 场景声明的时间段 (滞空/冲刺/滑铲…) 内的误差统计。
后坐力指标 (recoil_metrics): 周期性外部扰动工况下的指令平滑度 + 精度代价。

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


# ---- 后坐力工况 (周期性外部扰动) 的平滑度/精度指标 ----
#
# 为什么需要单独一组: 标准的 rmse/过冲口径只衡量"准星离目标多远", 对"同一段
# 补偿位移是怎么发出去的"完全不敏感 —— 把整发落差挤在间隔中间的一小段里压完、
# 其余时间停住等下一发, 与人类把同样的位移匀速摊到整个间隔, 二者的 rmse 可以一致,
# 但画面一个"一顿一顿"一个平滑。下面四个量把这件事量化:
#
#   1. 响应形态 fill / iqr: 以每发时刻对齐, 把该发间隔内交付的位移按时间展开成
#      一个分布 —— 每拍的位移 |counts|·s 作为权重, 取它 25/75/90% 的时间分位点。
#      分位点是常规统计量 (与 phase_metrics 的 p95 同族), 无阈值: 匀速压枪把同样
#      的位移摊满整个间隔 → 90% 分位点在 0.90 处、25↔75 跨度 0.50; 矩形脉冲把
#      位移挤在中间一小段 → 两个数都塌下来。fill90 = "响应基本完成"的时刻占间隔
#      的比例 (中性 0.90), iqr = 位移在时间上的集中度 (中性 0.50) —— 这两个数
#      就是"一顿一顿"的直接量化: 同样的 rmse, 分布可以完全不同。
#   2. 指令台阶 steps_per_s: 指令速度在一帧内的变化量超过 θ 的次数/秒。
#      θ = q = s_true/(w·h) = **帧分辨率的表达下限**: 整数 counts 在传感器一帧
#      里能表达的最小非零速度变化 (1 count/帧, w = 一帧的拍数)。这是一个表达性
#      事实而非选择: 比 q 更小的变化在整数指令上不存在, 所以它既是阈值也是量化的
#      本底 —— θ 不含自由倍数是刻意的 (1.5×θ 会把计数砍半, 那种敏感的数字不是
#      可观测量)。计数因此读作"每秒有多少次, 交付给游戏的帧速率变化超过一像素
#      每帧", 其中包含量化器自身的抖动 —— 抖动也是真实交付给游戏的运动。
#   3. 反转 rev_per_s: 实际交付位移 (counts 的精确积分, 无量化误差) 上方向翻转的
#      次数/秒, 翻转腿须超过 PX_QUANTUM (屏幕像素 = 显示器上最小可见位移)。衡量
#      "压过头 → 再拉回来"的往复。
#   4. 高频含量 acc_rms: 指令速度的帧间变化率 RMS (px/ms²)。指令速度是瞄准位置
#      的导数, 它的变化率就是瞄准加速度; 矩形脉冲正是一个加速度冲激, 其 RMS 即
#      高频含量。再差分一阶 (jerk) 只是同一量化台阶上的二阶差分: 实测它的 RMS
#      只比"量化本底 acc_rms/frame_dt"高 1.1-1.7 倍 (rc_300rpm_20px /
#      rc_900rpm_60px, 两种 law 一致), 即信号没有随再次差分增长, 只是噪声被
#      放大 —— 故不单列, 需要时可从 arena.trace 的逐拍 counts 自行求二阶差分。
#
# 精度侧: 竖直 rmse/偏置/p95, 每发过冲 over_y (间隔内 ey 的正峰值 = 压过头多少),
# on_body (躯干半宽随距离缩放, 与 phase_metrics 同口径), 以及停火后回到
# settle_band 的恢复时间 (与 step_metrics 同口径)。
#
# 指令速度的重建: 只有被控对象实际收到的整数 counts 是诚实的信号, 而单拍的整数
# counts 携带量化抖动 (1 count = s px), 因此按**传感器自己的帧周期**取块平均
# (frame_dt 取整到拍网格 = w 拍一块, 块内拍数恒定 → 重建信号的量化单位恒定) ——
# 这是回路里本来就存在的时间分辨力, 不是新加的滤波。
# 位移量 (形态分位/反转/过冲) 直接在 counts 的累积和上算, 量化误差为零。
PX_QUANTUM = 1.0        # 屏幕像素: 显示器上最小可见位移 px (翻转腿/补偿量下限)

_RECOIL_KEYS = ("n_shots", "n_frac", "rmse_y", "mean_y", "mean_abs_y",
                "p95_abs_y", "max_abs_y", "over_y", "on_body", "rec_ms",
                "steps_per_s", "steps_per_shot", "rev_per_s", "acc_rms",
                "fill90", "iqr", "thr_step")


def _median(xs):
    xs = sorted(xs)
    n = len(xs)
    if n == 0:
        return 0.0
    m = n // 2
    return xs[m] if n % 2 else 0.5 * (xs[m - 1] + xs[m])


def _zigzag_reversals(seq, quantum):
    """累计位移序列里幅值 ≥ quantum 的方向翻转次数 (更小的往复不计数)。"""
    if len(seq) < 2:
        return 0
    n = 0
    ext = seq[0]
    direction = 0                       # +1 上行 / -1 下行 / 0 未定
    for v in seq[1:]:
        if direction == 0:
            if v > ext:
                direction, ext = 1, v
            elif v < ext:
                direction, ext = -1, v
        elif direction > 0:
            if v >= ext:
                ext = v
            elif ext - v >= quantum:
                n += 1
                direction, ext = -1, v
        else:
            if v <= ext:
                ext = v
            elif v - ext >= quantum:
                n += 1
                direction, ext = 1, v
    return n


def _frame_blocks(frame_dt, h):
    """传感器一帧 ≈ 多少拍 (整数, 取整到拍网格; >=1)。"""
    return max(1, int(round(frame_dt / h)))


def _frame_velocity(cy, s_true, h, w, i0, i1):
    """窗口内按传感器帧取块平均的指令速度 (px/ms): 每 w 拍一块 (块内拍数恒定,
    于是重建信号的量化单位恒定 = s/(w·h)), 末尾不足一块的残块丢弃。"""
    out = []
    for a in range(i0, i1 - w + 1, w):
        out.append(s_true * sum(cy[a:a + w]) / (w * h))
    return out


def _motion_quantiles(prof, qs=(0.25, 0.75, 0.90)):
    """交付位移的时间分位点 (fraction of interval): prof = [(offset_ms, cum_px)]
    单调递增 (每拍的 |位移| 累加), 返回各分位点首次达到的时刻占比。"""
    tot = prof[-1][1]
    out = []
    j = 0
    for q in qs:
        lvl = q * tot
        while j < len(prof) - 1 and prof[j][1] < lvl:
            j += 1
        out.append(prof[j][0])
    return out


def recoil_metrics(res, scenario):
    """周期性外部扰动 (后坐力) 工况的平滑度 + 精度指标 (见上方注释)。

    场景须提供 shots (每发时刻 ms)、interval_ms (标称发间隔)、dist_m (命中带
    缩放) —— 由 arena/recoil.py 的 RecoilScenario 给出。统计窗口 = [首发,
    末发 + 一个发间隔]; 停火恢复时间 rec_ms 看到运行结束。
    发散场景全部 inf (口径与 event_metrics/phase_metrics 一致)。
    """

    def _bad():
        d = {k: float("inf") for k in _RECOIL_KEYS}
        d["n_shots"] = len(shots)
        d["n_frac"] = 0
        d["on_body"] = 0.0
        return d

    shots = tuple(getattr(scenario, "shots", ()) or ())
    if res["diverged"] or not shots:
        return _bad()

    t, ex, ey = res["t"], res["ex"], res["ey"]
    cy = res["sent_cy"]
    h = res.get("h", 2.0)
    frame_dt = res.get("frame_dt", 1000.0 / 120.0)
    s_true = res.get("s_true", 1.0)
    interval = float(scenario.interval_ms)
    t0, t1 = shots[0], shots[-1] + interval
    idx = [i for i, tt in enumerate(t) if t0 <= tt <= t1]
    if len(idx) < 2:
        return _bad()
    i0, i1 = idx[0], idx[-1] + 1
    win_s = (t[i1 - 1] - t[i0]) / 1000.0

    # --- 1. 每发响应形态 (交付位移的时间分位点) + 每发过冲
    fills, iqrs, overs = [], [], []
    for ts in shots:
        prof = []
        w = 0.0
        eymax = None
        for i in range(i0, i1):
            if ts <= t[i] < ts + interval:
                w += abs(cy[i]) * s_true
                prof.append((t[i] - ts, w))
                eymax = ey[i] if eymax is None else max(eymax, ey[i])
        if eymax is None:
            continue
        overs.append(max(0.0, eymax))
        if prof[-1][1] < PX_QUANTUM:
            continue
        q25, q75, q90 = _motion_quantiles(prof)
        iqrs.append((q75 - q25) / interval)
        fills.append(q90 / interval)
    fill90 = _median(fills) if fills else float("inf")
    iqr = _median(iqrs) if iqrs else float("inf")
    over_y = sum(overs) / len(overs) if overs else 0.0

    # --- 2. 指令台阶 (帧率上的指令速度序列; θ = max(帧分辨率下限, 信号稳健尺度))
    w = _frame_blocks(frame_dt, h)
    V = _frame_velocity(cy, s_true, h, w, i0, i1)
    dV = [b - a for a, b in zip(V, V[1:])]
    thr = s_true / (w * h)              # 重建信号的最小非零变化 = 1 count/帧
    steps = sum(1 for d in dV if abs(d) > thr)
    n_shots_win = sum(1 for ts in shots if t0 <= ts <= t[i1 - 1])
    steps_per_s = steps / win_s

    # --- 3. 方向反转 (实际交付位移: counts 的精确积分)
    cum = []
    acc = 0.0
    for i in range(i0, i1):
        acc += cy[i] * s_true
        cum.append(acc)
    rev_per_s = _zigzag_reversals(cum, PX_QUANTUM) / win_s

    # --- 4. 高频含量 (帧率指令速度的变化率 = 瞄准加速度)
    acc_rms = math.sqrt(sum(d * d for d in dV) / len(dV)) / frame_dt if dV else 0.0

    # --- 精度代价 (射击段窗口) + 停火后恢复
    eys = [ey[i] for i in range(i0, i1)]
    n = len(eys)
    rmse_y = math.sqrt(sum(v * v for v in eys) / n)
    abs_y = sorted(abs(v) for v in eys)
    hit = body_px(scenario.dist_m)
    on_body = sum(1 for i in range(i0, i1)
                  if math.hypot(ex[i], ey[i]) <= hit) / n
    band = scenario.settle_band
    last_out = shots[-1]
    for i in range(i1, len(t)):                      # 停火段 (窗口之后)
        if abs(ey[i]) > band:
            last_out = t[i]
    rec_ms = float("inf") if abs(ey[-1]) > band else max(0.0, last_out - shots[-1])

    return {
        "n_shots": n_shots_win, "n_frac": len(fills),
        "rmse_y": rmse_y, "mean_y": sum(eys) / n,
        "mean_abs_y": sum(abs_y) / n,
        "p95_abs_y": abs_y[min(n - 1, int(0.95 * n))], "max_abs_y": abs_y[-1],
        "over_y": over_y, "on_body": on_body, "rec_ms": rec_ms,
        "steps_per_s": steps_per_s,
        "steps_per_shot": steps / max(1, n_shots_win),
        "rev_per_s": rev_per_s, "acc_rms": acc_rms,
        "fill90": fill90, "iqr": iqr, "thr_step": thr,
    }


def pooled_recoil(rows, keys=("fill90", "iqr", "steps_per_shot", "rev_per_s",
                              "acc_rms", "on_body", "rmse_y", "over_y",
                              "rec_ms")):
    """把多个组合的 recoil_metrics 取平均 (逐组合等权)。发散组合计 inf。"""
    if not rows:
        return None
    return {k: sum(r[k] for r in rows) / len(rows) for k in keys}


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
