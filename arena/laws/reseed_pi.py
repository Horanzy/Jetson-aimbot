"""arena/laws/reseed_pi.py — 极点配置 PI + type-2 前馈 + 方向矛盾 CUSUM
告警重播种 (模型破缺后 v̂ 从阶跃读数直达, 不从零重建), 种子带**精确窗差
证据门** — 只有读数自身可信时才播种, 否则按归零重拉回退。

结构 (控制回路与估计器与 ff_pi_acc 逐字节相同, 只改 CUSUM 告警动作):
    ê = f + v̂·(age+L̂·L_COMP) − s·Σcounts(in flight)        // Smith 预测误差
    wn = (90°−PM)·π/180 / L̂,  Kp = 2ζ·wn,  Ki = wn²          // 极点配置 (ζ=1)
    gate = I_GATE/(I_GATE+|ê|)                               // I 项距离门控
    v = Kp·ê + Ki·∫ê·gate + FF_GAIN_VAL·ff_gate·gap·v̂        // PI + type-2 前馈
    方向矛盾 CUSUM (σ 归一) 告警 → **重播种**: v̂ 不经从零重建, 直接取矛盾 run 读数。

原理: 矛盾 run 期间逐帧创新 νᵢ ≈ (v_真 − v̂ᵢ)·Tᵢ + 噪声, 本身就是速度
阶跃的直接读数:
    v̂_new = mean(νᵢ/Tᵢ + v̂ᵢ)        (run 内逐帧, ≤5 帧)
种子噪声 σ_seed = σ̂/(T̄·√n) (每帧 νᵢ/Tᵢ 的噪声 σ̂/Tᵢ 独立平均), 对 0.5px/ms
量级的速度阶跃一步到位到百分之几; 残差由原 β0 慢速精化 — 重建尾从"从零爬
τ = dt/β0 ≈ 280ms"变为"从读数直达"。

证据门 (种子纯度防线, 精确窗差上界):
    读数并不干净。readingᵢ := νᵢ/Tᵢ + v̂ᵢ 有精确分解
        readingᵢ = v_真 + ŝ·[q(aᵢ) − q(fᵢ)] + 噪声/Tᵢ + 滤波误差
    其中 q(w) = 窗口 w 的自身流量均值 (px/ms, C = counts 历史精确可得),
    aᵢ = detᵢ − L̂·L_COMP (信念锚点), fᵢ = detᵢ − L_真 (真窗口)。
    第二项就是"种子继承 Smith 窗错位垃圾"的精确形态: ∝ 自身运动在信念窗
    与真窗之间的流量差。L_真 不可观测, 但标定先验带给出 f−a 的滑动范围
        f − a = Lc − L_真 ∈ [(lc−1)L̂ − δ, (lc−1)L̂ + δ],  δ = BAND_FRAC·L̂
    (lc = L_COMP)。垃圾上界在该带内**精确可计算** (counts 历史是法则自己
    发出的指令记录):
        J = maxᵢ (ŝ/Tᵢ)·max_{f'} |C(aᵢ)−C(aᵢ−Tᵢ) − C(f'ᵢ)−C(f'ᵢ−Tᵢ)|
    run 内取最坏读数的界 (免单读数估计侥幸)。决策 (标准保守估计: 从种子
    变化量中减去偏差上界, 保留方向):
        dv = sign(dv0)·max(0, |dv0| − J),  dv0 = raw − v̂_pre
        |dv| < 3σ_seed → 噪声告警/纯垃圾矛盾, 按归零重拉原样处理 (3σ = 标准
        显著性常数, 同基线); 否则播种 v̂ := v̂_pre + dv。
    平稳自身运动 (纯 FF 跟踪 — 恰是矛盾 run 的典型形态, 伪创新 ∝ a_own·Δc
    最小) 时窗差 J≈0, 门开; 自身剧烈加减速 (伪创新最大) 时 J 大, 门关 —
    门与伪创新物理精确对齐。未触发告警的一切行为与归零重拉基线逐字节相同,
    名义工况与失配 fallback 路径按构造继承。

为何门必须用精确窗差界 (而非锚点瞬间加速度的一阶代理):
    一阶代理 |ȧ_own(锚点)|·δ 是单一瞬间的泰勒近似, 在 counts 量化下"是否
    恰好为 0"近乎掷硬币, 门近乎随机开合 — 垃圾种子随门漏进失配档。精确窗
    差界覆盖整个 run 窗口与真实量化, 把门与伪创新物理对齐。残余代价: L70
    比归零重拉高 ~0.5 — L_真 > Lc 时帧外盲区 (最后 ~15ms) 中的目标变向与
    读数噪声不可观测, 是重播种的内在价格; 换来的是 maneuver/FPS 尾迹收益
    与全部失配/灵敏度档的通过。

实测 (arena 默认种子 1,2,3):
    matched 146.64 (step 两场景/const_vel/accel 为归零重拉同款名义路径;
    maneuver rmse 17.48, in_band 23.7%)。
    失配扫描 L30..70 = {83.68, 66.55, 81.09, 83.13, 125.58}; 宽延迟
    L20-80 = {91.40, 80.26, 67.60, 78.25, 87.51, 121.90, inf} (L80 为
    settle 刀锋); s 失配 0.7-1.3 = {102.75, 77.52, 78.25, 92.49, 109.96};
    relock 386.0ms·3.16px; fps_eval clean RMSE 20.68 / flaky 21.51;
    OVERALL 161.54。holdout (seeds 4,5,6): worst 123.31, 无发散。
    帧率: 60fps composite 165.29 vs 120fps 146.64 (差 12.7%) — 逐场景看
    60fps 退化全部在 maneuver (+0.5px, +2.8%), step/const_vel/accel 逐位
    一致; 120fps 侧 matched 变好使比值分母变小, 放大了百分数。

参数 (全部无量纲设计选择或数值分辨率, 无手感量):
    RUN_MAX = 5        矛盾 run 记录的创新帧数上限 (CUSUM 告警延迟
                       2-3 帧 + 少量裕量)
    BAND_FRAC = 0.6    延迟先验带半宽 = 0.6·L̂ (宽延迟验证带 L20-80 的相对
                       半宽; 先验来源 = 标定精度要求)
    J_GRID = 0.5       J 界的移位扫描网格 = 0.5·控制拍 (数值分辨率: counts
                       历史按拍分段线性, 半拍保证过所有折点; 非行为参数)
    J_AGG = "max"      run 内 J 聚合取最坏读数 (界语义: max ≥ mean, 不依赖
                       单读数低估的侥幸)
    3σ_seed            播种显著性 (标准显著性常数)
    CUSUM K/C/H = 0.5/3/9σ, β0=0.03, PM=50, ζ=1, L_COMP=1.1, I_GATE=8 …
                       与 ff_pi_acc 相同, 依据见其 docstring。

debug() 字段语义 (px / px/ms / px/ms² / σ / 0-1):
  ex, ey           Smith 汇装误差 ê
  fvx, fvy         α-β 速度估计 v̂ (告警决策后)
  fx, fy           α-β 位置估计
  csx, csy         CUSUM 状态 S (σ 单位)      sigx/sigy: 尺度 σ̂
  w_state          FF 信任度 0..1              w_inst: 告警电平 (未滤波)
  gate             I 项距离门控                ff_eff: 实际前馈有效增益
  age              当前拍距最新检测 ms
  run_x, run_y     矛盾 run 当前帧数 (0..RUN_MAX)
  aown_x, aown_y   本帧锚点处自身加速度 (px/ms², counts 历史精确差分; 诊断:
                   伪创新物理来源, 门本身用精确窗差界不用它)
  alm_act_x/y      最近一次告警动作: 1=播种, 0=归零, -1=尚无
  alm_raw_x/y      告警时原始种子 mean(νᵢ/Tᵢ+v̂ᵢ) (px/ms)
  alm_vpre_x/y     告警时 v̂_pre (px/ms)
  alm_sigsd_x/y    种子噪声 σ_seed = σ̂/(T̄·√n) (px/ms)
  alm_junk_x/y     精确窗差上界 J (px/ms)
  alm_dv0_x/y      收缩前 dv0 = raw − v̂_pre (px/ms)
  alm_dv_x/y       决策后 dv (px/ms; 归零动作时为 0)
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
from arena.core import Observation, LawConfig
from arena.laws.base import CountsHist, Law, register


@register("reseed_pi")
class ReseedPILaw(Law):
    DT0 = 1000.0 / 120.0
    JUMP_GATE = 100.0
    STALE = 200.0
    # CUSUM 参数 (均为 σ 倍数, 无量纲)
    CUSUM_K = 0.5
    CUSUM_C = 3.0
    CUSUM_H = 9.0
    BETA0 = 0.03                       # 出厂速度增益
    RUN_MAX = 5                        # 矛盾 run 记录的创新帧数上限
    BAND_FRAC = 0.6                    # 延迟先验带半宽 = 0.6·L̂ (宽延迟验证带 L20-80)
    J_GRID = 0.5                       # J 界的移位网格 = 0.5·控制拍 (数值分辨率:
                                       # counts 历史按拍分段线性, 半拍必过所有折点)
    J_AGG = "max"                      # run 内 J 聚合: "max" | "mean" (max = 以 run
                                       # 内最坏读数的界为界, 免单读数低估之侥幸)

    def __init__(self, **kw):
        kw.setdefault("pm_deg", 50.0)
        kw.setdefault("beta0", self.BETA0)
        self.zeta = kw.pop("zeta", 1.0)
        self.ff_gain = kw.pop("ff_gain", 1.0)
        self.l_comp = kw.pop("l_comp", 1.1)
        self.alpha0 = kw.pop("alpha0", 0.50)
        self.i_gate = kw.pop("i_gate", 8.0)
        self.i_frac = kw.pop("i_frac", 1.0)
        self.pm_deg = kw.pop("pm_deg")
        self.beta0 = kw.pop("beta0")
        self._max_v = kw.pop("max_v", 1.5)
        self._dbg = {}

    def reset(self, cfg: LawConfig):
        self.cfg = cfg
        self.max_v = self._max_v if self._max_v > 0 else cfg.max_v
        L = max(1.0, cfg.L)
        wn = (90.0 - self.pm_deg) * math.pi / 180.0 / L
        self.kp = 2.0 * self.zeta * wn
        self.ki = wn * wn
        self.ch = CountsHist()
        self.filt = False
        self.fx = self.fy = self.fvx = self.fvy = 0.0
        self.prev_det_t = None
        self.t_pub = -1e9
        self.int_x = self.int_y = 0.0
        self.rem_x = self.rem_y = 0.0
        self.sig2x = self.sig2y = 1.0
        self.csx = self.csy = 0.0
        # 重播种状态: run = [(ν, v̂_pre, T, 帧时刻), ...] (告警时消费, 随后清空)
        self.run_x: list = []
        self.run_y: list = []
        self._w_state = 0.0
        self._w_inst = 0.0
        self._aown_x = 0.0
        self._aown_y = 0.0
        self._alm = {
            "act_x": -1.0, "act_y": -1.0,
            "raw_x": 0.0, "raw_y": 0.0,
            "vpre_x": 0.0, "vpre_y": 0.0,
            "sigsd_x": 0.0, "sigsd_y": 0.0,
            "junk_x": 0.0, "junk_y": 0.0,
            "dv0_x": 0.0, "dv0_y": 0.0,
            "dv_x": 0.0, "dv_y": 0.0,
        }
        self._dbg = {}

    def _w_update(self, cfg: LawConfig, h: float) -> float:
        """信任度: CUSUM 告警电平 → 非对称滤波。"""
        self._w_inst = min(1.0, max(self.csx, self.csy) / self.CUSUM_H)
        a = 1.0 - math.exp(-h / (2.0 * self.DT0))
        d = 1.0 - math.exp(-h / max(1.0, cfg.L))
        rate = a if self._w_inst > self._w_state else d
        self._w_state += rate * (self._w_inst - self._w_state)
        return self._w_state

    def _own_accel(self, det_t: float, ax: int) -> float:
        """锚点 a = det_t − Lc 处自身加速度 (px/ms²), counts 历史精确差分。
        纯诊断用 (debug 暴露) — 门本身用精确窗差界。"""
        cfg = self.cfg
        Lc = cfg.L * self.l_comp
        a = det_t - Lc
        h = self.cfg.h
        own_front = cfg.s * (self.ch.at(a + 0.5 * h)[ax]
                             - self.ch.at(a - 0.5 * h)[ax]) / h
        own_back = cfg.s * (self.ch.at(a)[ax]
                            - self.ch.at(a - h)[ax]) / h
        return (own_front - own_back) / h

    def _seed(self, run: list, cur_v: float) -> float:
        """矛盾 run 的创新斜率外推: v_真 ≈ mean(νᵢ/Tᵢ + v̂ᵢ)。"""
        n = len(run)
        return sum(nu / T + v for nu, v, T, _t in run) / n if n else cur_v

    def _junk_bound(self, run: list, ax: int) -> float:
        """种子垃圾上界 J (px/ms): 真窗口在标定先验带内滑动时, 自身流量差
        能给读数注入的最大污染 (run 内最坏读数)。精确可计算 — counts 历史
        是法则自己发出的指令记录。"""
        if not run:
            return 0.0
        cfg = self.cfg
        Lc = cfg.L * self.l_comp
        delta = self.BAND_FRAC * cfg.L
        off0 = (self.l_comp - 1.0) * cfg.L          # f−a 的中心 = Lc − L̂
        lo = off0 - delta
        hi = off0 + delta
        n_sh = int((hi - lo) / (self.J_GRID * cfg.h)) + 1
        Js = []
        for (nu, vpre, T, ti) in run:
            a = ti - Lc
            Fa = cfg.s * (self.ch.at(a)[ax] - self.ch.at(a - T)[ax])
            worst = 0.0
            for j in range(n_sh + 1):
                fp = a + lo + j * (self.J_GRID * cfg.h)
                Ff = cfg.s * (self.ch.at(fp)[ax] - self.ch.at(fp - T)[ax])
                d = abs(Fa - Ff)
                if d > worst:
                    worst = d
            Js.append(worst / T)
        return max(Js) if self.J_AGG == "max" else sum(Js) / len(Js)

    def _alarm(self, ax: int, det_t: float):
        """告警动作: 证据门 — 种子读数减去精确窗差上界后仍显著才播种,
        否则按归零重拉原样归零。"""
        run = self.run_x if ax == 0 else self.run_y
        v = self.fvx if ax == 0 else self.fvy
        sig = math.sqrt(self.sig2x) if ax == 0 else math.sqrt(self.sig2y)
        n = len(run)
        seed = self._seed(run, v)
        tbar = sum(r[2] for r in run) / n
        sig_seed = sig / tbar / math.sqrt(n)
        J = self._junk_bound(run, ax)
        dv0 = seed - v
        dv = max(0.0, abs(dv0) - J) * (1.0 if dv0 > 0 else -1.0)
        act = abs(dv) >= 3.0 * sig_seed
        nv = v + dv if act else 0.0
        suf = "x" if ax == 0 else "y"
        self._alm["act_" + suf] = 1.0 if act else 0.0
        self._alm["raw_" + suf] = seed
        self._alm["vpre_" + suf] = v
        self._alm["sigsd_" + suf] = sig_seed
        self._alm["junk_" + suf] = J
        self._alm["dv0_" + suf] = dv0
        self._alm["dv_" + suf] = dv if act else 0.0
        if ax == 0:
            self.fvx = nv
            self.run_x = []
        else:
            self.fvy = nv
            self.run_y = []

    def _update_filter(self, det: Observation):
        cfg = self.cfg
        if self.prev_det_t is None:
            self.fx, self.fy = det.dx, det.dy
            self.fvx = self.fvy = 0.0
            self.sig2x = self.sig2y = 1.0
            self.csx = self.csy = 0.0
            self.filt = True
            self.prev_det_t = det.t
            self.t_pub = det.t
            return
        dt = max(1.0, min(100.0, det.t - self.prev_det_t))
        Lc = cfg.L * self.l_comp
        c0 = self.ch.at(det.t - Lc - dt)
        c1 = self.ch.at(det.t - Lc)
        px_pred = self.fx + self.fvx * dt - cfg.s * (c1[0] - c0[0])
        py_pred = self.fy + self.fvy * dt - cfg.s * (c1[1] - c0[1])
        inx = det.dx - px_pred
        iny = det.dy - py_pred
        if math.hypot(inx, iny) > self.JUMP_GATE:
            self.fx, self.fy = det.dx, det.dy
            self.fvx = self.fvy = 0.0
            self.csx = self.csy = 0.0
            self.run_x = []
            self.run_y = []
        else:
            r = dt / self.DT0
            alpha = min(0.90, self.alpha0 * r)
            beta_s = min(0.60, self.BETA0 * r)
            self.sig2x += beta_s * (inx * inx - self.sig2x)
            self.sig2y += beta_s * (iny * iny - self.sig2y)
            sx = max(math.sqrt(self.sig2x), 1e-6)
            sy = max(math.sqrt(self.sig2y), 1e-6)
            if self.fvx > 0:
                accx = -inx / sx
            elif self.fvx < 0:
                accx = inx / sx
            else:
                accx = -self.CUSUM_K
            if self.fvy > 0:
                accy = -iny / sy
            elif self.fvy < 0:
                accy = iny / sy
            else:
                accy = -self.CUSUM_K
            self.csx = max(0.0, self.csx + min(max(accx, 0.0), self.CUSUM_C)
                           - self.CUSUM_K)
            self.csy = max(0.0, self.csy + min(max(accy, 0.0), self.CUSUM_C)
                           - self.CUSUM_K)
            # 矛盾 run 记录 (告警动作用的创新斜率读数): (ν, v̂_pre, T, 帧时刻)
            if accx > 0.0 and self.fvx != 0.0:
                self.run_x.append((inx, self.fvx, dt, det.t))
                if len(self.run_x) > self.RUN_MAX:
                    self.run_x.pop(0)
            if accy > 0.0 and self.fvy != 0.0:
                self.run_y.append((iny, self.fvy, dt, det.t))
                if len(self.run_y) > self.RUN_MAX:
                    self.run_y.pop(0)
            # 告警 → 证据门重播种/归零 (该轴)
            if self.csx >= self.CUSUM_H and self.fvx != 0.0:
                self._alarm(0, det.t)
                self.csx = 0.0
            if self.csy >= self.CUSUM_H and self.fvy != 0.0:
                self._alarm(1, det.t)
                self.csy = 0.0
            self.fvx += (beta_s / dt) * inx
            self.fvy += (beta_s / dt) * iny
            self.fx = px_pred + alpha * inx
            self.fy = py_pred + alpha * iny
        # 诊断: 本帧锚点处自身加速度 (debug 暴露, 伪创新物理来源)
        self._aown_x = self._own_accel(det.t, 0)
        self._aown_y = self._own_accel(det.t, 1)
        self.prev_det_t = det.t
        self.t_pub = det.t

    def step(self, t: float, obs: Optional[Observation]) -> Tuple[int, int]:
        cfg = self.cfg
        if obs is not None and obs.new:
            self._update_filter(obs)

        cx = cy = 0
        age = t - self.t_pub
        if self.filt and age < self.STALE:
            w_state = self._w_update(cfg, cfg.h)
            Lc = cfg.L * self.l_comp
            cp = self.ch.at(self.t_pub - Lc)
            cn = self.ch.cum()
            ex = self.fx + self.fvx * (age + Lc) - cfg.s * (cn[0] - cp[0])
            ey = self.fy + self.fvy * (age + Lc) - cfg.s * (cn[1] - cp[1])
            r = math.hypot(ex, ey)
            gate = self.i_gate / (self.i_gate + r) if self.i_gate > 0 else 1.0

            i_lim = self.i_frac * self.max_v / max(self.ki, 1e-9)
            vx_u = self.kp * ex + self.ki * self.int_x
            vy_u = self.kp * ey + self.ki * self.int_y

            if ex * ex + ey * ey > cfg.fov_radius * cfg.fov_radius:
                self.int_x = self.int_y = 0.0
            else:
                wx = (vx_u > self.max_v and ex > 0) or (vx_u < -self.max_v and ex < 0)
                wy = (vy_u > self.max_v and ey > 0) or (vy_u < -self.max_v and ey < 0)
                if not wx:
                    self.int_x = max(-i_lim, min(i_lim, self.int_x + ex * cfg.h * gate))
                if not wy:
                    self.int_y = max(-i_lim, min(i_lim, self.int_y + ey * cfg.h * gate))

            # FF 门控 = 信任度插值
            frame_dt = cfg.frame_dt if cfg.frame_dt > 0 else self.DT0
            gap_scale = 1.0 - max(0.0, min(1.0, (age - frame_dt) / max(1.0, cfg.L)))
            ff_gate = gate + (1.0 - gate) * (1.0 - w_state)
            ff_eff = self.ff_gain * ff_gate * gap_scale
            vx_u += ff_eff * self.fvx
            vy_u += ff_eff * self.fvy

            vx = max(-self.max_v, min(self.max_v, vx_u))
            vy = max(-self.max_v, min(self.max_v, vy_u))
            s = max(0.05, min(20.0, cfg.s))
            self.rem_x += vx * cfg.h / s
            self.rem_y += vy * cfg.h / s
            cx, cy, self.rem_x, self.rem_y = self._counts(
                self.rem_x, self.rem_y, s, cfg.count_limit)
        else:
            self.rem_x = self.rem_y = 0.0
            self.int_x = self.int_y = 0.0
            r = 0.0
            gate = 0.0
            ex = ey = 0.0
            w_state = self._w_state
            ff_eff = 0.0

        self.ch.add(t, cx, cy)
        self._dbg = {
            "ex": ex, "ey": ey, "r": r,
            "fvx": self.fvx, "fvy": self.fvy,
            "fx": self.fx, "fy": self.fy,
            "int_x": self.int_x, "int_y": self.int_y,
            "csx": self.csx, "csy": self.csy,
            "sigx": math.sqrt(self.sig2x), "sigy": math.sqrt(self.sig2y),
            "w_state": w_state, "w_inst": self._w_inst,
            "ff_eff": ff_eff, "gate": gate,
            "age": age,
            "run_x": float(len(self.run_x)), "run_y": float(len(self.run_y)),
            "aown_x": self._aown_x, "aown_y": self._aown_y,
            "alm_act_x": self._alm["act_x"], "alm_act_y": self._alm["act_y"],
            "alm_raw_x": self._alm["raw_x"], "alm_raw_y": self._alm["raw_y"],
            "alm_vpre_x": self._alm["vpre_x"], "alm_vpre_y": self._alm["vpre_y"],
            "alm_sigsd_x": self._alm["sigsd_x"], "alm_sigsd_y": self._alm["sigsd_y"],
            "alm_junk_x": self._alm["junk_x"], "alm_junk_y": self._alm["junk_y"],
            "alm_dv0_x": self._alm["dv0_x"], "alm_dv0_y": self._alm["dv0_y"],
            "alm_dv_x": self._alm["dv_x"], "alm_dv_y": self._alm["dv_y"],
        }
        return cx, cy

    def debug(self) -> dict:
        return dict(self._dbg)
