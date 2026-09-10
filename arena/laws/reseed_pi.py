"""arena/laws/reseed_pi.py — 极点配置 PI + type-2 前馈 + 方向矛盾 CUSUM
告警重播种 (模型破缺后 v̂ 从阶跃读数直达, 不从零重建)。
落选: 失配带付出大于尾迹收益, 见文末实测与结论。

结构 (与 ff_pi 逐字节相同的控制回路与估计器, 只改 CUSUM 告警动作):
    ê = f + v̂·(age+L̂·L_COMP) − s·Σcounts(in flight)        // Smith 预测误差
    wn = (90°−PM)·π/180 / L̂,  Kp = 2ζ·wn,  Ki = wn²          // 极点配置 (ζ=1)
    gate = I_GATE/(I_GATE+|ê|)                               // I 项距离门控
    v = Kp·ê + Ki·∫ê·gate + FF_GAIN_VAL·ff_gate·gap·v̂        // PI + type-2 前馈
    方向矛盾 CUSUM (σ 归一) 告警 → **重播种** (ff_pi 是归零重拉)。

原理: 矛盾 run 期间逐帧创新 νᵢ ≈ (v_真 − v̂ᵢ)·Tᵢ + 噪声, 本身就是速度
阶跃的直接读数:
    v̂_new = mean(νᵢ/Tᵢ + v̂ᵢ)        (run 内逐帧, ≤5 帧)
种子噪声 σ_seed = σ̂/(T̄·√n) (每帧 νᵢ/Tᵢ 的噪声 σ̂/Tᵢ 独立平均), 2-3 帧 ×
0.5px → σ_seed ≈ 0.035px/ms, 对 0.5px/ms 量级的速度阶跃一步到位到 ~7%;
残差由原 β0 慢速精化 — 重建尾从 "从零爬 τ = dt/β0 ≈ 280ms" 变为
"从 ~93% 直接驻留"。

失配纯度防线 (种子质量的门):
    窗口错位 Δc=Lc−L_真 使自身指令瞬态以 −ȧ_own·Δc 混入创新斜率。精确
    扣除需要 own_true (τ_k 时刻的自身流量) — 只能由真实延迟给出, 不可
    观测 (突发不可重构, 只可检测)。两级检测:
      1. 显著性: |seed − v̂| < 3σ_seed → 噪声告警 (v̂≈0 时符号被噪声翻转
         的 CUSUM 告警匀速期偶发), 按 ff_pi 原样归零。3σ = 标准显著性常数。
      2. 不确定度收缩: 退化方向还受锚点处自身加速度 |ȧ_own| (可测) ×
         不确定度半宽 δ = 0.6·L̂ (宽延迟验证带) 的污染上界约束 — 从种子
         变化量中收缩掉这个界 (保守估计: 减去偏差上界, 保留方向), 收缩后
         不足 3σ 显著性同样归零。
    未触发告警的一切行为与 ff_pi 逐字节相同 — 名义工况与失配 fallback
    路径按构造继承。

arena 成绩 (实测, eval @匹配 L=50/120fps 噪声0.5): matched 148.35
(step 与 const_vel/accel 与 ff_pi 逐位一致; maneuver rmse 17.68 vs
ff_pi 19.41); 失配扫描 L30..70 = {86.0, 85.2, 82.8, 87.9, 133.9}, 最坏
133.9 (ff_pi 125.1); 帧率差 10.3% (ff_pi 9.2%); OVERALL 161.8
(ff_pi 156.7)。integrate 评测: matched/relock 与基线一致 (386ms·3.16px),
L20-80 无新增发散档, L80 仍为基线同款 settle 刀锋。

为何落选:
    收益全部在尾迹 (maneuver −9%、fps flaky RMSE −0.37px、摩擦急停事件
    过冲 −18%), 而 FPS 事件过冲峰值 (mean 28px) 几乎不动 — 峰值由延迟
    下界 v·L + CUSUM 告警延迟 (~2-3 帧, 受 C=3σ 单帧上限的防误告纪律
    约束) 决定, 重播种只影响峰值之后的重建段。失配侧 L60/L70 劣化
    (+3/+9) 来自种子在锚点不确定度内吸入自身加速度污染 — 收缩防线能压
    但压不净 (δ 内的符号效应不可分辨), OVERALL 净退步。失配带再次被
    证实是本对象/传感器族的硬边界; 除非标定精度把 |Δc| 压到 ~5ms 以内
    (2ms 细扫已是实用下限), 否则种子机制得不偿失。

CUSUM 参数与 σ 自标定 (K/C/H = 0.5/3/9σ, β0=0.03, PM=50, ζ=1 …) 全部
与 ff_pi 相同, 依据见 ff_pi docstring。
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
    # CUSUM 参数 (均为 σ 倍数, 无量纲): 同 ff_pi
    CUSUM_K = 0.5
    CUSUM_C = 3.0
    CUSUM_H = 9.0
    BETA0 = 0.03                       # 出厂速度增益 (与 ff_pi 一致)
    RUN_MAX = 5                        # 矛盾 run 记录的创新帧数上限
    BAND_FRAC = 0.6                    # 延迟不确定度半宽 = 0.6·L̂ (宽延迟验证带 L20-80)

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
        # 重播种状态: run = [(ν, v̂_pre, T, j), ...] (告警时消费, 随后清空)
        self.run_x: list = []
        self.run_y: list = []
        self._w_state = 0.0
        self._w_inst = 0.0

    def _w_update(self, cfg: LawConfig, h: float) -> float:
        """信任度: CUSUM 告警电平 → 非对称滤波 (同 ff_pi)。"""
        self._w_inst = min(1.0, max(self.csx, self.csy) / self.CUSUM_H)
        a = 1.0 - math.exp(-h / (2.0 * self.DT0))
        d = 1.0 - math.exp(-h / max(1.0, cfg.L))
        rate = a if self._w_inst > self._w_state else d
        self._w_state += rate * (self._w_inst - self._w_state)
        return self._w_state

    def _seed(self, run: list, cur_v: float) -> float:
        """矛盾 run 的创新斜率外推: v_真 ≈ mean(νᵢ/Tᵢ + v̂ᵢ)。"""
        n = len(run)
        return sum(nu / T + v for nu, v, T in run) / n if n else cur_v

    def _alarm(self, ax: int, det_t: float):
        """告警动作: 重播种, 种子按不确定度界收缩, 不足显著性则归零。

        种子污染: 窗口错位 Δc=Lc−L_真 使自身加速度以 −ȧ_own·Δc 混入
        创新斜率。Δc 不可观测 (精确扣除需要 L_真), 但其上界可测:
        锚点 a=t−Lc 处的自身加速度 |ȧ_own| × 不确定度半宽 δ=0.6·L̂
        (宽延迟验证带)。从种子变化量中收缩掉这个界 (标准保守估计:
        减去偏差上界, 保留方向); 收缩后不足 3σ 显著性 → 噪声告警/
        纯垃圾矛盾, 按 ff_pi 原样归零。"""
        run = self.run_x if ax == 0 else self.run_y
        v = self.fvx if ax == 0 else self.fvy
        sig = math.sqrt(self.sig2x) if ax == 0 else math.sqrt(self.sig2y)
        n = len(run)
        seed = self._seed(run, v)
        tbar = sum(T for _, _, T in run) / n
        sig_seed = sig / tbar / math.sqrt(n)
        cfg = self.cfg
        Lc = cfg.L * self.l_comp
        a = det_t - Lc
        h = self.cfg.h
        own_front = cfg.s * (self.ch.at(a + 0.5 * h)[ax]
                             - self.ch.at(a - 0.5 * h)[ax]) / h
        own_back = cfg.s * (self.ch.at(a)[ax]
                            - self.ch.at(a - h)[ax]) / h
        acc_own = (own_front - own_back) / h
        dv = seed - v
        dv = max(0.0, abs(dv) - abs(acc_own) * self.BAND_FRAC * cfg.L) \
            * (1.0 if dv > 0 else -1.0)
        if abs(dv) < 3.0 * sig_seed:
            v = 0.0
        else:
            v = v + dv
        if ax == 0:
            self.fvx = v
            self.run_x = []
        else:
            self.fvy = v
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
            # 矛盾 run 记录 (告警动作用的创新斜率读数): (ν, v̂_pre, T)
            if accx > 0.0 and self.fvx != 0.0:
                self.run_x.append((inx, self.fvx, dt))
                if len(self.run_x) > self.RUN_MAX:
                    self.run_x.pop(0)
            if accy > 0.0 and self.fvy != 0.0:
                self.run_y.append((iny, self.fvy, dt))
                if len(self.run_y) > self.RUN_MAX:
                    self.run_y.pop(0)
            # 告警 → 重播种/归零 (该轴)
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

            # FF 门控 = 信任度插值 (同 ff_pi)
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

        self.ch.add(t, cx, cy)
        return cx, cy
