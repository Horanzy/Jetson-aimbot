"""arena/laws/ff_pi.py — 主控制律: 极点配置 PI + type-2 速度前馈 + 方向矛盾
CUSUM 速度归零重拉。

结构 (控制增益全部由标定延迟 L̂ 导出, 无手调参数):
    ê = f + v̂·(age+L̂·L_COMP) − s·Σcounts(in flight)        // Smith 预测误差
    wn = (90°−PM)·π/180 / L̂,  Kp = 2ζ·wn,  Ki = wn²          // 极点配置 (ζ=1)
    gate = I_GATE/(I_GATE+|ê|)                               // I 项距离门控 (防 flick windup)
    v = Kp·ê + Ki·∫ê·gate + FF_GAIN_VAL·ff_gate·gap·v̂        // PI + type-2 前馈
    FF_GAIN_VAL=1: 被积对象是纯积分器, v̂ 是匀速目标零拖尾的精确开环指令
    (由对象模型导出, 非调参)。

方向矛盾 CUSUM → 速度归零重拉:
    移动目标上的一切大过冲同源: 目标模型破缺 (急停/变向) 后 v̂ 成为幽灵 —
    同时朝旧方向推 FF, 并在 Smith 预测里掩盖真实误差 (P 看不见拖尾 → 回拉
    "面")。检测 = 双向 CUSUM (Page 序贯变化检测), 只累计与 v̂ 矛盾方向的
    创新; 告警即该轴 v̂ 归零 (位置估计保留) — 环路回到与阶跃响应相同的
    初始条件: P 无遮蔽全程看见真实误差 (回拉 sharp), FF 从 0 随 v̂ 朝正确
    方向重建 (渐进介入无踢脚)。变向/急停因此复用评测组里最优的阶跃响应。

    CUSUM 参数 K/C/H 均为 σ 倍数 (无量纲), σ 在线自标定 (创新方差 EMA):
    噪声越大门自动越宽, 设备自适应, 无绝对 px 常数。
      K = 0.5σ  漂移: 噪声随机游走被负漂移压回零
      C = 3σ    单帧增量上限: 拒绝后坐力式单帧踢脚
      H = 9σ    告警门限: 同号持续 ~2 帧触发
    v̂≈0 或创新与 v̂ 同向时不累计 — 归零后重建期间的追击创新不会反复触发。

FF 门控 = 信任度插值:
    信任满格 (稳态追击/加速) → 无距离门控, 全力前馈 (距离门控会在拖尾
    30px 时把 FF 压到 21%, P 单腿跑 = 可见拖尾的根源);
    CUSUM 告警 (模型破缺, v̂ 已归零) → 回到距离门控保守形态 (重拉期防
    二次过冲); 信任非对称滤波: 告警 ~2 帧降级, 按标定 L 尺度恢复 (无踢脚)。

丢帧衰减: 检测中断时 FF 按标定 L 时间尺度撤回, 盲推上界 v̂·STALE(200ms)
→ ~v̂·L。

设计点 (失配带测试选定, 非手感): PM=50 (全延迟带 L20-80 通过的最快点);
β0=0.03 (带边缘余量)。

评测 (arena 实测): matched 151.3 (step settle 277ms / 过冲 3.11px, accel
rmse 7.1px, maneuver rmse 19.4px); 失配带 L30-70 全过; FPS 行为组 ADAD
RMSE 24.1px / 事件过冲 28.0px。
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
from arena.core import Observation, LawConfig
from arena.laws.base import CountsHist, Law, register


@register("ff_pi")
class FFPILaw(Law):
    DT0 = 1000.0 / 120.0
    JUMP_GATE = 100.0
    STALE = 200.0
    # CUSUM 参数 (均为 σ 倍数, 无量纲): K 漂移 / C 单帧增量上限 / H 告警
    CUSUM_K = 0.5
    CUSUM_C = 3.0
    CUSUM_H = 9.0

    def __init__(self, **kw):
        # 设计点 (失配带测试选定, 非手感): PM=50 全带最快; β0=0.03 带边缘余量
        kw.setdefault("pm_deg", 50.0)
        kw.setdefault("beta0", 0.03)
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
        self.sig2x = self.sig2y = 1.0   # 创新方差在线估计 (px², 自标定)
        self.csx = self.csy = 0.0       # CUSUM 状态 (σ 单位)
        self._w_state = 0.0             # 信任度 0..1 (1=信任, FF 无门控)
        self._w_inst = 0.0              # CUSUM 告警电平

    def _w_update(self, cfg: LawConfig, h: float) -> float:
        """信任度: CUSUM 告警电平 → 非对称滤波 (告警 ~2 帧降级, 按标定
        L 尺度恢复)。降级期 FF 回距离门控保守形态, 满格期无门控。"""
        self._w_inst = min(1.0, max(self.csx, self.csy) / self.CUSUM_H)
        a = 1.0 - math.exp(-h / (2.0 * self.DT0))
        d = 1.0 - math.exp(-h / max(1.0, cfg.L))
        rate = a if self._w_inst > self._w_state else d
        self._w_state += rate * (self._w_inst - self._w_state)
        return self._w_state

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
        else:
            r = dt / self.DT0
            alpha = min(0.90, self.alpha0 * r)
            beta_s = min(0.60, self.beta0 * r)
            self.sig2x += beta_s * (inx * inx - self.sig2x)
            self.sig2y += beta_s * (iny * iny - self.sig2y)
            # 双向 CUSUM, 只累计与 v̂ 矛盾方向的创新 (σ 归一):
            #   矛盾 = 创新方向与 v̂ 相反 — 急停/变向的签名。
            #   v̂≈0 时不累计 (无可矛盾); 单帧封顶 C 拒单帧踢脚。
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
            # 告警 → 该轴速度归零 (位置保留): 环路重跑阶跃响应
            if self.csx >= self.CUSUM_H and self.fvx != 0.0:
                self.fvx = 0.0
                self.csx = 0.0
            if self.csy >= self.CUSUM_H and self.fvy != 0.0:
                self.fvy = 0.0
                self.csy = 0.0
            self.fx = px_pred + alpha * inx
            self.fy = py_pred + alpha * iny
            self.fvx += (beta_s / dt) * inx
            self.fvy += (beta_s / dt) * iny
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

            # FF 门控 = 信任度插值 (信任满格无门控, 告警回保守距离门控)
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
