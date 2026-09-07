"""arena/laws/ff_pi2.py — ff_pi + 方向矛盾 CUSUM 速度归零重拉 ("ffpi2", 主律)。

与 ff_pi 的差异 (极点配置骨架不变, wn=(90°−PM)π/180/L 照旧):

原理: 速度型目标上的一切大过冲, 根源都是"目标模型破缺后 v̂ 成为幽灵" —
它同时污染 FF (朝旧方向推) 和 Smith 预测 (把误差掩盖, P 看不见真实拖尾
→ 回拉"面")。急停与变向 (ADAD/蹬墙跳) 都是这一种破缺。

处理方式 (归零重拉): 检测到"创新持续与 v̂ 矛盾"时, 把该轴 v̂ 归零、保留
位置估计 — 环路回到与阶跃响应完全相同的初始条件 (无幽灵投影, P 全程
看见真实误差; FF 从 0 随 v̂ 朝正确方向重建, 逐渐介入无踢脚)。阶跃响应是
电池里调得最好的工况 (settle 277ms / over 3.1px), 变向/急停因此复用同一
套已被验证的行为。

检测 = 双向 CUSUM (Page 序贯变化检测), 仅累计与 v̂ 矛盾方向的创新:
  S± = max(0, S± + clip(∓sign(v̂)·in/σ, 0, C) − K),  告警 S > H → 该轴 v̂ 归零
  K 漂移 (0.5σ): 噪声随机游走被负漂移压回零;  C 帧增量上限 (3σ): 单帧踢脚
  (后坐力) 不可一次越限;  H 告警门限 (9σ): 同号持续 ~2 帧即触发。
  v̂≈0 或创新与 v̂ 同向时不累计 (重建期间的追击创新不会反复触发重置)。
σ 在线自标定 (创新方差 EMA): 无任何绝对 px 常数 — K/C/H 均为 σ 倍数,
噪声越大门自动越宽 (设备自适应); 重建/量化噪声不误触发。

丢帧衰减 (沿袭): 检测中断时 FF 按标定 L 时间尺度撤回, 盲推上界
v̂·STALE(200ms) → ~v̂·L。

设计点 (失配带电池选定, 非手感): pm_deg=50 (全延迟带 L20-80 通过的最快
点, wn+26%/Ki+59%); beta0=0.03 (带边缘余量)。
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
from arena.core import Observation, LawConfig
from arena.laws.base import register
from arena.laws.ff_pi import FFPILaw


@register("ff_pi2")
class FFPI2Law(FFPILaw):
    # CUSUM 参数 (均为 σ 倍数, 无量纲 — Page 序贯变化检测标准结构):
    # K = 漂移 0.5σ (噪声负漂移); C = 单帧增量上限 3σ (拒单帧踢脚);
    # H = 告警门限 9σ (同号持续 ~2 帧触发)
    CUSUM_K = 0.5
    CUSUM_C = 3.0
    CUSUM_H = 9.0

    def __init__(self, **kw):
        kw.setdefault("pm_deg", 50.0)
        kw.setdefault("beta0", 0.03)
        super().__init__(**kw)

    def reset(self, cfg: LawConfig):
        super().reset(cfg)
        self.sig2x = self.sig2y = 1.0   # 创新方差在线估计 (px², 自标定)
        self.csx = self.csy = 0.0       # CUSUM 状态 (σ 单位)

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
            beta_s = min(0.60, self.beta0 * r ** self.beta_exp)
            self.sig2x += beta_s * (inx * inx - self.sig2x)
            self.sig2y += beta_s * (iny * iny - self.sig2y)
            # 双向 CUSUM, 只累计与 v̂ 矛盾方向的创新 (σ 归一):
            #   矛盾 = 创新方向与 v̂ 相反 (目标模型说"目标还在朝 v̂ 走",
            #   测量说"没有" — 急停/变向的签名)。v̂≈0 时不累计 (无可矛盾)。
            sx = max(math.sqrt(self.sig2x), 1e-6)
            sy = max(math.sqrt(self.sig2y), 1e-6)
            if self.fvx > 0:
                accx = -inx / sx
            elif self.fvx < 0:
                accx = inx / sx
            else:
                accx = -self.CUSUM_K            # v̂≈0: 无可矛盾, 漂移归零
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
            # 告警 → 该轴速度归零 (位置估计保留): 环路重跑阶跃响应
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

            # FF = 门控 × 丢帧衰减 (v̂ 已由 CUSUM 归零机制保证可信)
            frame_dt = cfg.frame_dt if cfg.frame_dt > 0 else self.DT0
            gap_scale = 1.0 - max(0.0, min(1.0, (age - frame_dt) / max(1.0, cfg.L)))
            ff_eff = self.ff_gain * gate * gap_scale
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
