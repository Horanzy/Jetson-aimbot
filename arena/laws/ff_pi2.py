"""arena/laws/ff_pi2.py — ff_pi + 机动撤回 FF + 丢帧 FF 衰减 ("ffpi2", 主律候选)。

与 ff_pi 的差异 (极点配置骨架不变, wn=(90°−PM)π/180/L 照旧; 只动 FF 的
开关与两个设计点):

原理: FF 是环路里唯一的开环项 — 目标模型错了它就带着准星冲错方向
(急停后的幽灵 v̂、折返后的反向推、丢帧中的航位推算全是它)。当测量证据
说"模型破缺"时, 把 FF 撤掉让闭环 P/I 接管, 比重新定向 FF 更稳 (定向需要
可信的快速 v̂, 真实噪声下正是弱点, 且抬高估计环路增益 → 失配发散, 实测)。

    ff_eff = ff_gain · gate · (1 − w_eff) · gap_scale · v̂

w 的测量 (全部复用已有常数, 无新绝对量):
  hp = 创新 − 创新均值; 均值以 beta_s 衰减 (高通去偏置: L 失配时 Smith
    计数窗错位的 ~v·ΔL 持续系统创新被均值吸收, 不误判为机动。实测直接用
    |in| 会在 L_true=30/40/70 发散)。均值只从模型容差内 (|in| < i_gate/2)
    的创新学习: 急停后创新持续 ~v·dt 达数百 ms, 放任均值吸收会使 hp 中途
    缩水 → w 回落 → FF 带残 ghost 重介 → 切断 … FF 权限慢极限环。
  w_inst = min(1, (|hp[k]+hp[k-1]| / i_gate)²): 两帧同号累加, 机动爆发
    两帧内 w→1; 饱和边界/噪声的零均值创新振荡相消 (单帧版会在 accel 饱和
    段把 FF 永久误撤)。
  饱和守卫: 意图命令 (PI+全 FF) 已顶到速度帽时撤回不生效 — 在逃不是在停,
    cap 边界的钳位使撤回只造成极限环。
  gap_scale = 1 − clamp((age − frame_dt)/L, 0, 1): 检测中断时 FF 是纯
    开环航位推算, 按标定 L 的时间尺度撤回, 盲推上界 v̂·STALE(200ms) → ~v̂·L。

设计点 (由失配带电池选定, 非手感): pm_deg=50 — 全延迟带 L_true∈[20,80]
通过的最快设计点 (wn +26%, Ki +59%); beta0=0.03 — 带边缘余量 (β0=0.04
时 L80 的估计器污染使 step 压不回 3px; 0.03 恢复全带)。

与噪声的交互: |in| 冻结门限 i_gate/2=4px 高于 arena 默认噪声 0.5px 与
常见设备噪声 ~1.5px; 噪声 >2px 时稳态 |in| 偶发越过门限 → FF 轻度误撤
(const_vel trail +9% @2px), 但 accel/maneuver 反而更好 (撤回同时减少
FF 噪声注入)。设备噪声大时优先降 beta0 (AGENTS 调参表)。

电池结果 (vs ff_pi): matched composite 171.3→152.9 (全场景更好: step
settle 377→277ms, accel 11.3→9.1px); 宽延迟带 L20-80 全过 (L80 152.4≈
ff_pi 152.5); s 失配 0.7-1.3 全过; relock 483→356ms; FPS 行为电池
RMSE 均值 −9%, 急停过冲 47.4→43.3px, EVENT_OVER 均值/最坏 双降,
approach 拖尾 −25%。60/120fps 的 composite 差 15% 是 settle 刀口指标
噪声 (60fps 逐场景 4/5 优于 ff_pi, 见 AGENTS arena 节)。
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
from arena.core import Observation, LawConfig
from arena.laws.base import register
from arena.laws.ff_pi import FFPILaw


@register("ff_pi2")
class FFPI2Law(FFPILaw):
    """设计点 (由失配带电池选定, 非手感):
       pm_deg=50  — 全延迟带 L_true∈[20,80] 通过的最快设计点 (wn +26%, Ki +59%);
                    PM60 的保守点 matched=171 只用到了带内很少余量。
       beta0=0.03 — 带-边缘余量: β0=0.04 时 L80 的估计器污染使 step 压不回
                    3px; 0.03 恢复全带 (L80 composite 151.4, 优于 ff_pi 152.5)。
    """

    def __init__(self, **kw):
        kw.setdefault("pm_deg", 50.0)
        kw.setdefault("beta0", 0.03)
        super().__init__(**kw)

    def reset(self, cfg: LawConfig):
        super().reset(cfg)
        self.in_mx = self.in_my = 0.0   # 创新均值 (偏置估计)
        self.hp_px = self.hp_py = 0.0   # 上一帧 hp (两帧同号累加)
        self._wdraw = 0.0               # 最近一次 FF 撤回权重 (mouse 侧消费)

    def _update_filter(self, det: Observation):
        cfg = self.cfg
        if self.prev_det_t is None:
            self.fx, self.fy = det.dx, det.dy
            self.fvx = self.fvy = 0.0
            self.in_mx = self.in_my = 0.0
            self.hp_px = self.hp_py = 0.0
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
            self.in_mx = self.in_my = 0.0
            self.hp_px = self.hp_py = 0.0
            self._wdraw = 0.0
        else:
            r = dt / self.DT0
            alpha = min(0.90, self.alpha0 * r)
            beta_s = min(0.60, self.beta0 * r ** self.beta_exp)
            # 高通创新: 去掉以慢通道速率跟踪的均值 (偏置), 留下机动爆发。
            # 均值只从模型容差内 (|in| < i_gate/2) 的创新学习: 急停后创新
            # 持续 ~v·dt 达数百 ms, 若放任均值吸收, hp 会中途缩水 → w 回落
            # → FF 带残 ghost 重介 → 切断 … FF 权限慢极限环; 冻结后 w 稳定。
            if abs(inx) < self.i_gate * 0.5 and abs(iny) < self.i_gate * 0.5:
                self.in_mx += beta_s * inx
                self.in_my += beta_s * iny
            hp_x = inx - self.in_mx
            hp_y = iny - self.in_my
            # 两帧同号累加: 机动爆发超门限, 零均值振荡 (饱和/噪声) 相消
            w = min(1.0, (math.hypot(hp_x + self.hp_px,
                                     hp_y + self.hp_py) / self.i_gate) ** 2)
            self._wdraw = w
            self.hp_px, self.hp_py = hp_x, hp_y
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

            # 饱和守卫: 意图命令已顶到速度帽时 (在逃不是在停) 撤回不生效,
            # 否则 cap 边界钳位使撤回只造成极限环
            sat = (abs(vx_u + self.ff_gain * gate * self.fvx) >= self.max_v
                   or abs(vy_u + self.ff_gain * gate * self.fvy) >= self.max_v)
            w_eff = 0.0 if sat else self._wdraw

            # FF = 门控 × (1 − 机动撤回) × (1 − 丢帧撤回)
            frame_dt = cfg.frame_dt if cfg.frame_dt > 0 else self.DT0
            gap_scale = 1.0 - max(0.0, min(1.0, (age - frame_dt) / max(1.0, cfg.L)))
            ff_eff = self.ff_gain * gate * (1.0 - w_eff) * gap_scale
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
