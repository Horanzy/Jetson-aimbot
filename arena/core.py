"""arena/core.py — 中立模拟器 (植物 + 传感器)。

arena 只做三件事:
  1. 输出观测: 按帧率发布"最新一帧检测" = 时间戳 + 目标相对准星位置 (目标-准星,
     反映 帧时刻-L_真 那一刻的世界) + 可选噪声。无新帧时给 new=False。
  2. 接受输入: 每控制拍接收 law 发来的鼠标 counts (整数), 据此推进准星。
  3. 给出结果: 用内部真值记录误差序列, 交给 metrics 计算。

arena 不含任何估计/预测/控制逻辑。law 是黑盒, 通过最小接口交互。
单位: 时间 ms, 位置 px, 速度 px/ms, 灵敏度 px/count。
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional, Callable
import math


@dataclass
class Observation:
    t: float          # 帧"可用"时间戳 (ms) = 采集时刻 + L_真
    dx: float         # 目标-准星 x (px), 反映 (t - L_真) 那一刻
    dy: float
    new: bool         # 自上一控制拍以来是否为新帧


@dataclass
class LawConfig:
    s: float          # 灵敏度 px/count (law 相信的值)
    L: float          # 环路延迟 ms (law 相信的值)
    h: float          # 控制拍周期 ms (500Hz -> 2)
    frame_dt: float   # 标称帧周期 ms (120fps -> 8.333)
    max_v: float      # 速度上限 px/ms
    count_limit: int  # 单拍 counts 限幅
    fov_radius: float # FOV 半径 px (仅供参考)


@dataclass
class ArenaConfig:
    s_true: float = 1.0        # 真灵敏度 px/count
    L_true: float = 50.0       # 真延迟 ms
    h: float = 2.0             # 控制拍 ms
    fps: int = 120             # 帧率
    noise_std: float = 0.0     # 检测噪声 std (px, 每轴)
    duration: float = 3000.0   # 场景时长 ms
    count_limit: int = 120     # 植物侧 counts 限幅 (忠实复现硬件)
    fov_radius: float = 150.0


class _StateHistory:
    """记录 (t, tx, ty, cx, cy) 真值, 支持过去时刻线性插值 (供传感器取 t-L 的世界)。"""
    __slots__ = ("t", "tx", "ty", "cx", "cy")

    def __init__(self):
        self.t: list[float] = []
        self.tx: list[float] = []
        self.ty: list[float] = []
        self.cx: list[float] = []
        self.cy: list[float] = []

    def push(self, t, tx, ty, cx, cy):
        self.t.append(t); self.tx.append(tx); self.ty.append(ty)
        self.cx.append(cx); self.cy.append(cy)

    def at(self, t: float):
        ts = self.t
        n = len(ts)
        if n == 0:
            return None
        if t <= ts[0]:
            return self.tx[0], self.ty[0], self.cx[0], self.cy[0]
        if t >= ts[-1]:
            return self.tx[-1], self.ty[-1], self.cx[-1], self.cy[-1]
        lo, hi = 0, n - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ts[mid] <= t:
                lo = mid
            else:
                hi = mid
        span = ts[hi] - ts[lo]
        f = (t - ts[lo]) / span if span > 0 else 0.0
        return (self.tx[lo] + (self.tx[hi] - self.tx[lo]) * f,
                self.ty[lo] + (self.ty[hi] - self.ty[lo]) * f,
                self.cx[lo] + (self.cx[hi] - self.cx[lo]) * f,
                self.cy[lo] + (self.cy[hi] - self.cy[lo]) * f)


class Arena:
    """中立模拟器。target: 一个有 .x .y 且 .advance(h, t) 的对象 (见 scenarios.py)。"""

    def __init__(self, cfg: ArenaConfig, target, rng, cross0=(0.0, 0.0)):
        self.cfg = cfg
        self.frame_dt = 1000.0 / cfg.fps
        self.target = target
        self.rng = rng
        self.cross_x, self.cross_y = cross0
        self.hist = _StateHistory()
        self.rec_t: list[float] = []
        self.rec_ex: list[float] = []
        self.rec_ey: list[float] = []
        self.rec_sent_cx: list[float] = []
        self.rec_sent_cy: list[float] = []
        self._next_cap = 0.0          # 下一帧采集时刻
        self._pending: list[Observation] = []
        self._last_det: Optional[Observation] = None
        self._t = 0.0
        self.diverged = False
        self.hist.push(0.0, target.x, target.y, self.cross_x, self.cross_y)

    def law_config(self, s_belief: float, L_belief: float) -> LawConfig:
        return LawConfig(s=s_belief, L=L_belief, h=self.cfg.h,
                         frame_dt=self.frame_dt, max_v=0.0,
                         count_limit=self.cfg.count_limit,
                         fov_radius=self.cfg.fov_radius)

    def _make_frames_up_to(self, t: float):
        """交付所有"已可用"的帧: 采集时刻 cap 满足 cap+L_真 <= t。
        尚未可用的采集 (cap 在 (t-L, t]) 留待后续, 不提前消耗。"""
        cfg = self.cfg
        while self._next_cap <= t - cfg.L_true:
            cap = self._next_cap
            avail = cap + cfg.L_true
            st = self.hist.at(cap)   # 反映采集时刻的世界
            if st is not None:
                tx, ty, cx, cy = st
                nx = self.rng.gauss(0.0, cfg.noise_std) if cfg.noise_std > 0 else 0.0
                ny = self.rng.gauss(0.0, cfg.noise_std) if cfg.noise_std > 0 else 0.0
                self._pending.append(
                    Observation(t=avail, dx=(tx - cx) + nx, dy=(ty - cy) + ny, new=True))
            self._next_cap += self.frame_dt

    def run(self, law, s_belief: float, L_belief: float, max_v: float):
        """主循环。law.step(t, obs|None) -> (cx, cy) 整数 counts。"""
        cfg = self.cfg
        h = cfg.h
        n_ticks = int(round(cfg.duration / h))
        lcfg = self.law_config(s_belief, L_belief)
        lcfg.max_v = max_v
        law.reset(lcfg)
        div_thr = max(cfg.fov_radius * 6.0, 800.0)

        for k in range(n_ticks):
            t = k * h
            self._t = t
            self._make_frames_up_to(t)

            obs = None
            if self._pending:
                newest = self._pending[-1]
                self._pending.clear()
                self._last_det = newest
                obs = newest
            elif self._last_det is not None:
                obs = Observation(self._last_det.t, self._last_det.dx,
                                  self._last_det.dy, False)

            cx, cy = law.step(t, obs)
            if not (math.isfinite(cx) and math.isfinite(cy)):
                self.diverged = True
                cx = cy = 0
            cx = int(max(-cfg.count_limit, min(cfg.count_limit, cx)))
            cy = int(max(-cfg.count_limit, min(cfg.count_limit, cy)))

            self.cross_x += cfg.s_true * cx
            self.cross_y += cfg.s_true * cy
            self.rec_sent_cx.append(cx)
            self.rec_sent_cy.append(cy)

            self.target.advance(h, t)
            ex = self.target.x - self.cross_x
            ey = self.target.y - self.cross_y
            self.rec_t.append(t)
            self.rec_ex.append(ex)
            self.rec_ey.append(ey)
            self.hist.push(t + h, self.target.x, self.target.y,
                           self.cross_x, self.cross_y)
            if ex * ex + ey * ey > div_thr * div_thr:
                self.diverged = True
                break

        return self.result()

    def result(self):
        return {
            "t": self.rec_t, "ex": self.rec_ex, "ey": self.rec_ey,
            "diverged": self.diverged,
            "sent_cx": self.rec_sent_cx, "sent_cy": self.rec_sent_cy,
        }
