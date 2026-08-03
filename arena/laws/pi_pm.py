"""arena/laws/pi_pm.py — PI, 显式极点配置 + 相位裕度 (PM) 设计。

## 算法原理

控制结构: 状态预测器 (Smith 式延迟补偿) + 极点配置 PI + 抗饱和 + 余数量化。

植物: 发速度 v (px/ms), 准星每拍 += s×counts ≈ v·h → P(s)=1/s (单位增益积分器)。
测量误差延迟 L: 帧只反映 t−L 的世界。

本律的延迟补偿 (Smith 预测):
  1. alpha-beta 滤波器估计测量时刻 (t−L̂) 的误差 f 与目标速度 fvx
     (预测步扣除上一帧间隔内自身 counts 的作用, 使 fvx 只跟目标运动);
  2. 每控制拍把误差前推到当前时刻:
        e_pred = f + fvx·(age + L̂·l_comp) − s·Σcounts(自 t_pub − L̂·l_comp)
     即用相信的延迟 L̂ 外推目标运动, 并用指令历史精确扣除同期在途准星运动;
  3. 对 e_pred (去延迟误差) 做 PI。补偿精确时闭环特征方程不含延迟项。

极点配置: PI C(s)=Kp+Ki/s 配 P(s)=1/s → 去延迟闭环恰为二阶 s²+Kp·s+Ki=0:
     ωn = √Ki,  ζ = Kp/(2ωn)   ⇔   Kp = 2ζωn,  Ki = ωn²
  - ζ=1.0 临界阻尼 (本律固定): 无过冲, 抗失配最稳;
  - 匀速目标零稳态误差 (type-2); 加速目标稳态滞后 a/Ki。

带宽从延迟导出 (核心, 无手调魔法数): 残留延迟 (标定误差 + Smith 不完美 +
  滤波滞后) 吃相位裕度。把整段相信延迟 L̂ 当作保守的残留延迟上界, 令
     wn·L̂ = (90° − PM)·π/180   ⇔   wn = (90° − PM)·π/180 / L̂
  PM=60° 是无量纲设计选择 (留 60° 相位裕度吸收上述一切非理想), 得
  wn = 0.5236/L̂ (L̂=50 → wn=0.01047 rad/ms)。wn 随标定 L̂ 自动缩放:
  延迟大的设备自动更保守, 延迟小的设备自动更快, 无需逐设备试凑。
  这是用一点速度换可推广性: 比试凑 wn=0.018 慢, 但失配下不发散、跨设备通用。
  reset() 每局按 cfg.L 现算 wn, 不存硬编码常数。

失配下的两个极限环机制 (设计已避开):
  a. 自身指令污染 fvx: L̂≠L_true 时滤波器 counts 扣除窗口错位, 创新量混入
     自身指令 → β 把它积进 fvx → 自激。beta0 小 (0.04) 抑制。
  b. 积分 relay: 残留延迟引起误差振荡时, 积分在 ±i_lim 间摆荡主导指令 →
     饱和极限环。i_gate 远距衰减 + PM=60 的保守带宽抑制。

## 参数表 (默认 / 来源 / 意义)

pm_deg = 60.0   [无量纲设计选择]
    相位裕度。直接决定导出带宽 wn=(90−pm_deg)·π/180/L̂。60° 留足裕度吸收
    标定误差/Smith 残差/滤波滞后。↑ 更鲁棒更慢; ↓ 更快但裕度小。一般不改。

zeta = 1.0   [无量纲设计选择]
    阻尼比, 临界阻尼。固定 1.0: 无过冲、抗失配最稳。

l_comp = 1.1   [EMPIRICAL: 方向有原理, 数值试出]
    Smith 补偿系数 (外推/扣除时域 = L̂×l_comp)。>1 过补偿偏向安全侧:
    低估 L (L_true>L̂) 是危险方向 (残留正延迟→相位滞后→失稳), 过补偿给它
    相位提前; 高估只拖慢。1.0=精确补偿(纯原理值), 1.1 是 arena 下兼顾两侧
    失配的稳健偏置; L̂ 系统性偏低可 ↑(≤1.2), 偏高 → 1.0。

i_gate = 8.0 px   [EMPIRICAL: 划分拉枪/跟踪职责]
    I 距离衰减 ig=gate/(gate+|e|): 拉枪远距几乎不积分 (防过冲), 到位后才
    buildup (移动目标零拖尾)。↑ 跟踪起速更快; ↓ 拉枪更干净。

i_frac = 1.0   [原理: 可跟踪最大目标速度 = i_frac×max_v]
    积分限幅 = i_frac×max_v/Ki。1.0 使积分能命令到 max_v, 即无位置滞后
    可跟踪任意 ≤max_v 的匀速目标。<1 压制失配 I-relay 但快目标出现拖尾。

alpha0 = 0.50   [EMPIRICAL: alpha-beta 估计器增益]
    位置修正增益 @DT0 (120fps)。实际 α=alpha0×dt/DT0 (k1=α/dt 恒定 → 帧率
    无关), 封顶 0.9。↑ 滤波滞后小但噪声 ↑; ↓ 更平滑。

beta0 = 0.04   [EMPIRICAL: alpha-beta 估计器增益]
    速度修正增益 @DT0。↑ 机动适配快但放大机制 (a) 自激; ↓ 更鲁棒。
    保持 <0.05 (arena 失配自激悬崖)。

beta_exp = 1.0   [设计: 每机动事件 fvx 适配量恒定 → 帧率无关]
    β 按 dt^beta_exp 缩放。1 = 60/120fps 机动跟踪一致。

max_v = 1.5 px/ms   [由 cfg.max_v 注入]
    速度上限, 同时决定积分限幅。

## 实机调参

1. 腰射标定 s 与 L̂。wn 由 L̂ 自动导出, 无需手设带宽。确认 L̂ 无系统偏低
   (危险方向); 有偏就把 l_comp 往 1.1–1.2。
2. 静止目标拉枪过冲 → ↑l_comp 或 ↓i_gate; 太慢 → ↓pm_deg (放带宽, 慎)。
3. 移动目标稳态拖尾 → ↑i_gate; 拉枪过冲变大 → ↓i_gate。
4. 鲁棒性自检: 故意把 L̂ 误设 ±10ms 跑拉枪, 持续振荡 → ↑pm_deg / ↓beta0 /
   或按偏差方向调 l_comp。
5. 映射到 aimbot.cu 的 τ 参数: τ=1/(2ζωn)=L̂/((90−PM)π/180)≈95ms@L̂=50,
   τ_i=1/(Ki·τ)。只有带 Smith 预测器才成立, 别搬进无预测器的流水线。

设计取舍: 用 PM=60 从 L̂ 导出 wn, 牺牲拉枪速度 (比试凑 wn=0.018 慢约 1.7×)
换跨设备/失配下的可推广性与不发散。这是本轮明确的优先级: 泛化 > 极速。
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
from arena.core import Observation, LawConfig
from arena.laws.base import Law, register


class _CountsHist:
    __slots__ = ("t", "cx", "cy", "cumx", "camy")

    def __init__(self):
        self.t: list[float] = []
        self.cx: list[float] = []
        self.cy: list[float] = []
        self.cumx = 0.0
        self.camy = 0.0

    def add(self, t, dx, dy):
        self.cumx += dx
        self.camy += dy
        self.t.append(t)
        self.cx.append(self.cumx)
        self.cy.append(self.camy)
        if len(self.t) > 2000:
            self.t.pop(0); self.cx.pop(0); self.cy.pop(0)

    def at(self, t):
        ts = self.t
        n = len(ts)
        if n == 0:
            return 0.0, 0.0
        if t <= ts[0]:
            return self.cx[0], self.cy[0]
        if t >= ts[-1]:
            return self.cx[-1], self.cy[-1]
        lo, hi = 0, n - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ts[mid] <= t:
                lo = mid
            else:
                hi = mid
        span = ts[hi] - ts[lo]
        f = (t - ts[lo]) / span if span > 0 else 0.0
        return (self.cx[lo] + (self.cx[hi] - self.cx[lo]) * f,
                self.cy[lo] + (self.cy[hi] - self.cy[lo]) * f)

    def cum(self):
        return self.cumx, self.camy


@register("pi_pm")
class PiPmLaw(Law):
    DT0 = 1000.0 / 120.0
    JUMP_GATE = 100.0
    STALE = 200.0

    def __init__(self, zeta=1.0, pm_deg=60.0, max_v=1.5, i_gate=8.0, i_frac=1.0,
                 alpha0=0.50, beta0=0.04, l_comp=1.1, beta_exp=1.0):
        self.zeta = zeta            # 阻尼比 (无量纲设计选择, 临界阻尼=1)
        self.pm_deg = pm_deg        # 相位裕度 (无量纲设计选择), 决定导出带宽
        self._max_v = max_v
        self.i_gate = i_gate        # I 距离衰减 (EMPIRICAL: 划分拉枪/跟踪)
        self.i_frac = i_frac        # 积分限幅 = i_frac×max_v/Ki (可跟踪最大速度)
        self.alpha0 = alpha0        # @DT0 位置修正; 按 dt 缩放 (k1=α/dt 恒定)
        self.beta0 = beta0          # @DT0 速度修正; 按 dt^beta_exp 缩放
        self.l_comp = l_comp        # Smith 补偿系数 (>1 偏安全侧, 帮欠补偿)
        self.beta_exp = beta_exp    # β 的 dt 缩放指数 (1=每机动事件等量适配)

    def reset(self, cfg: LawConfig):
        self.cfg = cfg
        self.max_v = self._max_v if self._max_v > 0 else cfg.max_v
        L = max(1.0, cfg.L)
        wn = (90.0 - self.pm_deg) * math.pi / 180.0 / L
        self.kp = 2.0 * self.zeta * wn
        self.ki = wn * wn
        self.ch = _CountsHist()
        self.filt = False
        self.fx = self.fy = self.fvx = self.fvy = 0.0
        self.prev_det_t = None
        self.t_pub = -1e9
        self.int_x = self.int_y = 0.0
        self.rem_x = self.rem_y = 0.0

    def _update_filter(self, det: Observation):
        cfg = self.cfg
        if self.prev_det_t is None:
            self.fx, self.fy = det.dx, det.dy
            self.fvx = self.fvy = 0.0
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
        else:
            r = dt / self.DT0
            alpha = min(0.90, self.alpha0 * r)
            beta = min(0.60, self.beta0 * r ** self.beta_exp)
            self.fx = px_pred + alpha * inx
            self.fy = py_pred + alpha * iny
            self.fvx += (beta / dt) * inx
            self.fvy += (beta / dt) * iny
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

            i_lim = self.i_frac * self.max_v / max(self.ki, 1e-9)
            vx_u = self.kp * ex + self.ki * self.int_x
            vy_u = self.kp * ey + self.ki * self.int_y

            if ex * ex + ey * ey > cfg.fov_radius * cfg.fov_radius:
                self.int_x = self.int_y = 0.0
            else:
                if self.i_gate > 0.0:
                    ig = self.i_gate / (self.i_gate + math.hypot(ex, ey))
                else:
                    ig = 1.0
                wx = (vx_u > self.max_v and ex > 0) or (vx_u < -self.max_v and ex < 0)
                wy = (vy_u > self.max_v and ey > 0) or (vy_u < -self.max_v and ey < 0)
                if not wx:
                    self.int_x = max(-i_lim, min(i_lim, self.int_x + ex * cfg.h * ig))
                if not wy:
                    self.int_y = max(-i_lim, min(i_lim, self.int_y + ey * cfg.h * ig))

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
