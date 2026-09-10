"""arena/laws/kalman_pi.py — Kalman 状态预测器 + 相位裕度 PI。

原理
====
两段式时间尺度分离, 每个参数都由物理量或声明的无量纲设计选择导出, 无试凑增益。

1) 估计器 — 常速 (CV) Kalman 滤波, 逐轴。
   状态 x = [e, v]ᵀ: e = 目标−准星误差 (px), v = 相对速度 (px/ms),
   自身控制作用已扣除 (Smith 式, 经 B·u 项)。
       F = [[1,dt],[0,1]],  B = [[-dt],[0]],  H = [[1,0]]
       Q = q·[[dt³/3, dt²/2],[dt²/2, dt]]   (白加速度过程噪声)
       R = noise_std²                        (测量方差)
   滤波器是纯估计器: 只输出帧所反映时刻 (t_pub − L̂) 的位置+速度,
   内部不含任何控制逻辑。

   过程噪声 q 由分离原理定。连续 CV Kalman 滤波器解为二阶系统: 阻尼
   ζ_f = 1/√2, 自然频率 ω_f = (q/R)^(1/4) (由代数 Riccati 方程:
   P12 = √(qR), P11 = √2·R^(3/4)·q^(1/4) → 滤波器特征多项式 s²+K1·s+K2,
   K2 = √(q/R) ⇒ ω_f = √K2 = (q/R)^(1/4))。观测器必须快于它馈送的控制器:
   取 ω_f = c·wn, c 为分离因子 (无量纲设计选择, 惯例 c≈3–5)。故
       q = R·(c·wn)⁴。
   因 wn ∝ 1/L (见下), q 自动缩放: 延迟越大滤波越平滑。

   测量噪声 R = noise_std²。noise_std 是检测抖动 (arena 默认 0.5 px/轴;
   实机在静止场景实测)。

2) 延迟补偿 — 线性 Smith 外推 (控制率 500Hz)。
   滤波状态锚在 (t_pub − L̂); 以时域 T = age + L̂ (age = t − t_pub)
   外推到"现在":
       e_pred = e_filt + v_filt·T − s·(t_pub − L̂ 以来的在途 counts)
   纯线性外推 (无临时时域阻尼): 环路带宽足够低 (按完整延迟留 PM),
   残留失配延迟落在相位裕度内, 稳定性无需有界误差外推。

3) 控制器 — 对 e_pred 做极点配置 PI。
   被控对象 ≈ 单位积分器 (准星 += s·counts ≈ v·h)。PI C(s)=Kp+Ki/s 配
   1/s 给出闭环 s²+Kp·s+Ki = 0 ⇒ ωn = √Ki, ζ = Kp/(2ωn)。取
       ζ = 1            (临界阻尼 — 无量纲设计选择)
       ωn = (90°−PM)·π/180 / L̂,  PM = 60°   (无量纲设计选择)
   ωn 是延迟 L̂ 完全不补偿时 (ωn·L̂ = 30°) 仍留 PM=60° 相位裕度的带宽。
   Smith 预测器工作时环路快; 失配下预测器失效则退回这个保底 60° 裕度
   → 不发散。在 reset() 里由 cfg.L 现算 (随 ~1/L 自动缩放; L=50 时
   ≈0.01047 rad/ms), 非硬编码调参常数。
       Kp = 2ζωn,  Ki = ωn²。
   抗饱和: 条件积分 + 积分限幅 ±max_v/Ki (积分最多命令到 max_v →
   最高速目标零位置滞后)。距离门控 ig = i_gate/(i_gate+|e|): 拉枪期
   积分关闭 (|e|≫i_gate → 无到位过冲), 跟踪期开启 (|e|≲i_gate →
   零稳态拖尾)。

参数表 (每项都有依据)
=====================
  pm_deg = 60      无量纲设计选择。延迟完全不补偿时环路仍保留的相位裕度,
                   定 ωn。
  zeta = 1         无量纲设计选择。临界阻尼 (线性极点无过冲; 失配下稳)。
  sep (c) = 5      无量纲设计选择。观测器/控制器分离因子: 滤波带宽 = c·wn。
                   ≈5× 是标准经验值 (观测器快到滞后可忽略, 慢到能拒噪声)。
                   ↑ 机动细节多但噪声大; ↓ 更平滑。
  noise_std = 0.5  物理量。检测噪声 std (px/轴), R = noise_std²。
                   arena 默认; 实机在静止抖动中实测。
  i_gate = 8.0     物理距离 (px)。积分激活尺度 ≈ 数倍稳定容差; 划分拉枪
                   (门关) 与跟踪 (门开)。弱经验的物理尺度 — 已标注。
  l_comp = 1.1     Smith 补偿系数 (外推/扣除时域 = L̂·l_comp)。>1 过补偿,
                   守住危险的欠补偿侧 (L_true > L̂): 标定 L 是下界 (双侧键
                   流程测的是可观测最小延迟), 故工作点放在 L̂ 略过处
                   (10% 裕度), 更远的残留由 PM=60 地板吸收。
  max_v            由 cfg 注入 (速度饱和)。

  reset() 里推导: wn, Kp, Ki, q, R。JUMP_GATE=100 / STALE=200 是跳变/
  超时约定 (同 reference), 非调参控制增益。

已知局限
========
  • CV 模型无法预测加速度; 匀加速目标留 ≈ a/Ki 的滞后, 由积分缓慢消除
    (accel 场景的残余)。
  • ωn = 0.5236/L̂ 刻意保守 (按完整延迟 PM=60) → 拉枪慢于激进律;
    换失配鲁棒性/可推广性。
  • 延迟失配边界: belief=50 时 L_true ≈ 20–70 稳定 (不发散); L_true=80
    (+60% 延迟误差) 时 Smith 扣除窗口错位到污染速度估计, 阶跃无法稳定。
    l_comp=1.1 比 1.0 把边界推远些, 但 PM=60 带宽下盖不住 +60% 误差。
    超出 ±~30ms 残留靠标定精度保证。
  • 真机检测噪声 ≫ noise_std 时滤波器过信测量; noise_std 应由真实静止
    场景实测设定。
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
import numpy as np
from arena.core import Observation, LawConfig
from arena.laws.base import CountsHist, Law, register


@register("kalman_pi")
class KalmanPILaw(Law):
    JUMP_GATE = 100.0
    STALE = 200.0
    V0_STD = 1.0          # 初始速度先验 std (px/ms); 仅影响暂态

    def __init__(self, pm_deg=60.0, zeta=1.0, sep=5.0, noise_std=0.5,
                 i_gate=8.0, l_comp=1.1, max_v=1.5):
        self.pm_deg = pm_deg
        self.zeta = zeta
        self.sep = sep
        self.noise_std = noise_std
        self.i_gate = i_gate
        self.l_comp = l_comp
        self._max_v = max_v

    def reset(self, cfg: LawConfig):
        self.cfg = cfg
        self.max_v = self._max_v if self._max_v > 0 else cfg.max_v

        L = max(1.0, cfg.L)
        wn = (90.0 - self.pm_deg) * math.pi / 180.0 / L
        self.kp = 2.0 * self.zeta * wn
        self.ki = wn * wn

        self.R = self.noise_std * self.noise_std
        self.q = self.R * (self.sep * wn) ** 4

        self.H = np.array([[1.0, 0.0]])
        self.ch = CountsHist()
        self.x_x = np.zeros(2)
        self.x_y = np.zeros(2)
        self.P_x = np.array([[self.R, 0.0], [0.0, self.V0_STD ** 2]])
        self.P_y = np.array([[self.R, 0.0], [0.0, self.V0_STD ** 2]])
        self.initialized = False
        self.prev_det_t = None
        self.t_pub = -1e9
        self.int_x = self.int_y = 0.0
        self.rem_x = self.rem_y = 0.0

    def _update_filter(self, det: Observation):
        cfg = self.cfg
        if not self.initialized:
            self.x_x = np.array([det.dx, 0.0])
            self.x_y = np.array([det.dy, 0.0])
            self.P_x = np.array([[self.R, 0.0], [0.0, self.V0_STD ** 2]])
            self.P_y = np.array([[self.R, 0.0], [0.0, self.V0_STD ** 2]])
            self.initialized = True
            self.prev_det_t = det.t
            self.t_pub = det.t
            return

        dt = max(1.0, min(100.0, det.t - self.prev_det_t))
        Lc = cfg.L * self.l_comp
        c0 = self.ch.at(det.t - Lc - dt)
        c1 = self.ch.at(det.t - Lc)
        u_x = cfg.s * (c1[0] - c0[0]) / dt
        u_y = cfg.s * (c1[1] - c0[1]) / dt

        F = np.array([[1.0, dt], [0.0, 1.0]])
        B = np.array([-dt, 0.0])
        Q = self.q * np.array([[dt ** 3 / 3.0, dt ** 2 / 2.0],
                               [dt ** 2 / 2.0, dt]])

        for axis in (0, 1):
            x = self.x_x if axis == 0 else self.x_y
            P = self.P_x if axis == 0 else self.P_y
            u = u_x if axis == 0 else u_y
            xp = F @ x + B * u
            Pp = F @ P @ F.T + Q
            S = (self.H @ Pp @ self.H.T)[0, 0] + self.R
            K = (Pp @ self.H.T).flatten() / S
            z = det.dx if axis == 0 else det.dy
            innov = z - (self.H @ xp)[0]
            xn = xp + K * innov
            Pn = (np.eye(2) - np.outer(K, self.H.flatten())) @ Pp
            if axis == 0:
                self.x_x, self.P_x, inx = xn, Pn, innov
            else:
                self.x_y, self.P_y, iny = xn, Pn, innov

        if math.hypot(inx, iny) > self.JUMP_GATE:
            self.x_x = np.array([det.dx, 0.0])
            self.x_y = np.array([det.dy, 0.0])
            self.P_x = np.array([[self.R, 0.0], [0.0, self.V0_STD ** 2]])
            self.P_y = np.array([[self.R, 0.0], [0.0, self.V0_STD ** 2]])
            self.int_x = self.int_y = 0.0

        self.prev_det_t = det.t
        self.t_pub = det.t

    def step(self, t: float, obs: Optional[Observation]) -> Tuple[int, int]:
        cfg = self.cfg
        if obs is not None and obs.new:
            self._update_filter(obs)

        cx = cy = 0
        age = t - self.t_pub
        if self.initialized and age < self.STALE:
            Lc = cfg.L * self.l_comp
            cp = self.ch.at(self.t_pub - Lc)
            cn = self.ch.cum()
            T = age + Lc
            ex = self.x_x[0] + self.x_x[1] * T - cfg.s * (cn[0] - cp[0])
            ey = self.x_y[0] + self.x_y[1] * T - cfg.s * (cn[1] - cp[1])

            vx_u = self.kp * ex + self.ki * self.int_x
            vy_u = self.kp * ey + self.ki * self.int_y

            if ex * ex + ey * ey > cfg.fov_radius * cfg.fov_radius:
                self.int_x = self.int_y = 0.0
            else:
                ig = self.i_gate / (self.i_gate + math.hypot(ex, ey)) \
                    if self.i_gate > 0 else 1.0
                i_lim = self.max_v / max(self.ki, 1e-9)
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
