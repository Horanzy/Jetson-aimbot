"""arena/laws/mpc.py — 模型预测控制 (MPC, 滚动时域最优控制)。

================================================================================
原理 (principle-first; 每个参数都从物理量推出, 无手调魔数)
================================================================================
把"瞄准"建模成有限时域最优控制, 每个控制拍 (h=2ms) 求解一次, 只施加第一步最优指令 u_0
(receding horizon / 滚动时域)。下一拍用新的状态估计重新求解 —— 这种"反复重解"天然带反馈
修正: 延迟失配/模型误差造成的偏差每拍都被重新规划吸收, 因此失配下表现为**有界偏差
(graceful)** 而非发散。

状态 x=[e, v]ᵀ: e=目标−准星 (px), v=目标速度/误差变化率 (px/ms)。控制 u=准星速度指令
(px/ms)。植物是纯积分器: 发速度 u → 准星每拍移动 u·h px (counts=u·h/s), 准星速度瞬时
等于 u, 无动量。误差动力学 (匀速目标模型):

    e_{i+1} = e_i + h·v_i − h·u_i,      v_{i+1} = v_i

每拍求解箱式约束 QP (x/y 逐轴解耦, 各一次):

    minimize   J = Σ_{i=1..N} [ q·e_i² + r·(u_i − v0)² ] + qf·e_N²
    subject to e_{i+1}=e_i+h·v_i−h·u_i,  v_{i+1}=v_i,  |u_i| ≤ Vmax (=max_v)

--------------------------------------------------------------------------------
[1] 阶段代价权重 q/r —— 由期望闭环带宽 (相位裕度) 推出, LQR 式选权
--------------------------------------------------------------------------------
植物 (去延迟后) 是积分器 e_dot = −u。连续 LQR 代价 ∫(q·e² + r·u²)dt 对标量积分器给出
闭环极点 −√(q/r), 即**闭环带宽 wn = √(q/r)**。带宽不能任意高: 环路延迟 L 在频率 wn 处
贡献相位滞后 wn·L (rad); 积分器本身有 −90° 相移, 故相位裕度

    PM = 90° − wn·L·(180/π)   ⟹   wn = (90°−PM)·(π/180) / L

取**PM=60°** (教科书鲁棒设计点: 60° PM ≈ 增益裕度 ≥2、对建模误差/残留延迟有充足裕度)。
于是唯一设计自由度是 PM, 其余全推出:

    wn = (π/6) / L_hat                      (L_hat=cfg.L×L_inflate, 见 [4])
    q  = 1                                   (归一化: 只有比值 q/r 有物理意义)
    r  = 1/wn² = (6·L_hat/π)²                (LQR 带宽=√(q/r) 反解)

这把手调的 "q=1,r=10000" 换成 "PM=60° → r=(6·L_hat/π)²"。验算: L_hat=55 时
r=(330/π)²≈11036, k=√(1/11036)=0.00952, wn·L_hat=0.524rad=30° → PM=60° 精确成立。
(原 r=10000 对应 PM≈58.5°, 本设计与其几乎同工作点, 但从原理推出而非试出。)

**为什么控制代价是 r·(u_i−v0)² 而不是 r·u_i²**: 若罚 r·u_i², 则"命令 u=v0 跟踪匀速
目标"本身被重罚 → MPC 会抑制速度前馈, 匀速跟踪留大稳态拖尾。改罚"偏离前馈速度的量"
(u_i−v0)² 后: 命令 u=v0 跟踪是**零代价** → 匀速跟踪零拖尾; 而"纠偏"增益仍由 q/r 定
(可独立按 [1] 求鲁棒)。代数上只给 b_v 加一个 +r·1 项 (见 [5]), 几乎零成本, 却把"跟踪"
与"纠偏增益"彻底解耦。v0=vff·fvx 是估计的目标速度前馈。

--------------------------------------------------------------------------------
[2] 预测时域 N —— 由延迟 L 与控制步 h 推出
--------------------------------------------------------------------------------
Smith 预测器已把延迟从 QP 初态里扣掉 (QP 植物去延迟), 但有限时域仍须"看见"主导时间尺度。
闭环的自然时间尺度由延迟设定 (带宽 wn∝1/L_hat, 见 [1]); 终端权重 qf ([3]) 负责折叠无穷
远尾巴, 但它表达不了纯传输延迟。故 N 取:

    N = ceil(L_hat / h)  +  ceil(frame_dt / h)
        └─跨越延迟─┘     └─再看一个传感器帧的延迟后演化─┘

两项都由物理量 (L_hat, h, frame_dt) 推出, 无自由整数。第一项让时域至少覆盖一个延迟
(主导非最小相位/传输尺度); 第二项让时域越过延迟边界**至少一个完整测量周期**, 使预测始终
含一帧新鲜的延迟后演化、终端罚锚定在已收敛的轨迹上而非延迟切点。L_hat=55/h=2/120fps:
ceil(27.5)+ceil(4.17)=28+5=33; 60fps (frame_dt=16.67): 28+9=37 (低帧率测量间隔长 → 看得
更远, 行为仍帧率无关因为增益按 dt 归一)。

--------------------------------------------------------------------------------
[3] 终端权重 qf —— 无穷时域 LQR (DARE) 折叠, 免调
--------------------------------------------------------------------------------
v (目标速度) 是测得常量、不受 u 控制 (不可控模态 λ=1), 2 态 DARE 不可解; 真正需终端权重
稳定的是 e 积分器子系统 e_{i+1}=e_i−h·u_i (A=1,B=−h,Q=q,R=r)。其标量 DARE 解
P=(q+√(q²+4qr/h²))/2 即 qf, 把"无穷远尾巴"折叠进有限时域 → 即使 N 不大也稳定。qf 随
q,r 一起由 [1] 推出, 不是独立旋钮。

--------------------------------------------------------------------------------
[4] 延迟处理 + L_inflate
--------------------------------------------------------------------------------
唯一延迟在观测侧: obs.t 的帧反映 (obs.t−L_真) 的世界。两步:
  1. alpha-beta 滤波估计**测量世界时刻**的误差 (fx,fy) 与误差率 (fvx,fvy); 预测步扣除
     自身指令在该段时间对准星的贡献 (用相信的 L_hat 取指令历史)。增益按 dt 归一 (α∝dt,
     β∝dt^beta_exp) → 60/120fps 行为一致。
  2. Smith 前推 (用 L_hat=cfg.L×L_inflate): e0 = fx + fvx·(age+L_hat) − s·(在途 counts),
     v0 = vff·fvx。[e0,v0] 即 MPC 初态。
失配 (L_真≠L_信) 时 Smith 残留 ~(L_真−L_hat)·v 的偏差; MPC 每拍重解把它当新初态重新规划
→ 稳态有界偏差, 不累积发散。**L_inflate=1.1 过补偿 10%**: 欠补偿 (L_真>L_hat, 残留正延迟,
加相位滞后) 是危险方向, 过补偿加相位超前更稳; 把鲁棒性从欠补偿侧匀给过补偿侧。方向由
原理定 (过补偿), 幅度 10% 是常规鲁棒裕度。注意 L_hat 同时进 [1] 的带宽与 [2] 的时域 ——
更大的相信延迟 → 更低带宽/更长时域 → 更鲁棒, 内部自洽。

--------------------------------------------------------------------------------
[5] QP 的显式构造 (常量 reset 构造一次, 每拍只做向量合成)
--------------------------------------------------------------------------------
决策 u=[u_0..u_{N-1}]∈R^N。由动力学 e_i=e0+i·h·v0−h·Σ_{j<i}u_j。令 M=tril(ones(N))
((Mu)_r=Σ_{j≤r}u_j), 自由响应 d=e0·1+h·v0·k (k=[1..N]), 则 e_vec=d−h·M·u, 末端
e_N=d_N−h·m_Nᵀu (m_N=全1)。代价 J=uᵀHu−2bᵀu+const (控制罚 r·||u−v0·1||²):

    H = q·h²·MᵀM + r·I + qf·h²·m_N m_Nᵀ            (常量, reset 构造 + Cholesky)
    b = e0·b_e + v0·b_v
    b_e = q·h·Mᵀ1 + qf·h·m_N
    b_v = q·h²·Mᵀk + qf·h²·N·m_N + r·1            (末项 +r·1 即 (u−v0)² 罚的线性部分)

H 正定 (r·I)。**快路径**: 无约束解 u_unc=H⁻¹b 若全满足 |u|≤Vmax (跟踪小误差的常见情形)
直接取 u_unc[0]; 否则走 **FISTA 投影梯度** (步长 1/λ_max(H), 上一拍解滚动移位暖启动,
~40 迭代), 只取 u_0。在本文鲁棒调参下增益低 (k≈0.0095), 80px 阶跃的纠偏指令尚未触到
Vmax (饱和阈值≈Vmax/k≈158px>80px), 故标准阶跃表现为"平滑低过冲逼近"; 约束在更大误差
或更高 max_v 下才主导。MPC 相对 PI 的实测优势: 极低过冲、速度前馈精跟踪、滚动重解鲁棒。

================================================================================
参数表 (默认 / 推出依据 / 真机调参方向)
================================================================================
参数        默认      依据                                        真机调参方向
----------  --------  ------------------------------------------  ----------------------------
PM          60.0deg   **唯一设计旋钮** (principle)。相位裕度。     要更快且 L 标定准/噪声小→↓PM
                      wn=(90−PM)π/180/L_hat, r=1/wn², q/r=wn²    (↑带宽, 但↓裕度); 失配振荡/
                      全由此推出。60°=教科书鲁棒点。               噪声大→↑PM。一次只动它。
qf          None      principle: 无约束 LQR 的 DARE 解 (见[3])。  保持 None。手动加大→更稳/更
                      None=自动。随 q,r 推出, 非独立旋钮。         激进但↓裕度。
max_v       1.5       physical: 速率上限 px/ms (=cfg.max_v), QP   按硬件/手感: 想更快→↑; 更稳
                      硬约束 Vmax, 决定最大拉枪速度。              →↓。MPC 自动按它规划, 不过冲。
L_inflate   1.1       principle 方向 + 常规裕度 (见[4])。过补偿    失配总朝欠补偿侧偏 (易振荡)→
                      10% 把鲁棒性匀给危险的欠补偿侧。             ↑(1.1→1.2); 朝过补偿侧偏→↓。
vff         1.0       principle: =1 完全前馈估计目标速度 → 匀速  保持 1.0。↓ 破坏匀速跟踪
                      跟踪零稳态拖尾的唯一值。                     (拖尾∝1/vff); 仅速度估计被
                                                                  噪声严重污染时略微↓。
alpha0      0.30      ESTIMATOR CONVENTION (与 reference.py 一致,  检测噪声大→↓; 滤波滞后大→↑。
                      非 MPC 特有调参)。@120fps alpha-beta 位置    属状态估计器, 不属 MPC 权重。
                      增益; 按 dt 归一 (α=alpha0·dt/DT0)。
beta0       0.08      ESTIMATOR CONVENTION (与 reference.py 一致)。 机动滞后→想↑, 但前馈把噪声/
                      @120fps 速度增益; 按 dt^beta_exp 归一。      失配灌回, ↑受失配发散边界限。
beta_exp    1.0       principle: =1 → β∝dt, 每机动事件等量适配     60fps 机动明显比 120fps 糊→
                      (帧率无关)。                                 试 2。
JUMP_GATE   100.0     physical/convention: 创新超此 px 视为目标    按 FOV/目标切换幅度调。
                      切换, 重置滤波器 (不加补丁式限幅)。
STALE       200.0     convention: 无新帧超此 ms 停止出指令。       按帧率/掉帧容忍度调。

================================================================================
真机调参备注
================================================================================
arena 的具体数字不会直接迁移到真机: 真机的 s、L、噪声、帧率、目标动态、以及发散是否被
重罚都不同。迁移流程:
  1. 先标定 s 与 L (本系统双侧键标定), 填进 cfg。L_inflate 从 1.0 起步。
  2. PM 是唯一主旋钮: 从 60° 起, L 标定准且噪声小可↓PM (如 50°) 换更快拉枪, 直到阶跃/
     机动出现极限环或失配发散, 回退一档。带宽 wn=(90−PM)π/180/L_hat 自动随之。
  3. L_inflate 微调失配对称性: 实测 L 偏小 (欠补偿, 易振荡)→↑; 偏大→↓。
  4. max_v 按手腕/鼠标极限设; 硬约束, MPC 自动按它规划。
  5. alpha0/beta0 按噪声调 (估计器): 噪声大↓, 滞后大↑; beta0 受失配边界限制。
  6. N、qf 免调 (N 由 L_hat/h/frame_dt 推出, qf 自动 DARE)。vff 保持 1.0。
  7. 60/120fps 无需重调: 增益按 dt 归一, QP 用真实 h, N 随 frame_dt 自适应。
已知局限: (a) 匀速模型, 对匀加速/高频机动有原理性拖尾 (accel in_band 偏低); 加速度前馈
试过但加速度由位置二阶微分、噪声极敏感, 改善 accel 同时恶化 const_vel 并破坏鲁棒性, 弃。
(b) 鲁棒调参 (PM=60) 压低增益, 标准 80px 阶跃不触发 Vmax 饱和 (阈值≈158px), "约束最优
拉枪"在更大误差才显现。(c) 逐轴解耦, 无矢量幅值约束。(d) 每拍解 QP, 算力远高于 PI:
N≈33 (120fps) 时 H 为 33×33, 小误差走 Cholesky 快路径 O(N²), 仅大误差 (|e|≳Vmax/k≈158px)
走 FISTA 40 迭代。本机实测 eval 电池 ~3.6s、integrate 全程 ~7.5s; 嵌入式 500Hz 需评估,
暖启动 + 快路径已把均摊成本压低。(e) 依赖
s/L 估计, 失配下是有界偏差而非零偏差; |L_真−L_信|≳30ms 极端失配会发散 (超出标定应有精度)。
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
import numpy as np
from scipy.linalg import solve_discrete_are, cho_factor, cho_solve
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


@register("mpc")
class MpcLaw(Law):
    DT0 = 1000.0 / 120.0      # 参考帧周期 (120fps), 滤波器增益 dt 归一基准
    Q_NORM = 1.0              # 误差权重归一化 (只有 q/r 比值有物理意义, 见 docstring [1])
    JUMP_GATE = 100.0
    STALE = 200.0

    def __init__(self, PM=60.0, qf=None, max_v=1.5, L_inflate=1.1, vff=1.0,
                 alpha0=0.30, beta0=0.08, beta_exp=1.0):
        self._PM = PM                # 相位裕度 (deg): 唯一设计旋钮, 定带宽/权重 (docstring [1])
        self._qf = qf                # None=自动 DARE 终端权重 ([3])
        self._max_v = max_v          # 速率硬约束 Vmax (px/ms)
        self._L_inflate = L_inflate  # Smith 过补偿系数 ([4])
        self._vff = vff              # 速度前馈系数 (=1 匀速零拖尾)
        self._alpha0 = alpha0        # 估计器约定 (@DT0 位置增益, 按 dt 归一)
        self._beta0 = beta0          # 估计器约定 (@DT0 速度增益, 按 dt^beta_exp 归一)
        self._beta_exp = beta_exp

    def reset(self, cfg: LawConfig):
        self.cfg = cfg
        self.max_v = self._max_v if self._max_v > 0 else cfg.max_v
        self.L_hat = cfg.L * self._L_inflate
        self.ch = _CountsHist()
        self.filt = False
        self.fx = self.fy = self.fvx = self.fvy = 0.0
        self.prev_det_t = None
        self.t_pub = -1e9
        self.rem_x = self.rem_y = 0.0

        # [1] 带宽由相位裕度推出: wn=(90−PM)π/180/L_hat; LQR 带宽=√(q/r) → r=1/wn²。
        wn = (90.0 - self._PM) * (math.pi / 180.0) / self.L_hat
        q = self.Q_NORM
        r = 1.0 / (wn * wn)
        # [2] 时域由延迟+控制步+帧周期推出: 跨越延迟, 再看一个传感器帧的延迟后演化。
        N = int(math.ceil(self.L_hat / cfg.h) + math.ceil(cfg.frame_dt / cfg.h))
        self._build_qp(cfg.h, q, r, N)

    def _build_qp(self, h: float, q: float, r: float, N: int):
        if self._qf is None:
            # [3] 终端权重 = e 积分器子系统的无穷时域 LQR (DARE) 解。v 不可控 (λ=1),
            # 2 态 DARE 不可解; 标量 DARE 解析解 P=(q+√(q²+4qr/h²))/2, 数值由 dare 给出。
            try:
                P = solve_discrete_are(np.array([[1.0]]), np.array([[-h]]),
                                       np.array([[q]]), np.array([[r]]))
                qf = float(P[0, 0])
            except Exception:
                qf = 0.5 * (q + math.sqrt(q * q + 4.0 * q * r / (h * h)))
        else:
            qf = float(self._qf)
        self.qf = qf
        self.q = q
        self.r = r

        M = np.tril(np.ones((N, N)))
        ones = np.ones(N)
        k = np.arange(1, N + 1, dtype=float)
        mN = ones
        H = q * h * h * (M.T @ M) + r * np.eye(N) + qf * h * h * np.outer(mN, mN)
        self.H = H
        self.chol = cho_factor(H)
        self.b_e = q * h * (M.T @ ones) + qf * h * mN
        # 控制代价 r·(u_i−v0)² (偏离前馈速度才罚): u=v0 跟踪匀速目标零代价 → 零稳态拖尾;
        # 纠偏增益仍由 q/r 定。展开 r·||u−v0·1||² 的线性项给 b_v 贡献 +r·1。
        self.b_v = q * h * h * (M.T @ k) + qf * h * h * N * mN + r * ones
        self.N = N
        # FISTA 步长 = 1/λ_max(H); 暖启动缓存 (上一拍解, 按轴)
        self.lip = float(np.linalg.eigvalsh(H)[-1])
        self.warm_x = np.zeros(N)
        self.warm_y = np.zeros(N)
        self.fista_iters = 40

    def _solve_u0(self, e0: float, v0: float, warm: np.ndarray) -> Tuple[float, np.ndarray]:
        b = e0 * self.b_e + v0 * self.b_v
        u_unc = cho_solve(self.chol, b)
        vm = self.max_v
        if np.all(np.abs(u_unc) <= vm):
            return float(u_unc[0]), u_unc
        # 箱式约束 QP: FISTA 投影梯度, 上一拍解 (滚动移位) 暖启动。只取 u_0, 精度够用。
        H = self.H
        alpha = 1.0 / self.lip
        u = np.clip(warm, -vm, vm)
        y = u.copy()
        tk = 1.0
        for _ in range(self.fista_iters):
            grad = H @ y - b
            un = np.clip(y - alpha * grad, -vm, vm)
            tkn = 0.5 * (1.0 + math.sqrt(1.0 + 4.0 * tk * tk))
            y = un + ((tk - 1.0) / tkn) * (un - u)
            u = un
            tk = tkn
        return float(u[0]), u

    def _update_filter(self, det: Observation):
        cfg = self.cfg
        if self.prev_det_t is None:
            self.fx, self.fy = det.dx, det.dy
            self.fvx = self.fvy = 0.0
            self.filt = True
            self.prev_det_t = det.t
            self.t_pub = det.t
            return
        dt = det.t - self.prev_det_t
        dt = max(1.0, min(100.0, dt))
        c0 = self.ch.at(det.t - self.L_hat - dt)
        c1 = self.ch.at(det.t - self.L_hat)
        cax = c1[0] - c0[0]
        cay = c1[1] - c0[1]
        px_pred = self.fx + self.fvx * dt - cfg.s * cax
        py_pred = self.fy + self.fvy * dt - cfg.s * cay
        inx = det.dx - px_pred
        iny = det.dy - py_pred
        if math.hypot(inx, iny) > self.JUMP_GATE:
            self.fx, self.fy = det.dx, det.dy
            self.fvx = self.fvy = 0.0
        else:
            rr = dt / self.DT0
            alpha = min(0.90, self._alpha0 * rr)
            beta = min(0.60, self._beta0 * rr ** self._beta_exp)
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
            cp = self.ch.at(self.t_pub - self.L_hat)
            cn = self.ch.cum()
            ifx = cfg.s * (cn[0] - cp[0])
            ify = cfg.s * (cn[1] - cp[1])
            T = age + self.L_hat
            e0x = self.fx + self.fvx * T - ifx
            e0y = self.fy + self.fvy * T - ify

            v0x = self._vff * self.fvx
            v0y = self._vff * self.fvy
            wx = np.empty(self.N); wx[:-1] = self.warm_x[1:]; wx[-1] = self.warm_x[-1]
            wy = np.empty(self.N); wy[:-1] = self.warm_y[1:]; wy[-1] = self.warm_y[-1]
            vx, self.warm_x = self._solve_u0(e0x, v0x, wx)
            vy, self.warm_y = self._solve_u0(e0y, v0y, wy)
            vx = max(-self.max_v, min(self.max_v, vx))
            vy = max(-self.max_v, min(self.max_v, vy))

            s = max(0.05, min(20.0, cfg.s))
            self.rem_x += vx * cfg.h / s
            self.rem_y += vy * cfg.h / s
            cx, cy, self.rem_x, self.rem_y = self._counts(
                self.rem_x, self.rem_y, s, cfg.count_limit)
        else:
            self.rem_x = self.rem_y = 0.0

        self.ch.add(t, cx, cy)
        return cx, cy
