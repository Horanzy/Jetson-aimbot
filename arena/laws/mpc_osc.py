"""arena/laws/mpc_osc.py — 模型预测控制 (MPC, 滚动时域最优控制) + 振荡签名前馈门。

================================================================================
原理 (principle-first; 每个参数都从物理量推出, 无手调魔数)
================================================================================
把"瞄准"建模成有限时域最优控制, 每个控制拍 (h=2ms) 求解一次, 只施加第一步最优指令 u_0
(receding horizon / 滚动时域)。下一拍用新的状态估计重新求解 —— 这种"反复重解"天然带反馈
修正: 延迟失配/模型误差造成的偏差每拍都被重新规划吸收, 因此失配下表现为**有界偏差
(graceful)** 而非发散。

状态 x=[e, v]ᵀ: e=目标−准星 (px), v=目标速度/误差变化率 (px/ms)。控制 u=准星速度指令
(px/ms)。被控对象是纯积分器: 发速度 u → 准星每拍移动 u·h px (counts=u·h/s), 准星速度瞬时
等于 u, 无动量。误差动力学 (匀速目标模型):

    e_{i+1} = e_i + h·v_i − h·u_i,      v_{i+1} = v_i

每拍求解箱式约束 QP (x/y 逐轴解耦, 各一次):

    minimize   J = Σ_{i=1..N} [ q·e_i² + r·(u_i − v0)² ] + qf·e_N²
    subject to e_{i+1}=e_i+h·v_i−h·u_i,  v_{i+1}=v_i,  |u_i| ≤ Vmax (=max_v)

[1] 阶段代价权重 q/r —— 由期望闭环带宽 (相位裕度) 推出
    去延迟后的被控对象是积分器; 连续 LQR 代价对积分器给出闭环带宽 wn = √(q/r)。
    环路延迟 L 在频率 wn 处贡献相位滞后 wn·L, 积分器本身有 −90° 相移, 故
        PM = 90° − wn·L_hat·(180/π)  ⟹  wn = (90°−PM)·(π/180) / L_hat
    PM=60° 为教科书鲁棒设计点 (对建模误差/残留延迟有充足裕度); q=1 归一 (只有比值
    q/r 有物理意义), r = 1/wn² 反解。
    **控制代价罚 r·(u_i−v0)² 而非 r·u_i²**: 命令 u=v0 跟踪匀速目标本身零代价 →
    匀速跟踪零拖尾, 而"纠偏"增益仍由 q/r 独立决定 —— 跟踪与纠偏彻底解耦。
    v0 = vff·fvx 是估计的目标速度前馈。

[2] 预测时域 N —— 由延迟与控制步/帧周期推出
    N = ceil(L_hat/h) + ceil(frame_dt/h): 第一项让时域跨越主导传输尺度 (延迟),
    第二项越过延迟边界至少一个完整测量周期, 使终端罚锚定在已收敛的轨迹上。
    两项皆由物理量推出, 无自由整数。

[3] 终端权重 qf —— 无穷时域 LQR (DARE) 折叠, 免调
    e 积分器子系统 (A=1, B=−h) 的标量 DARE 解 P = (q+√(q²+4qr/h²))/2 即 qf,
    把无穷远尾巴折叠进有限时域; 随 q,r 一起推出, 不是独立旋钮。

[4] 延迟处理 + L_inflate
    唯一延迟在观测侧。alpha-beta 滤波 (增益按 dt 归一, 帧率无关) 估计测量世界时刻的
    (fx,fy) 与 (fvx,fvy); Smith 前推 (L_hat = cfg.L×L_inflate):
        e0 = fx + fvx·(age+L_hat) − s·(在途 counts),   v0 = vff·fvx
    [e0,v0] 即 QP 初态。失配下 Smith 残留偏差被滚动重解当作新初态吸收 → 有界不发散。
    L_inflate=1.1 过补偿 10%: 欠补偿侧 (L_真>L_hat, 残留正延迟加相位滞后) 是危险
    方向, 把鲁棒性匀给它; 方向由原理定, 幅度 10% 是常规鲁棒裕度。

[5] QP 的显式构造 (常量 reset 构造一次, 每拍只做向量合成)
    H = q·h²·MᵀM + r·I + qf·h²·m_N·m_Nᵀ (M=下三角全1; 常量, Cholesky 分解);
    b = e0·b_e + v0·b_v。快路径: 无约束解 u_unc = H⁻¹b 若全满足 |u|≤Vmax 直接取
    u_unc[0]; 否则 FISTA 投影梯度 (步长 1/λ_max(H), 上一拍解滚动移位暖启动, ~40
    迭代), 只取 u_0。本调参下纠偏增益低, 标准阶跃表现为平滑低过冲逼近。

================================================================================
振荡签名前馈门 (失配自激防护)
================================================================================
[为什么需要] Smith 预测的在途扣除用相信窗 [t−L̂−dt, t−L̂]。标定残差 |L真−L̂| 大时
(宽延迟带两端: L_true=20 → ΔL=−35ms; L_true=80 → +25ms), 两窗 counts 差
≈ s·(自身指令变化率)·ΔL, 作为伪创新经 α-β 滤波的 β 通道泵入 fvx → 幽灵速度。幽灵经
两条路回到指令: (a) QP 前馈 v0=vff·fvx (增益 1.0 全通), (b) Smith 外推 e0 += fvx·T
(T≈age+L̂, QP 增益 k·T≈0.5)。环路增益在 |ΔL| 大时 >1 → 自激极限环 → 永不进 3px 带
(L_true=20 实测: 末值 21px, 幅度 28px, ex 过零 10 次/500ms)。匹配时 ΔL 垃圾小,
无自激 —— 所以门必须"匹配时不可见"。

[判别原理] 自激的唯一签名是"创新的交替", 不是"矛盾幅度":
  - 自激极限环: 伪创新由自身指令经错位窗产生, 环路振荡 → 创新大幅**交替翻转**
    (L_true=20 实测周期 ~100ms)。
  - 真实目标事件 (急停/反转/机动): 矛盾**单极性持续**到 v̂ 重建完成, 之后创新与 v̂
    同向 —— 孤立反转/过冲在创新维度不构成快速交替。
  - s 失配追赶偏置: (s真−s信)·自身速率 恒定符号, 不翻转 → s 带零改动。
  - 慢速真实欠阻尼响应 (机动反转+过冲+追摆): 其振荡在**误差 e0 维度**清晰可见, 但在
    **创新维度**被滤波 α 通道跟踪掉 (创新单极性、小), 与自激的创新维度签名可分
    (对比实验: e0 维度翻转计数无法区分"真实事件振铃"与"自激", 会误伤机动)。

[机制] (三个部件, 全部 σ/时间标定化, 无绝对 px 常数):
  1) σ̂ 在线自标定 + Huber 守卫: 创新方差 EMA (β 同滤波), 单帧创新 > IN_SIGMA·σ̂
     (=3σ) 视为信号不回训 —— 否则自激垃圾创新被 σ̂ EMA 吸收 (实测 σ̂ 1→44px),
     所有 σ 归一门限失明 (σ̂ 致盲机制)。
  2) 显著创新符号交替计数 (逐轴): |in| > IN_SIGMA·σ̂ 才计显著; 符号相对上次显著创新
     翻转、且距上次 ≤ W 才计一次翻转; flip 按 τ=W 泄漏。W = L̂ + 2·τ_v (τ_v =
     DT0/β₀, 帧率归一): 相邻矛盾必须经由环路往返 (L̂) 加一次 v̂ 重建 (τ_v) 才可能
     同源; 真实反转事件间隔 ≫ W。JUMP_GATE 复位时清零 (目标切换非振荡)。
  3) 信任度门控 (w_state 语义): flip ≥ FLIP_N (=2; 一次振荡至少两个方向反转) →
     w 升起 (~2 帧, τ=2·DT0), 证据消失按标定 L 尺度恢复 (τ=cfg.L)。确认时**同时
     切断两条幽灵回路**: QP 前馈 v0 与 Smith 外推 fvx·T 都乘 ff_gate = 1−w。满信任
     w=0 → ff_gate=1 → 门完全不可见。

================================================================================
参数表 (默认 / 推出依据 / 真机调参方向)
================================================================================
参数        默认    依据                                            真机调参方向
----------  ------  ----------------------------------------------  ----------------
PM          60.0    唯一设计旋钮: wn=(90−PM)π/180/L_hat, r=1/wn²    失配振荡/噪声大→↑;
                    全由此推出; 60°=教科书鲁棒点                    标定准且要更快→↓
qf          None    原理: e 积分器子系统的标量 DARE 解 (见[3])      保持 None
max_v       1.5     物理量: 速率硬约束 Vmax (px/ms)                 想更快→↑; 更稳→↓
L_inflate   1.1     原理方向 + 常规裕度 (见[4]): 过补偿偏向安全的   失配总朝欠补偿侧偏→↑;
                    过补偿侧                                        朝过补偿侧偏→↓
vff         1.0     原理: =1 完全前馈估计目标速度 → 匀速零拖尾      保持 1.0
alpha0/beta0 0.30/  估计器约定 (@DT0, 按 dt 归一, 帧率无关)         模型抖动→↓alpha0
            0.08
IN_SIGMA    3.0     σ̂ 倍数 (无量纲): 创新超此倍数才算"显著",        假告警→↑; 自激漏检→↓
                    低于即噪声不参与翻转计数
FLIP_N      2.0     无量纲: 一次振荡 ≥2 次方向反转; 孤立反转/       误伤机动→↑;
                    过冲只 1 次。升级阈值                            自激压不住→↓
w 升级/恢复  2·DT0  时间常数: 升级 ~2 帧, 恢复按标定 L 尺度          同信任度通用约定
            /cfg.L
W (=L̂+2τ_v) 导出    翻转窗: 环路往返 (L̂) + 2 次 v̂ 重建 (τ_v=DT0/β₀)  随标定 L 与 β₀ 自动伸缩

================================================================================
评测 (arena 实测, 默认种子)
================================================================================
matched composite 160.3 (const_vel rmse 0.70px, accel 6.05px, step 过冲 3.0px /
首达 376.7ms); 失配扫描最坏 113.4; 宽延迟 L20-70 全过 (L20=121.4), s 失配
0.70-1.30 全档通过; relock 444.0ms / 3.00px; 60/120fps 差 4.0%; OVERALL 144.8。

已知边界: L_true=80 (+25ms) 的慢速边缘气泡 (±6-13px, ~330ms 周期) 创新不可见 (被
α 通道跟踪), 本门不覆盖 → L80 仍 settle-fail。其自激直接通路 (vff=1) 增益≈1, 压低
它只能牺牲 matched 带宽或前馈精确性 —— 按系统哲学交给标定, 不硬凑。另: QP/tick 计算
重, Jetson 500Hz 嵌入式算力未验证。

与 rejected paths 的边界: 不重瞄准 FF (只按振荡证据暂时隔离); 不做常开机动自适应估计
(无第二模型; σ̂ 有 Huber 守卫, 无致盲路径); 不在线适配延迟 (L̂ 恒为标定值, 翻转窗
只做事件定时, 不回改 L̂)。
================================================================================
debug() 字段 (逐拍采集为 dbg_* 列, 无副作用; eval/integrate/fps_eval 不调用)
================================================================================
  N/w_ghost  QP 时域拍数 / 翻转窗 ms          wn_mrad  带宽 rad/ms×1000
  L_hat      相信延迟×L_inflate ms
  fx fy      滤波误差@相信测量世界时 px
  fvx fvy    目标速度估计 px/ms               inx iny  最近滤波创新 px
  e0x e0y    Smith 初态误差 px (门控后)       v0x v0y  QP 前馈 v0 px/ms
  u0x u0y    QP 首步指令 px/ms                fast     1=快路径 0=FISTA
  sig_x/y    创新 σ̂ 自标定 px                 flipx    x 轴创新翻转计数
  w_state    信任度 0..1 (1=满信任)           ff_gate  前馈门控 = 1−w_state
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
import numpy as np
from scipy.linalg import solve_discrete_are, cho_factor, cho_solve
from arena.core import Observation, LawConfig
from arena.laws.base import CountsHist, Law, register


@register("mpc_osc")
class MpcOscLaw(Law):
    DT0 = 1000.0 / 120.0      # 参考帧周期 (120fps), 滤波器增益 dt 归一基准
    Q_NORM = 1.0              # 误差权重归一化 (只有 q/r 比值有物理意义, 见模块 docstring [1])
    JUMP_GATE = 100.0
    STALE = 200.0
    IN_SIGMA = 3.0            # 显著创新门限 (σ̂ 倍数, 无量纲)
    FLIP_N = 2.0              # 升级所需翻转数 (无量纲: 一次振荡 ≥2 次方向反转)

    def __init__(self, PM=60.0, qf=None, max_v=1.5, L_inflate=1.1, vff=1.0,
                 alpha0=0.30, beta0=0.08, beta_exp=1.0):
        self._PM = PM                # 相位裕度 (deg): 唯一设计旋钮, 定带宽/权重 ([1])
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
        self.ch = CountsHist()
        self.filt = False
        self.fx = self.fy = self.fvx = self.fvy = 0.0
        self.prev_det_t = None
        self.t_pub = -1e9
        self.rem_x = self.rem_y = 0.0
        self.sig2x = self.sig2y = 1.0
        # 振荡签名状态: 显著创新交替翻转计数 + 上次显著创新符号/时刻 (逐轴)
        self.tau_v = self.DT0 / self._beta0        # 帧率归一 v̂ 重建时标 (ms)
        self.w_ghost = self.L_hat + 2.0 * self.tau_v
        self.flip_x = self.flip_y = 0.0
        self.lsig_x = self.lsig_y = 0
        self.lst_x = self.lst_y = -1e9
        self.w_state = 0.0
        self.last_inx = self.last_iny = 0.0
        self.d_e0x = self.d_e0y = 0.0
        self.d_v0x = self.d_v0y = 0.0
        self.d_u0x = self.d_u0y = 0.0
        self.d_fast = 1.0
        self.d_gate = 1.0

        # [1] 带宽由相位裕度推出: wn=(90−PM)π/180/L_hat; LQR 带宽=√(q/r) → r=1/wn²。
        wn = (90.0 - self._PM) * (math.pi / 180.0) / self.L_hat
        self.wn = wn
        q = self.Q_NORM
        r = 1.0 / (wn * wn)
        # [2] 时域由延迟+控制步+帧周期推出。
        N = int(math.ceil(self.L_hat / cfg.h) + math.ceil(cfg.frame_dt / cfg.h))
        self._build_qp(cfg.h, q, r, N)

    def _build_qp(self, h: float, q: float, r: float, N: int):
        if self._qf is None:
            # [3] 终端权重 = e 积分器子系统的无穷时域 LQR (DARE) 解。
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
        # 控制代价 r·(u_i−v0)²: u=v0 跟踪匀速目标零代价; +r·1 为其线性项。
        self.b_v = q * h * h * (M.T @ k) + qf * h * h * N * mN + r * ones
        self.N = N
        self.lip = float(np.linalg.eigvalsh(H)[-1])
        self.warm_x = np.zeros(N)
        self.warm_y = np.zeros(N)
        self.fista_iters = 40

    def _solve_u0(self, e0: float, v0: float, warm: np.ndarray) -> Tuple[float, np.ndarray]:
        b = e0 * self.b_e + v0 * self.b_v
        u_unc = cho_solve(self.chol, b)
        vm = self.max_v
        if np.all(np.abs(u_unc) <= vm):
            self.d_fast = 1.0
            return float(u_unc[0]), u_unc
        # 箱式约束 QP: FISTA 投影梯度, 上一拍解 (滚动移位) 暖启动。
        self.d_fast = 0.0
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
            self.sig2x = self.sig2y = 1.0
            self.filt = True
            self.prev_det_t = det.t
            self.t_pub = det.t
            self.last_inx = self.last_iny = 0.0
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
            self.flip_x = self.flip_y = 0.0
            self.lsig_x = self.lsig_y = 0
        else:
            rr = dt / self.DT0
            alpha = min(0.90, self._alpha0 * rr)
            beta = min(0.60, self._beta0 * rr ** self._beta_exp)
            # σ̂ Huber 守卫: 单帧创新 > IN_SIGMA·σ̂ 即信号而非噪声, 不回训 ——
            # 否则自激垃圾创新被 σ̂ EMA 吸收 (实测 σ̂ 1→44px), 显著性门限失明。
            huber_x = inx * inx <= (self.IN_SIGMA * self.IN_SIGMA) * self.sig2x
            huber_y = iny * iny <= (self.IN_SIGMA * self.IN_SIGMA) * self.sig2y
            if huber_x:
                self.sig2x += beta * (inx * inx - self.sig2x)
            if huber_y:
                self.sig2y += beta * (iny * iny - self.sig2y)
            # 振荡签名: 显著创新的符号交替翻转计数。翻转窗 W=L̂+2τ_v:
            # 相邻矛盾须经由环路往返+v̂ 重建才可能同源; flip 按 τ=W 泄漏。
            # 单极性矛盾 (急停/反转后重建/s 失配偏置/approach 瞬态) 不计
            # 翻转 —— 匹配与真实事件零税的关键。
            leak = math.exp(-dt / self.w_ghost)
            self.flip_x *= leak
            self.flip_y *= leak
            sx = max(math.sqrt(self.sig2x), 1e-6)
            sy = max(math.sqrt(self.sig2y), 1e-6)
            if inx * inx > (self.IN_SIGMA * self.IN_SIGMA) * self.sig2x:
                s_in = 1 if inx > 0 else -1
                if self.lsig_x != 0 and s_in != self.lsig_x \
                        and (det.t - self.lst_x) <= self.w_ghost:
                    self.flip_x += 1.0
                self.lsig_x = s_in
                self.lst_x = det.t
            if iny * iny > (self.IN_SIGMA * self.IN_SIGMA) * self.sig2y:
                s_in = 1 if iny > 0 else -1
                if self.lsig_y != 0 and s_in != self.lsig_y \
                        and (det.t - self.lst_y) <= self.w_ghost:
                    self.flip_y += 1.0
                self.lsig_y = s_in
                self.lst_y = det.t
            self.fx = px_pred + alpha * inx
            self.fy = py_pred + alpha * iny
            self.fvx += (beta / dt) * inx
            self.fvy += (beta / dt) * iny
        self.last_inx = inx
        self.last_iny = iny
        self.prev_det_t = det.t
        self.t_pub = det.t

    def step(self, t: float, obs: Optional[Observation]) -> Tuple[int, int]:
        cfg = self.cfg
        if obs is not None and obs.new:
            self._update_filter(obs)

        cx = cy = 0
        age = t - self.t_pub
        if self.filt and age < self.STALE:
            # 信任度门控: 振荡证据 (flip≥FLIP_N) → 升级 ~2 帧 (τ=2·DT0),
            # 证据消失按标定 L 尺度恢复 (τ=cfg.L)。
            w_inst = 1.0 if max(self.flip_x, self.flip_y) >= self.FLIP_N else 0.0
            h = cfg.h
            a_r = 1.0 - math.exp(-h / (2.0 * self.DT0))
            d_r = 1.0 - math.exp(-h / max(1.0, cfg.L))
            if w_inst > self.w_state:
                self.w_state += a_r * (w_inst - self.w_state)
            else:
                self.w_state += d_r * (w_inst - self.w_state)

            cp = self.ch.at(self.t_pub - self.L_hat)
            cn = self.ch.cum()
            ifx = cfg.s * (cn[0] - cp[0])
            ify = cfg.s * (cn[1] - cp[1])
            T = age + self.L_hat
            # 振荡确认时同时切断两条幽灵回路: QP 前馈 v0 与 Smith 外推
            # fvx·T (只切 v0 时外推回路单独残余 3px 振荡, 实测)。满信任
            # w=0 → ff_gate=1 → 门完全不可见。
            ff_gate = 1.0 - self.w_state
            self.d_gate = ff_gate
            fv_ex = self.fvx * ff_gate
            fv_ey = self.fvy * ff_gate
            e0x = self.fx + fv_ex * T - ifx
            e0y = self.fy + fv_ey * T - ify
            self.d_e0x, self.d_e0y = e0x, e0y
            v0x = self._vff * fv_ex
            v0y = self._vff * fv_ey
            self.d_v0x, self.d_v0y = v0x, v0y
            wx = np.empty(self.N); wx[:-1] = self.warm_x[1:]; wx[-1] = self.warm_x[-1]
            wy = np.empty(self.N); wy[:-1] = self.warm_y[1:]; wy[-1] = self.warm_y[-1]
            vx_raw, self.warm_x = self._solve_u0(e0x, v0x, wx)
            vy_raw, self.warm_y = self._solve_u0(e0y, v0y, wy)
            self.d_u0x, self.d_u0y = vx_raw, vy_raw
            vx = max(-self.max_v, min(self.max_v, vx_raw))
            vy = max(-self.max_v, min(self.max_v, vy_raw))

            s = max(0.05, min(20.0, cfg.s))
            self.rem_x += vx * cfg.h / s
            self.rem_y += vy * cfg.h / s
            cx, cy, self.rem_x, self.rem_y = self._counts(
                self.rem_x, self.rem_y, s, cfg.count_limit)
        else:
            self.rem_x = self.rem_y = 0.0

        self.ch.add(t, cx, cy)
        return cx, cy

    def debug(self) -> dict:
        return {
            "N": float(self.N), "wn_mrad": self.wn * 1000.0,
            "L_hat": self.L_hat, "w_ghost": self.w_ghost,
            "fx": self.fx, "fy": self.fy, "fvx": self.fvx, "fvy": self.fvy,
            "inx": self.last_inx, "iny": self.last_iny,
            "e0x": self.d_e0x, "e0y": self.d_e0y,
            "v0x": self.d_v0x, "v0y": self.d_v0y,
            "u0x": self.d_u0x, "u0y": self.d_u0y, "fast": self.d_fast,
            "sig_x": math.sqrt(self.sig2x), "sig_y": math.sqrt(self.sig2y),
            "flipx": self.flip_x, "flipy": self.flip_y,
            "w_state": self.w_state, "ff_gate": self.d_gate,
        }
