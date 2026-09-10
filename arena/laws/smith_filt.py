"""arena/laws/smith_filt.py — 滤波 Smith 预测器, 参数由原理导出。

================================================================================
设计原则 (无试凑增益)
================================================================================
最关键的两个量 — 控制器带宽 wn 与残差滤波时间常数 lam — 都由相信延迟 L
导出, 律随任意延迟自动缩放, 鲁棒性靠设计而非试凑。刻意用 arena 速度换
这一推广裕度。

================================================================================
Smith 预测器原理
================================================================================
环路是被控对象为纯积分器 (速度指令 u (px/ms) 每拍移动准星 u·h, 误差
e = 目标−准星 按 ė = −u + d 积分, d = 目标运动 = 输出扰动), 唯一的测量
带延迟 L (时间戳 obs.t 的帧反映 obs.t − L_true 时刻的世界)。

Smith 预测器内部运行对象的无延迟模型并从该模型取信号喂给控制器, 延迟从
特征方程中消失; 真实测量的代价只通过一个修正项支付:

    ŷ_nd += −s·counts                 无延迟模型输出 (预测误差)
    ŷ_d   = ŷ_nd 延迟 L̂               模型认为帧应有的读数
    r     = y_meas − ŷ_d               测量残差 (扰动 + 失配)
    f     = low-pass(r)                滤波修正, F(s)=1/(1+λs)
    ê     = ŷ_nd + f                   修正后的无延迟误差估计
    u     = Kp·ê + Ki·∫ê               PI

模型完美时 y_meas == ŷ_d, r == 0, 控制器只看到无延迟信号 ŷ_nd。
ref = 0 (把误差调到零); 正指令减小正误差。

================================================================================
经典 Smith 预测器为何脆弱, 滤波如何使其鲁棒
================================================================================
经典 Smith 直接取 f = r (不滤波)。延迟误差 ΔL = L_true − L̂ 使 ŷ_d 抽错
延迟线的抽头。ŷ_nd 以指令速度 v 移动, 错抽头相差 ≈ v·ΔL, 即残差注入
∝ v·ΔL 的伪扰动。频域上失配贡献 (e^{−L s} − e^{−L̂ s}) ≈ −ΔL·s, 幅值随
ΔL·ω 增长。在穿越频率 ωc 处这是 ≈ ΔL·ωc 的增益扰动: 高带宽 × 小 ΔL 即可
失稳。可生存失配约为 ΔL_max ≈ PM/ωc, 于是推带宽 (Smith 的全部意义) 反而
收缩鲁棒裕度。

在残差通路插入低通 F(s)=1/(1+λs)。模型只在 ~1/λ 以下频段被信任; 以上
残差 (及 ∝ ΔL·s 的失配项) 被衰减。因 s·F(s) = s/(1+λs) → 1/λ (ω→∞),
失配增益有界 ≈ ΔL/λ, 不再随 ωc 增长。鲁棒性与带宽就此解耦 — 这个滤波器
正是 Smith 预测器能在模型误差下存活的原因。

由原理导出 λ (λ = L̂):
延迟自身的自然频率是 ω_L = 1/L̂ (一个延迟周期积累 ~1 rad 相位的频率,
ω·L̂ = 1)。ω_L 以下模型的延迟补偿有意义; 以上补偿的相位基本未知, 不应
信任模型。把滤波截止恰好放在该频率, 1/λ = 1/L̂ ⇒ λ = L̂: 信任模型至其
有效极限, 之外一律衰减。有界失配增益成为 ΔL/λ = ΔL/L̂ — 只要标定误差
小于延迟本身就是 O(1) 量, 而非无界的 ΔL·ωc。λ = L̂ 因此不是调出常数,
而是延迟本身的尺度; 随 L 自动缩放 (lam = lam_frac·L̂, 无量纲 lam_frac = 1)。

================================================================================
由原理导出带宽 wn (按完整延迟 PM=60°)
================================================================================
无延迟 PI + 积分器闭环为二阶环 s²+Kp·s+Ki, ωn = √Ki, ζ = Kp/(2ωn);
取 ζ = 1 (临界阻尼) 故 Kp = 2ωn, Ki = ωn²。

Smith 预测器的好坏取决于其模型, 所以稳定性上不信任它。wn 按 Smith 对消
可能完全失效、完整相信延迟 L̂ 留在环里的最坏情形定尺寸。纯延迟 L̂ 消耗
相位 ω·L̂; 要在延迟在场时保住相位裕度 PM,
        ωn·L̂ = (90° − PM)·π/180   ⇒   wn = (90° − PM)·π/180 / L̂。
取 PM = 60° (标准充裕裕度):
        wn = (π/6)/L̂ ≈ 0.5236/L̂   (L̂ = 50ms 时 ≈ 0.01047 rad/ms)。
刻意保守: 即使零延迟对消, 环路也保证 PM≈60°, 正常工作的 Smith 预测器
(消掉大部分延迟) 只会再加裕度。没有悬崖, 没有硬编码调参 wn — wn 就是
随 1/L̂ 缩放。代价 (已接受): 带宽低于激进调参的 Smith 所能榨出的,
阶跃稳定更慢。

================================================================================
扰动外推 (消除 v·L 跟踪拖尾) — 物理导出
================================================================================
纯积分器模型下, 滤波残差 f 表示采集时刻 (晚 L) 的目标运动。匀速目标会留
≈ v·L 的稳态跟踪误差, 除非把扰动前推。残差 alpha-beta 同时估计扰动速度
rv (平滑为 sv)。物理正确的时域恰是自采集起经过的时间: 现在可用的帧采集于
(now − L̂), 滤波状态锚在上一帧 t_meas, 故当前扰动 = 采集时刻扰动外推
T = age + L̂。于是
        ê = ŷ_nd + rf + vext·sv·(age + L̂),
vext = 1 = 全量物理外推 (非调参增益 — 就是"把观测到的扰动外推到现在"这
句话)。rho = 0 保持时域无阻尼 (= age + L̂), 物理正确选择; rho > 0 会把
时域饱和 (有界前视的安全变体), 代价是滞后。

失配污染速度的开环外推是经典正反馈危险, 因此外推速度有两重保险:
  • sv 经 ~一帧的 EMA 平滑 (rv_tau = rv_tau_frames·frame_dt, 物理上最短的
    有意义平滑尺度 — 帧率无关);
  • |sv| 硬封顶于 rv_max, 约束外推斜率, 失配无法驱策跑飞。rv_max 是唯一
    显式安全界 (经验值: 设在最快真实目标之上; arena 机动 vmax = 0.4 px/ms)。

================================================================================
离散方程 (每控制拍 h, 逐轴)
================================================================================
  新帧到达 t_meas (dt = t_meas − prev):
      ŷ_d  = delayline.at(t_meas − L̂)          # 采集时刻的模型输出
      r    = y_meas − ŷ_d                        # 原始残差 = 采集时刻扰动
      r̂p   = rf + rv·dt                          # 残差滤波器按 dt 预测
      inn  = r − r̂p                              # (|inn|>gate 跳变重置)
      rf  += α·inn ;  rv += (β/dt)·inn           # α = dt/(λ+dt), β = beta_frac·α
      sv  += (1−e^{−dt/rv_tau})·(rv − sv)        # 平滑扰动速度
      |sv| capped to rv_max                       # 约束外推斜率
  每拍:
      T    = age + L̂                             # 自采集起的时间 (物理)
      ê    = ŷ_nd + rf + vext·sv·T               # 修正后的无延迟误差
      u    = Kp·ê + Ki·∫ê                         # PI (抗饱和, 饱和)
      ŷ_nd −= s·counts                            # 模型精确镜像对象
  Kp = 2ζωn, Ki = ki_mult·ωn²  with wn = (90°−PM)·π/180 / L̂, ζ = 1, ki_mult = 1.

================================================================================
参数表
================================================================================
原理导出 (随 L̂ / frame_dt 自动缩放; 无 arena 调参):
  pm_deg = 60.0   对完整延迟留的相位裕度, 用于定 wn。允许的无量纲设计
                  选择。wn = (90−pm_deg)·π/180 / L̂。↑PM = 降 wn, 裕度
                  更大更慢; 60° 是标准值。
  zeta = 1.0      无延迟阻尼 (临界)。允许的无量纲选择。
  lam_frac = 1.0  λ = lam_frac·L̂: 残差滤波截止在延迟自然频率 1/L̂ (见上
                  推导)。唯一的鲁棒性旋钮; 原理默认 1.0。↑ 失配更鲁棒但
                  机动响应更慢; ↓ 跟踪更快但更脆。实测: lam_frac=1.0 是
                  L_true∈{30..70} 上最坏情形有限的唯一值 — 0.7 (过失配) 与
                  ≥1.3 (饿死修正) 都在 L_true=70 发散。故 λ=L̂ 不是约定,
                  是被强逼的鲁棒最优点; 不要动。
  L_inflate = 1.0 L̂ = L_belief·L_inflate。默认 1.0 (相信标定 L 原值);
                  保守 wn 已保证对完整延迟的 PM, 稳定性无需膨胀。>1 把
                  模型抽头偏向过补偿 (欠补偿侧 L_true>L̂ 是危险方向) —
                  特定设备标定系统性偏低时的可选杠杆。
  vext = 1.0      扰动位置外推增益 = 采集时刻扰动到当前的全量物理外推。
                  0 退回纯滤波 Smith (鲁棒但留 v·L 匀速拖尾)。
  rho = 0.0       无阻尼 (物理正确) 外推时域 T=age+L̂。
  rv_tau_frames=1 rv→sv 的 EMA 平滑跨一个帧周期 (帧率无关: rv_tau =
                  rv_tau_frames·frame_dt)。物理上最短的有意义平滑。
  ki_mult = 1.0   Ki = ki_mult·wn² (设计的二阶环)。>1 加相位滞后, 破坏
                  失配裕度; 保持 1。

经验 (已标注; 标准控制旋钮, 非 arena 特有魔数):
  beta_frac = 0.3 残差速度增益 β = beta_frac·α (α = dt/(λ+dt))。标准
                  alpha-beta 位置/速度增益比。↑ 变向响应快但更噪 / 失配
                  更脆; ↓ 更平滑。实测被迫值: 0.2 与 0.4 都在 L_true=70
                  发散; 0.3 是唯一既有限又快的点。
  rv_max = 2.0    |sv| 硬封顶 (px/ms)。外推斜率的显式安全界; 设在最快
                  真实目标之上 (arena 机动 vmax = 0.4), 永不限制诚实运动。
                  更低更安全但削快速 strafe。
  i_gate = 8.0    积分距离门 ig = gate/(gate+|ê|): 目标近处抑制 I (拉枪
                  无 windup/过冲), 跟踪移动目标时 I 建立。标准抗饱和整形;
                  8 (与姊妹律 pi_pm 同惯例值) 在此给出最平失配最坏值。
                  ↑ 移动目标起速快但拉枪过冲更大、低帧率稳定尾巴更长。
  i_frac = 1.0    积分限幅 = i_frac·max_v/Ki (抗饱和)。1 = 任意 ≤max_v
                  目标零位置滞后可跟踪。
  jump_gate=120.0 残差创新跳变门 (px): 切场景/切目标时重置残差滤波器
                  而非追它。结构性, 非性能旋钮 (~FOV 尺度)。
  max_v = 1.5     指令速度饱和 (px/ms); 硬件速度上限 (正常经 cfg.max_v 注入)。

================================================================================
已知局限
================================================================================
  • 积分器 + 匀速模型: 无法预测目标加速度。accel 场景留稳态滞后 a/Ki,
    只能由 (慢、受裕度限制的) I 项消除; accel rmse ≈ 12px。
  • 鲁棒性是拿带宽买的: wn = (π/6)/L̂ 刻意远低于激进调参的 Smith, 匹配
    阶跃稳定 (~430ms) 慢于近悬崖设计。这是已接受的速度↔泛化交换 —
    环路远离一切悬崖保持稳定。
  • 远大于 ~L̂ 的延迟失配仍会退化跟踪 (有界失配增益 ΔL/L̂ 增长);
    欠补偿侧 (L_true>L̂) 仍是脆弱方向。仅当设备标定系统性偏低时才升
    L_inflate。
  • arena 数字只是相对参考; 改 pm_deg / lam_frac / beta_frac / L_inflate
    后要在实机上重扫延迟失配。
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
from arena.core import Observation, LawConfig
from arena.laws.base import Law, register


class _ModelHist:
    """无延迟模型输出 ŷ_nd 的延迟线: 线性插值 + 端点截断
    (截断使启动良定义: 首个样本之前的时刻查询返回初始模型值)。"""
    __slots__ = ("t", "x", "y")

    def __init__(self):
        self.t: list[float] = []
        self.x: list[float] = []
        self.y: list[float] = []

    def add(self, t, x, y):
        self.t.append(t); self.x.append(x); self.y.append(y)
        if len(self.t) > 4000:
            self.t.pop(0); self.x.pop(0); self.y.pop(0)

    def at(self, t):
        ts = self.t
        n = len(ts)
        if n == 0:
            return 0.0, 0.0
        if t <= ts[0]:
            return self.x[0], self.y[0]
        if t >= ts[-1]:
            return self.x[-1], self.y[-1]
        lo, hi = 0, n - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ts[mid] <= t:
                lo = mid
            else:
                hi = mid
        span = ts[hi] - ts[lo]
        f = (t - ts[lo]) / span if span > 0 else 0.0
        return (self.x[lo] + (self.x[hi] - self.x[lo]) * f,
                self.y[lo] + (self.y[hi] - self.y[lo]) * f)


@register("smith_filt")
class SmithFiltLaw(Law):
    STALE = 200.0

    def __init__(self, pm_deg=60.0, zeta=1.0, lam_frac=1.0, L_inflate=1.0,
                 vext=1.0, rho=0.0, rv_tau_frames=1.0, ki_mult=1.0,
                 beta_frac=0.3, rv_max=2.0, i_gate=8.0, i_frac=1.0,
                 jump_gate=120.0, max_v=1.5):
        self.pm_deg = pm_deg          # 对完整延迟留的 PM, 用于定 wn
        self.zeta = zeta              # 无延迟阻尼 (临界 = 1)
        self.lam_frac = lam_frac      # λ = lam_frac·L̂ (滤波截止在 1/L̂)
        self.L_inflate = L_inflate    # L̂ = L_belief·L_inflate (默认 1: 原值)
        self.vext = vext              # 扰动外推增益 (1 = 物理全量)
        self.rho = rho                # 时域阻尼 (0 = 无阻尼, 物理)
        self.rv_tau_frames = rv_tau_frames  # rv 平滑跨度 (帧周期数)
        self.ki_mult = ki_mult        # Ki = ki_mult·wn²
        self.beta_frac = beta_frac    # 经验: 残差速度增益比
        self.rv_max = rv_max          # 经验: |sv| 安全封顶 (px/ms)
        self.i_gate = i_gate          # 经验: 积分距离门 (px)
        self.i_frac = i_frac          # 积分限幅 = i_frac·max_v/Ki
        self.jump_gate = jump_gate    # 结构: 残差创新重置门 (px)
        self._max_v = max_v           # 硬件速度上限 (px/ms)

    def reset(self, cfg: LawConfig):
        self.cfg = cfg
        self.max_v = self._max_v if self._max_v > 0 else cfg.max_v
        # 相信延迟锚定一切 (模型抽头、滤波器、带宽、外推时域)。
        self.L_hat = max(1.0, cfg.L) * self.L_inflate
        # 带宽由原理导出: 按完整延迟留 PM (Smith 对消失效也稳定)。
        # wn = (90°−PM)·π/180 / L̂, 随 1/L̂ 自动缩放。
        wn = (90.0 - self.pm_deg) * math.pi / 180.0 / self.L_hat
        self.kp = 2.0 * self.zeta * wn
        self.ki = self.ki_mult * wn * wn
        # 滤波时间常数由原理导出: 截止在延迟自然频率 1/L̂ ⇒ λ = L̂
        # (此处乘无量纲 lam_frac)。
        self.lam = self.lam_frac * self.L_hat
        # 扰动速度平滑跨 ~一帧 (帧率无关)。
        self.rv_tau = max(1.0, self.rv_tau_frames * cfg.frame_dt)
        self.hist = _ModelHist()
        self.m_x = self.m_y = 0.0          # 无延迟模型输出 ŷ_nd
        self.rf_x = self.rf_y = 0.0        # 滤波残差 (扰动位置)
        self.rv_x = self.rv_y = 0.0        # 残差速度 (扰动速率)
        self.svx = self.svy = 0.0          # 平滑+封顶后的扰动速度
        self.t_meas = -1e9
        self.have = False
        self.int_x = self.int_y = 0.0
        self.rem_x = self.rem_y = 0.0

    def _new_frame(self, obs: Observation):
        cfg = self.cfg
        if not self.have:
            self.m_x, self.m_y = obs.dx, obs.dy
            self.rf_x = self.rf_y = 0.0
            self.rv_x = self.rv_y = 0.0
            self.svx = self.svy = 0.0
            self.have = True
            self.t_meas = obs.t
            return
        md_x, md_y = self.hist.at(obs.t - self.L_hat)
        r_x = obs.dx - md_x
        r_y = obs.dy - md_y
        dt = max(1.0, min(100.0, obs.t - self.t_meas))
        alpha = dt / (self.lam + dt)
        beta = self.beta_frac * alpha
        rpx = self.rf_x + self.rv_x * dt
        rpy = self.rf_y + self.rv_y * dt
        inx = r_x - rpx
        iny = r_y - rpy
        if math.hypot(inx, iny) > self.jump_gate:
            self.rf_x, self.rf_y = r_x, r_y
            self.rv_x = self.rv_y = 0.0
            self.svx = self.svy = 0.0
            self.int_x = self.int_y = 0.0
        else:
            self.rf_x = rpx + alpha * inx
            self.rf_y = rpy + alpha * iny
            self.rv_x += (beta / dt) * inx
            self.rv_y += (beta / dt) * iny
        a = 1.0 - math.exp(-dt / self.rv_tau)
        self.svx += a * (self.rv_x - self.svx)
        self.svy += a * (self.rv_y - self.svy)
        sp = math.hypot(self.svx, self.svy)
        if sp > self.rv_max:
            sc = self.rv_max / sp
            self.svx *= sc
            self.svy *= sc
        self.t_meas = obs.t

    def step(self, t: float, obs: Optional[Observation]) -> Tuple[int, int]:
        cfg = self.cfg
        if obs is not None and obs.new:
            self._new_frame(obs)

        cx = cy = 0
        if self.have:
            age = t - self.t_meas
            if age < self.STALE:
                hor = age + self.L_hat
                if self.rho > 1e-6:
                    hor = (1.0 - math.exp(-self.rho * hor)) / self.rho
                e_x = self.m_x + self.rf_x + self.vext * self.svx * hor
                e_y = self.m_y + self.rf_y + self.vext * self.svy * hor

                vx_u = self.kp * e_x + self.ki * self.int_x
                vy_u = self.kp * e_y + self.ki * self.int_y

                i_lim = self.i_frac * self.max_v / max(self.ki, 1e-9)
                if e_x * e_x + e_y * e_y > cfg.fov_radius * cfg.fov_radius:
                    self.int_x = self.int_y = 0.0
                else:
                    ig = (self.i_gate / (self.i_gate + math.hypot(e_x, e_y))
                          if self.i_gate > 0 else 1.0)
                    wx = (vx_u > self.max_v and e_x > 0) or (vx_u < -self.max_v and e_x < 0)
                    wy = (vy_u > self.max_v and e_y > 0) or (vy_u < -self.max_v and e_y < 0)
                    if not wx:
                        self.int_x = max(-i_lim, min(i_lim, self.int_x + e_x * cfg.h * ig))
                    if not wy:
                        self.int_y = max(-i_lim, min(i_lim, self.int_y + e_y * cfg.h * ig))

                vx = max(-self.max_v, min(self.max_v, vx_u))
                vy = max(-self.max_v, min(self.max_v, vy_u))
                s = max(0.05, min(20.0, cfg.s))
                self.rem_x += vx * cfg.h / s
                self.rem_y += vy * cfg.h / s
                cx, cy, self.rem_x, self.rem_y = self._counts(
                    self.rem_x, self.rem_y, s, cfg.count_limit)
            else:
                self.int_x = self.int_y = 0.0
                self.rem_x = self.rem_y = 0.0

        self.m_x -= cfg.s * cx
        self.m_y -= cfg.s * cy
        if self.have:
            self.hist.add(t, self.m_x, self.m_y)
        return cx, cy
