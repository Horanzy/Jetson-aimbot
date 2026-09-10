"""arena/laws/imm_pi.py — 极点配置 PI + type-2 速度前馈, IMM 机动自适应估计器。

结构 (与 ff_pi 同一控制回路, 估计器换代):
    ê = f + v̂·(age+L̂·L_COMP) − s·Σcounts(in flight)        // Smith 预测误差
    wn = (90°−PM)·π/180 / L̂,  Kp = 2ζ·wn,  Ki = wn²          // 极点配置 (ζ=1)
    gate = I_GATE/(I_GATE+|ê|)                               // I 项距离门控
    v = Kp·ê + Ki·∫ê·gate + FF_GAIN_VAL·gate·gap·v̂_out      // PI + type-2 前馈

估计器: 每轴 2 模型 IMM (Interacting Multiple Model, Blom & Bar-Shalom 1988):
    模型0 低机动 CV (稳态速度增益 β=β0), 模型1 高机动 CV (Γ=√6 天花板)。
    每帧: 输入混合 → 各模型 Kalman 预测/更新 → 卡方门 + 似然 → 贝叶斯模型
    概率更新。模型状态各养各的, 混合值只作输出。

估计器换代的意义 (对 ff_pi 的 "固定 β + CUSUM 归零重拉"):
    固定 β 是一个两难: β 大则机动响应快但噪声直通 FF、失配下估计器污染;
    β 小 (0.03) 则稳但 v̂ 重建 τ ≈ dt/β ≈ 280ms。IMM 把两头分开: 平稳期
    概率聚在低机动模型 (= 原 β0 动态), 机动期似然比几何累积在 2-3 帧内
    翻转 blended v̂ — 高机动模型始终用混合状态热养着, 接管无需从零重建。

快通道的三层纪律 (大创新分辨不了 "真机动" 与 "失配记账突发"):
    失配使 Smith 扣除窗口错位 Δc = Lc−L_真, 自身指令瞬态以 b = s·(最近
    窗口 counts − 相信窗口 counts) 的幅度伪装成创新 (可精确测量)。
      1. μ 稳定 — R_eff = σ̂² + b² 注入模型量测方差, σ_a 锚定 σ̂ 不随
         R_eff 缩放 → Γ_eff 突发期真实下降, 增益收缩; 两模型 S 同步膨胀
         → 似然比趋平, μ 停在先验。
      2. FF 授权门 w — 输出速度 v̂_out = w·v̂_blend + (1−w)·v̂_low 整体
         过门 (Smith 外推与 FF 同源); 混合创新 NIS (χ²₁) 超门 → w 立即
         归零 (单帧误伤率 = 99.9% 分位 = 0.1%), 门内向
         exp(−½·max(0, NIS−1)) 以 2 帧 EMA 恢复。v̂_low 是慢速地板
         (β=β0, ff_pi 出厂动态)。
      3. 幽灵猎杀 — 方向矛盾 CUSUM (同 ff_pi, σ 归一, 作用于 v̂_out):
         告警即两模型速度归零 (位置保留), 环路重跑阶跃响应。

实测成绩 (integrate + fps_eval, 匹配 L=50/120fps/噪声0.5):
    matched composite 104.0 (ff_pi 151.3) — accel rmse 4.25px (ff_pi 7.09),
    maneuver 13.63px (ff_pi 19.41), 全库最优; step settle 443ms 变慢
    (低机动模式位置增益 ~0.58 vs α-β 的 0.50, 尾段略糊)。
    FPS 组全库最优: RMSE 均值 17.1px (ff_pi 20.8), 事件过冲 mean 26.7 /
    worst 69.7 (ff_pi 28.0 / 77.3), REC 7ms (ff_pi 44ms)。

为何落选 — 失配带不存活 (OVERALL = inf):
    延迟失配 L30/L40/L70 与宽延迟 L20-40、L70-80、s0.70 全部 inf: L_真
    阶跃出现 ~43px 过冲的持续极限环 (非发散 — 是 settle 永远不进 3px 带)。
    根因链 (实测轨迹): 失配使 Smith 扣除窗口错位 Δc, 自身指令瞬态以
    −ȧ_own·Δc 伪装成大创新 → 大 ν 下大 S 的高机动模型似然必胜 → μ 翻高
    → 幽灵 v̂ 经 Smith 外推与 FF 双路进入指令。而全部防线都被同一个机制
    致盲: **σ̂ 归一化的门在自持振荡下必然失效** — σ̂ EMA 把振荡吸收为
    "噪声" (实测 σ̂ 0.5 → 24px 单调发散), NIS 分母随之膨胀恒有 NIS ≈ 1
    (Kalman 一致性的本质: 自调谐滤波器对自己的失配永远报 "一致"),
    w 授权门恒 1; CUSUM 增量被同一个 σ̂ 稀释, 永不告警。"分离原理保证
    估计器自适应不改环路鲁棒性" 在这里不成立: 幽灵 v̂ 经物理环路
    (指令 → 准星 → 创新) 闭环回来了, 延迟未补偿的那一段正是振荡的回路。
    三层防线各自的实测失效面: R_eff 膨胀能稳住 μ, 但增益收缩受 Γ 群
    限制、幅度不足以压住 ȧ_own·Δc 量级的污染; NIS 授权门拦得住幽灵
    出生, 拦不住吸收突发后自洽存活的幽灵; CUSUM 猎杀依赖 σ̂ 归一,
    σ̂ 膨胀后失明。
    教训 (与 reseed_pi 的结论汇合): **常开的估计器侧快通道与失配带不相容**;
    ff_pi 的 β0=0.03 慢速 + CUSUM 归零重拉是失配环境下经过验证的折中,
    其重建尾代价 (280ms) 只影响事件后的尾迹, 不影响事件峰值 (峰值 =
    v·L 延迟下界 + 告警延迟)。

参数表 (每项都有依据; σ_a 单位 px/ms², R = 测量噪声方差 px²):
    pm_deg=50, zeta=1, ff_gain=1, l_comp=1.1, i_gate=8, i_frac=1
                 与 ff_pi 完全相同 (控制回路不动, 依据见 ff_pi docstring)。
    SIG_A_LOW    反解自连续性原则: 取 σ_a,low 使 CV Kalman 稳态速度增益 β
                 恰等于 ff_pi 出厂设计点 β0=0.03 (@120fps)。离散 Riccati
                 不动点数值反解 (模块加载时一次); β 只依赖无量纲群
                 Γ = σ_a·T²/σ_m, 故运行中 σ 变化时低机动模式动态不变。
    SIG_A_HIGH   可辨识性天花板: 三点差分能从检测里无偏看见的加速度上限,
                 σ_a,high = √6·σ/T² → Γ = √6 恒定 (与噪声/帧率无关),
                 即 β_high ≈ 0.10 ≈ 3.4×β0 — 更快的模型只能拟合检测噪声
                 (三点差分的加速度不确定度就是 √6σ/T²)。恰在文献 2 模型
                 IMM 的 q 比甜点 (10-30×) 下缘。
    MARKOV_DIAG  0.95   模型转移矩阵对角 (文献标准值): 平均驻留 ~20 帧
                 ≈ 170ms, 匹配人类机动爆发时间尺度; 0.99 起步钝, 0.90 抖。
    CHI2_GATE    10.8   创新卡方门 (1 自由度 99.9% 分位, 统计常数)。
    GATE_EPS     0.01   被门模型的似然保留比例 (降权而非丢弃, 防锁死)。
    MU_FLOOR     0.05   低机动模型概率地板: 高机动模式速度噪声放大
                 ~3.4×, 概率永不全让它吃。
    W_TAU_FRAMES 2.0    FF 授权门上升 EMA 尺度 (帧): 与 CUSUM 告警延迟同级。
    CUSUM_K/C/H  0.5/3/9 (σ 倍数, 同 ff_pi): 漂移 / 单帧增量上限 / 告警门限。
    noise_std=0.5       测量噪声先验 (px/轴): σ̂ EMA 初值锚点, 实机在静止
                 场景实测; 运行中创新方差 EMA 自标定。
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
from arena.core import Observation, LawConfig
from arena.laws.base import CountsHist, Law, register


def _cv_riccati_beta(sa: float, T: float, R: float) -> float:
    """CV Kalman 稳态速度增益 β(σ_a, T, R): 定常 Riccati 不动点迭代 (Joseph
    形式, 任意跟踪指数下稳定收敛)。分段常加速度模型的精确离散
    Q = σ_a²·[[T⁴/4, T³/2],[T³/2, T²]]。β 只依赖无量纲群 Γ = σ_a·T²/√R。"""
    q11 = sa * sa * T ** 4 / 4.0
    q12 = sa * sa * T ** 3 / 2.0
    q22 = sa * sa * T ** 2
    p11, p12, p22 = R, 0.0, 1.0
    k2 = 0.0
    for _ in range(500):
        pp11 = p11 + 2.0 * T * p12 + T * T * p22 + q11
        pp12 = p12 + T * p22 + q12
        pp22 = p22 + q22
        s = pp11 + R
        k1, k2 = pp11 / s, pp12 / s
        a1 = 1.0 - k1
        p11 = a1 * a1 * pp11 + k1 * k1 * R
        p12 = a1 * a1 * pp12 + k1 * k2 * R
        p22 = pp22 - 2.0 * k2 * pp12 + k2 * k2 * (pp11 + R)
    return k2


def _sigma_a_for_beta(beta_target: float, T: float, R: float) -> float:
    """给定目标稳态速度增益反解 σ_a (β 对 σ_a 单调, 二分)。"""
    lo, hi = 1e-9, 1.0
    for _ in range(80):
        mid = math.sqrt(lo * hi)
        if _cv_riccati_beta(mid, T, R) < beta_target:
            lo = mid
        else:
            hi = mid
    return math.sqrt(lo * hi)


@register("imm_pi")
class ImmPILaw(Law):
    DT0 = 1000.0 / 120.0
    JUMP_GATE = 100.0
    STALE = 200.0
    V0_STD = 1.0          # 初始速度先验 std (px/ms); 仅影响暂态
    SIG2_EMA_BETA = 0.03  # 创新方差 EMA 速率 (@120fps, dt 归一; 同 ff_pi 的 σ 自标定)
    MARKOV_DIAG = 0.95
    CHI2_GATE = 10.8      # 1 自由度 99.9% 分位
    GATE_EPS = 0.01
    MU_FLOOR = 0.05
    W_TAU_FRAMES = 2.0    # FF 授权门上升 EMA 尺度 (帧, @120fps)
    # CUSUM 参数 (均为 σ 倍数, 同 ff_pi): K 漂移 / C 单帧增量上限 / H 告警
    CUSUM_K = 0.5
    CUSUM_C = 3.0
    CUSUM_H = 9.0
    BETA0 = 0.03          # ff_pi 出厂速度增益 — σ_a,low 的连续性锚点
    GAMMA_HI = math.sqrt(6.0)  # 高机动模式跟踪指数 = 三点差分可辨识天花板

    def __init__(self, pm_deg=50.0, zeta=1.0, ff_gain=1.0, l_comp=1.1,
                 i_gate=8.0, i_frac=1.0, noise_std=0.5, max_v=1.5):
        # 设计点与 ff_pi 完全一致 (PM=50 失配带全过最快点; ζ=1 临界阻尼)
        self.pm_deg = pm_deg
        self.zeta = zeta
        self.ff_gain = ff_gain
        self.l_comp = l_comp
        self.i_gate = i_gate
        self.i_frac = i_frac
        self.noise_std = noise_std
        self._max_v = max_v
        self.sigma_a_low = _sigma_a_for_beta(self.BETA0, self.DT0, 1.0)

    def reset(self, cfg: LawConfig):
        self.cfg = cfg
        self.max_v = self._max_v if self._max_v > 0 else cfg.max_v
        L = max(1.0, cfg.L)
        wn = (90.0 - self.pm_deg) * math.pi / 180.0 / L
        self.kp = 2.0 * self.zeta * wn
        self.ki = wn * wn
        self.ch = CountsHist()
        self.prev_det_t = None
        self.t_pub = -1e9
        # 每轴 2 模型: x[axis][model] = [e(px), v_t(px/ms 目标速度)];
        # p[axis][model] = (p00,p01,p11) 对称 2×2
        self.x = [[[0.0, 0.0], [0.0, 0.0]] for _ in range(2)]
        self.p = [[(1.0, 0.0, 1.0), (1.0, 0.0, 1.0)] for _ in range(2)]
        self.mu = [(0.5, 0.5), (0.5, 0.5)]
        self.f = [[0.0, 0.0], [0.0, 0.0]]   # blended 位置 + 门控后输出速度
        self.sig2 = [1.0, 1.0]              # 创新方差在线估计 (px²)
        self.w = [0.0, 0.0]                 # FF 授权门 (0=慢速地板, 1=快 v̂)
        self.cs = [0.0, 0.0]                # 方向矛盾 CUSUM (σ 单位)
        self.init = False
        self.int_x = self.int_y = 0.0
        self.rem_x = self.rem_y = 0.0

    def _reinit(self, det: Observation):
        for ax in range(2):
            z = det.dx if ax == 0 else det.dy
            for m in range(2):
                self.x[ax][m] = [z, 0.0]
                self.p[ax][m] = (self.sig2[ax], 0.0, self.V0_STD ** 2)
            self.mu[ax] = (0.5, 0.5)
            self.f[ax] = [z, 0.0]
            self.w[ax] = 0.0
            self.cs[ax] = 0.0
        self.init = True

    def _update_filter(self, det: Observation):
        cfg = self.cfg
        if not self.init:
            self._reinit(det)
            self.prev_det_t = det.t
            self.t_pub = det.t
            return

        T = max(1.0, min(100.0, det.t - self.prev_det_t))
        Lc = cfg.L * self.l_comp
        c0 = self.ch.at(det.t - Lc - T)
        c1 = self.ch.at(det.t - Lc)
        pi_d, pi_o = self.MARKOV_DIAG, 1.0 - self.MARKOV_DIAG

        # 第一遍: 混合状态 (上一帧输出) 的 Smith 式创新 — σ 自标定与
        # JUMP_GATE 判跳变均用它 (与 ff_pi 同口径)。
        # b = s·(最近窗口 counts − 相信窗口 counts): 窗口错位 Δc=Lc−L_真 下
        # 自身指令瞬态伪装成创新的幅度, 突发方差的直接观测 (见 docstring)。
        nu = [0.0, 0.0]
        uu = [0.0, 0.0]
        bb = [0.0, 0.0]
        for ax in range(2):
            z = det.dx if ax == 0 else det.dy
            cax = c1[ax] - c0[ax]
            uu[ax] = cfg.s * cax / T
            bb[ax] = cfg.s * (self.ch.at(det.t)[ax]
                              - self.ch.at(det.t - T)[ax] - cax)
            fe, fv = self.f[ax]
            nu[ax] = z - (fe + fv * T - uu[ax] * T)
            beta_ema = min(0.60, self.SIG2_EMA_BETA * T / self.DT0)
            self.sig2[ax] += beta_ema * (nu[ax] * nu[ax] - self.sig2[ax])
        if not math.isfinite(nu[0]) or not math.isfinite(nu[1]) \
                or math.hypot(nu[0], nu[1]) > self.JUMP_GATE:
            self._reinit(det)
            self.prev_det_t = det.t
            self.t_pub = det.t
            return

        rate_w = 1.0 - math.exp(-T / (self.W_TAU_FRAMES * self.DT0))
        for ax in range(2):
            z = det.dx if ax == 0 else det.dy
            u = uu[ax]
            xs, ps, mu = self.x[ax], self.p[ax], self.mu[ax]
            # Q(σ_a, T): 离散白加速度 (分段常加速度) 精确形式。两模式的 σ_a
            # 都随在线 σ 缩放 → Γ (从而稳态增益) 恒定: 低模式 β≡β0, 高模式
            # Γ≡√6 (β≈0.10)。σ_a,low 反解值在 R=1 下取得, 乘 √σ² 即可。
            sa = (self.sigma_a_low * math.sqrt(self.sig2[ax]),
                  self.GAMMA_HI * math.sqrt(self.sig2[ax]) / (self.DT0 * self.DT0))
            sa2 = (sa[0] * sa[0], sa[1] * sa[1])
            q11 = [sa2[m] * T ** 4 / 4.0 for m in range(2)]
            q12 = [sa2[m] * T ** 3 / 2.0 for m in range(2)]
            q22 = [sa2[m] * T ** 2 for m in range(2)]

            # 1) 输入混合: w_{i|j} = π_ij·μ_i / c̄_j
            cbar = (pi_d * mu[0] + pi_o * mu[1], pi_o * mu[0] + pi_d * mu[1])
            mix_x, mix_p = [], []
            for j in range(2):
                w0 = (pi_d if j == 0 else pi_o) * mu[0] / max(cbar[j], 1e-12)
                w1 = (pi_o if j == 0 else pi_d) * mu[1] / max(cbar[j], 1e-12)
                xe = w0 * xs[0][0] + w1 * xs[1][0]
                xv = w0 * xs[0][1] + w1 * xs[1][1]
                de0, de1 = xs[0][0] - xe, xs[1][0] - xe
                dv0, dv1 = xs[0][1] - xv, xs[1][1] - xv
                p00 = (w0 * (ps[0][0] + de0 * de0)
                       + w1 * (ps[1][0] + de1 * de1))
                p01 = (w0 * (ps[0][1] + de0 * dv0)
                       + w1 * (ps[1][1] + de1 * dv1))
                p11 = (w0 * (ps[0][2] + dv0 * dv0)
                       + w1 * (ps[1][2] + dv1 * dv1))
                mix_x.append((xe, xv))
                mix_p.append((p00, p01, p11))

            # 2) 各模型预测 + 更新 + 门控似然。
            # R_eff = σ̂² + b²: 自身瞬态的记账突发按有界偏差的方差处理;
            # σ_a 锚定 σ̂ 不随 R_eff 缩放 → Γ_eff 突发期真实下降 (增益收缩),
            # 且两模型 S 同步膨胀 → 似然比趋平, μ 停在先验。
            R = max(self.sig2[ax], 1e-4) + bb[ax] * bb[ax]
            like = []
            s_mod = []
            for m in range(2):
                p00, p01, p11 = mix_p[m]
                pp00 = p00 + 2.0 * T * p01 + T * T * p11 + q11[m]
                pp01 = p01 + T * p11 + q12[m]
                pp11 = p11 + q22[m]
                xe = mix_x[m][0] + mix_x[m][1] * T - u * T
                xv = mix_x[m][1]
                s = pp00 + R
                nu_m = z - xe
                k1, k2 = pp00 / s, pp01 / s
                xs[m][0] = xe + k1 * nu_m
                xs[m][1] = xv + k2 * nu_m
                ps[m] = ((1.0 - k1) * pp00, (1.0 - k1) * pp01, pp11 - k2 * pp01)
                g = self.GATE_EPS if nu_m * nu_m > self.CHI2_GATE * s else 1.0
                like.append(g * math.exp(-0.5 * nu_m * nu_m / s)
                            / math.sqrt(2.0 * math.pi * s))
                s_mod.append(s)

            # 3) 模型概率贝叶斯更新 + 低机动地板
            c = cbar[0] * like[0] + cbar[1] * like[1]
            mu0 = max(self.MU_FLOOR, min(1.0, cbar[0] * like[0] / max(c, 1e-300)))
            self.mu[ax] = (mu0, 1.0 - mu0)

            # 4) FF 授权门 + 门控输出: v̂_out = w·v̂_blend + (1−w)·v̂_low
            m0, m1 = self.mu[ax]
            v_low = xs[0][1]
            v_blend = m0 * xs[0][1] + m1 * xs[1][1]
            s_mix = m0 * s_mod[0] + m1 * s_mod[1]
            nis = nu[ax] * nu[ax] / max(s_mix, 1e-9)
            if nis > self.CHI2_GATE:
                self.w[ax] = 0.0
            else:
                w_t = math.exp(-0.5 * max(0.0, nis - 1.0))
                self.w[ax] += rate_w * (w_t - self.w[ax])
            v_out = self.w[ax] * v_blend + (1.0 - self.w[ax]) * v_low

            # 5) 方向矛盾 CUSUM (σ 归一, 作用于 v̂_out — 实际推动指令的量):
            #    自洽幽灵对 NIS 不可见, 但持续矛盾的创新逃不过这里
            sx = max(math.sqrt(self.sig2[ax]), 1e-6)
            if v_out > 0:
                acc = -nu[ax] / sx
            elif v_out < 0:
                acc = nu[ax] / sx
            else:
                acc = -self.CUSUM_K
            self.cs[ax] = max(0.0, self.cs[ax]
                              + min(max(acc, 0.0), self.CUSUM_C)
                              - self.CUSUM_K)
            if self.cs[ax] >= self.CUSUM_H:
                for m in range(2):
                    xs[m][1] = 0.0
                self.cs[ax] = 0.0
                v_out = 0.0

            self.f[ax] = [m0 * xs[0][0] + m1 * xs[1][0], v_out]

        if not all(math.isfinite(self.f[ax][0]) for ax in range(2)):
            self.init = False
        self.prev_det_t = det.t
        self.t_pub = det.t

    def step(self, t: float, obs: Optional[Observation]) -> Tuple[int, int]:
        cfg = self.cfg
        if obs is not None and obs.new:
            self._update_filter(obs)

        cx = cy = 0
        age = t - self.t_pub
        if self.init and age < self.STALE:
            Lc = cfg.L * self.l_comp
            cp = self.ch.at(self.t_pub - Lc)
            cn = self.ch.cum()
            ex = self.f[0][0] + self.f[0][1] * (age + Lc) - cfg.s * (cn[0] - cp[0])
            ey = self.f[1][0] + self.f[1][1] * (age + Lc) - cfg.s * (cn[1] - cp[1])
            vx = self.f[0][1]
            vy = self.f[1][1]
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

            frame_dt = cfg.frame_dt if cfg.frame_dt > 0 else self.DT0
            gap_scale = 1.0 - max(0.0, min(1.0, (age - frame_dt) / max(1.0, cfg.L)))
            ff_eff = self.ff_gain * gate * gap_scale
            vx_u += ff_eff * vx
            vy_u += ff_eff * vy

            vx_u = max(-self.max_v, min(self.max_v, vx_u))
            vy_u = max(-self.max_v, min(self.max_v, vy_u))
            s = max(0.05, min(20.0, cfg.s))
            self.rem_x += vx_u * cfg.h / s
            self.rem_y += vy_u * cfg.h / s
            cx, cy, self.rem_x, self.rem_y = self._counts(
                self.rem_x, self.rem_y, s, cfg.count_limit)
        else:
            self.rem_x = self.rem_y = 0.0
            self.int_x = self.int_y = 0.0

        self.ch.add(t, cx, cy)
        return cx, cy
