"""arena/laws/ff_pi_acc.py — ff_pi + 结构性加速度偏差补偿 (ff_pi 系)。

在 ff_pi (极点配置 PI + type-2 速度前馈 + 方向矛盾 CUSUM 速度归零重拉)
之上增加一个**加速度偏差补偿通道**: 修正 α-β 估计器对匀加速目标的结构性
滞后, 使 Smith 误差 ê 在加速目标上不再系统性少报。控制律主体 (wn/Kp/Ki/
FF_GAIN/CUSUM K·C·H/信任度/丢帧衰减) 与 ff_pi 完全一致; CUSUM 的 σ̂ 归一
化也保持 ff_pi 原语义 (原始创新二阶矩 EMA — 失配伪创新自动撑宽告警门,
该"失明"在过估延迟档是对回弹的有效阻尼, 实测保留更优)。稳健化只作用于
新增的加速度传感器。

结构 (控制增益全部由标定延迟 L̂ 导出, 无手调参数):
    ê = f + (v̂+ε)·W + ½â·W² − s·Σcounts(in flight)      // Smith 预测误差
    wn = (90°−PM)·π/180 / L̂,  Kp = 2ζ·wn,  Ki = wn²      // 极点配置 (ζ=1)
    gate = I_GATE/(I_GATE+|ê|)                           // I 项距离门控
    v = Kp·ê + Ki·∫ê·gate + FF_GAIN_VAL·ff_gate·gap·(v̂+ε) // PI + type-2 前馈
    W = age + L̂·L_COMP,  ε = â·T·(α/β − ½)

加速度偏差补偿 (全部由估计器结构导出, 无调参):
    α-β 滤波对匀加速目标有自洽的稳态签名: 创新均值 ȳ = a·T²/β (非零均值),
    速度估计滞后 ε = a·T·(α/β − ½)。该滞后被外推窗口 W 放大成 ê 的系统性
    偏置 ε·W + ½a·W² (L=50/120fps 实测少报 5-8px): 环路在 ê≈0 处停住而
    真实误差 5-8px (accel 场景 rmse 7.1px 的主项); 60fps 时 T 翻倍 → 少报
    ~10px (60fps 退化的全部来源)。补偿:
      â = ȳ·β/T²          // 创新均值精确反演加速度, 120/60fps 公式自洽
      ε = â·T·(α/β − ½)   // α-β 速度滞后的结构公式
      ê 汇装与 FF 都用 v̂+ε 并加 ½â·W² — 对匀加速目标, "当前真实速度"才是
      type-2 零拖尾的精确开环指令, 补偿强化而非修改 type-2 原则。
    滤波器内部不动 (预测步不加 â 项: 否则 ȳ 传感器自消, 且抬高估计器环路
    增益 — rejected path)。

ȳ 传感器的三重门控 (决定"创新均值何时携带加速度信息"):
    1. 重建抑制: CUSUM 归零 / JUMP 重置后 v̂ 从 0 重建, 期间创新 ≈
       (v_true−v̂)·T 持续 — 任何创新均值传感器都会把它误读成加速度。
       每轴设重建旗标, 保持 ≥ RB_HOLD_N·(T/β) (α-β 速度估计自身时间常数,
       帧率不变量 T/β ≈ DT0/β0 ≈ 278ms) 且创新回到典型水平才解除; 抑制期
       间 ȳ 以自然速率向零衰减, â 强制为 0 — "速度估计明显错误期间, 创新
       均值不携带加速度信息"。真实匀加速 |创新| = aT²/β ≈ 1.4px 在典型
       σ̂ 下不受拦截。
    2. 自身加速度活动门: 延迟失配伪创新 ∝ a_own·Δ (自身加速度×失配量),
       真目标加速度签名 ∝ a_t·T²/β 与自身运动无关 — 物理上可分。a_own 由
       counts 历史精确可得 (L̂ 窗均值速度的帧间差分); 门 = 1/(1+(a_own/
       θ_a)⁶), θ_a = max_v/(OW_ACTIV_K·L̂): 平滑跟踪 (a_own≈a_t) 全开,
       自身剧烈加减速 (伪创新最大时) 全关。
    3. 显著性地板: 稳健 σ̂_r 存的是清洗创新的二阶矩 m₂ = σ_noise² + ȳ²,
       可精确分解 σ_noise² = m₂ − ȳ²; EMA 均值的白噪声地板 =
       ACC_SNR·σ_noise·sqrt(ρ/(2−ρ))。|ȳ| 不过地板 → â = 0。SNR 取值须
       盖过 |Δ|≤30ms 设计失配带下窗口偏移 dither 噪声 (相关噪声, 相关
       膨胀 sqrt((1+ρc)/(1−ρc)), ρc = 1−dt/τc, τc ≈ |Δ|max+dt → 膨胀
       ≈ 2.5-3) 驱动的均值游走 = 3σ白×膨胀 ≈ 8-10; 测试组选定 10 = 宽
       延迟 {20..80} 与 s {0.7..1.3} 全档与 ff_pi 逐位持平的最小整档。
       真实设备更吵时 σ_noise↑ → 地板↑ → 补偿自动退回 ff_pi 行为 (保守
       方向, 好的失效模式)。
    传感器输入清洗: 滤波预测扣的是 Lc=L̂·1.1 窗的 counts, 真实世界效应是
    L_true 窗, 两者之差在自身加/减速时非零 (matched 下也存在, 自身急减
    ~±6px/帧)。减去 s·[(Lc 窗)−(L̂ 窗)] (法则自己已知的过度补偿伪迹) 后,
    matched 时恰好还原真实目标创新; 失配残留 ∝ Δ·a_own, 瞬态成对, 由门 2
    吸收。输入再经 Huber 截断 (±SIG_CLIP_K·σ̂_r, 无量纲 M-estimator 标准
    断点) 供 σ̂_r 与 ȳ 共用 — 少数爆发不污染尺度与均值。
    统一衰减原则: 无新鲜证据 (重建期 / 门关 / CUSUM 矛盾积累期) 时 ȳ 以
    自然速率向零衰减; 有证据时按门缩放进入。

设计点 (失配带测试选定, 非手感): PM=50 (全延迟带 L20-80 通过的最快点);
β0=0.03 (带边缘余量) — 与 ff_pi 相同。

评测 (arena 实测, 默认种子; 与 ff_pi 逐位对比):
    matched 114.7 (ff_pi 151.3): step 两场景 / const_vel / maneuver 逐位
    相同; accel rmse 7.09→4.37, in_band 15.1%→77.6%。
    失配扫描 L30-70 与 ff_pi 逐位相同 (worst 125.12); 宽延迟 L20-70 与
    s {0.7..1.3} 全档逐位相同 (L80 settle-fail 持平); relock 386.0ms/
    3.16px 逐位相同; fps_eval 事件过冲/恢复逐位相同, RMSE 20.75→20.40;
    60/120fps 差 9.2%→1.9%; OVERALL 156.67→123.62。
    holdout (seeds 4,5,6): matched 145.16→111.96, accel 6.61→4.36,
    fpsΔ 16.8%→10.5%, OVERALL 168.51→139.37, 其余逐位相同。

debug() 字段语义 (px / px/ms / px/ms² / σ):
  ex, ey           Smith 汇装误差 ê (补偿后, 驱动 P/I)
  fvx, fvy         α-β 速度估计 v̂ (未修正)
  ax_e, ay_e       加速度估计 â (0 = 未通过显著性/重建/门控)
  ybar_x, ybar_y   创新均值 EMA ȳ (â 传感器)
  fx, fy           α-β 位置估计
  int_x, int_y     I 项积分器状态 (px·ms)
  csx, csy         CUSUM 状态 S (σ 单位, 原始 σ̂ 归一)
  sigx, sigy       CUSUM 尺度 σ̂ = sqrt(原始创新二阶矩)
  sigrx, sigry     â 传感器稳健尺度 σ̂_r = sqrt(清洗创新二阶矩)
  w_state          FF 信任度 0..1
  w_inst           CUSUM 告警电平 0..1 (未滤波)
  ff_eff           实际前馈有效增益
  gate             I 项距离门控 0..1
  age              当前拍距最新检测 ms
  accx, accy       本帧 CUSUM 增量 (σ 单位, 截断前)
  aown_x, aown_y   自身加速度 (px/ms², counts 历史精确差分)
  reb_x, reb_y     重建抑制旗标 (1 = v̂ 重建期)
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
from arena.core import Observation, LawConfig
from arena.laws.base import CountsHist, Law, register


@register("ff_pi_acc")
class FFPiAccLaw(Law):
    DT0 = 1000.0 / 120.0
    JUMP_GATE = 100.0
    STALE = 200.0
    # CUSUM 参数 (均为 σ 倍数, 无量纲): K 漂移 / C 单帧增量上限 / H 告警
    CUSUM_K = 0.5
    CUSUM_C = 3.0
    CUSUM_H = 9.0
    # Huber 截断断点 (σ 倍数, 无量纲 M-estimator 设计选择): 清洗创新在
    # ±K·σ̂_r 内进入尺度/均值估计, 少数爆发不污染; 高斯下无偏, 真实噪声
    # 水平变化仍能在数帧内自适应。太小→重尾误报, 太大→对爆发失效。
    SIG_CLIP_K = 2.0
    # 创新均值 EMA 记忆 = ACC_TAU_L·L̂ (无量纲设计选择, 与 FF 撤回/恢复同
    # 一标定时间尺度; 须 >> CUSUM 告警延迟, << 场景匀加速段时长)
    ACC_TAU_L = 4.0
    # 重建旗标最短保持 = RB_HOLD_N·(T/β) — α-β 速度估计时间常数的倍数
    # (无量纲; 1.5 → 保持期末 v̂ 残差 e^-1.5 ≈ 22%; 太短 → 重建残差灌入
    # ȳ 误读成加速度, 太长 → â 延迟介入)
    RB_HOLD_N = 1.5
    # 自身加速度活动门限 θ_a = max_v/(OW_ACTIV_K·L̂): "OW_ACTIV_K 个相信
    # 延迟窗内走完速度帽"的自身加速度算剧烈 (无量纲, 随标定缩放)。依据:
    # 伪创新 ∝ a_own·Δ 与真签名 ∝ a_t·T²/β 的可分性来自物理对齐。六次幂
    # 形状 = 平滑陡降无抖振。
    OW_ACTIV_K = 2.0
    # â 显著性门限 SNR 倍数 (无量纲设计选择, 测试组选定): 地板 =
    # SNR·σ_noise·sqrt(ρ/(2−ρ)), σ_noise² = m₂ − ȳ² (二阶矩精确分解)。
    # 依据见模块 docstring "显著性地板" — 盖过设计失配带下相关 dither
    # 噪声驱动的均值游走 (3σ白×相关膨胀≈8-10), 取全档逐位持平的最小整档。
    ACC_SNR = 10.0

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
        self._dbg = {}

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
        self.sig2x = self.sig2y = 1.0    # CUSUM 尺度: 原始创新二阶矩 (px²)
        self.sig2rx = self.sig2ry = 1.0  # â 传感器尺度: 清洗创新二阶矩 (px²)
        self.csx = self.csy = 0.0        # CUSUM 状态 (σ 单位)
        self._w_state = 0.0              # 信任度 0..1 (1=信任, FF 无门控)
        self._w_inst = 0.0               # CUSUM 告警电平
        self.ybar_x = self.ybar_y = 0.0  # 创新均值 EMA (px)
        self.ax_e = self.ay_e = 0.0      # 加速度估计 (px/ms²)
        self.reb_x = self.reb_y = True   # 重建抑制旗标 (v̂ 自 0 重建期)
        self.reb_until_x = self.reb_until_y = -1e9  # 旗标最短保持期 (det 轴)
        self._last_T = self.DT0          # 最近一帧的更新周期/增益 (供 â 反演)
        self._last_alpha = self.alpha0
        self._last_beta = self.beta0
        self._rho = 1.0 - math.exp(-self.DT0 / (self.ACC_TAU_L * max(1.0, cfg.L)))
        self._accx = 0.0
        self._accy = 0.0
        self._aown_x = 0.0
        self._aown_y = 0.0
        self._dbg = {}

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
            self.sig2rx = self.sig2ry = 1.0
            self.csx = self.csy = 0.0
            self.ybar_x = self.ybar_y = 0.0
            self.ax_e = self.ay_e = 0.0
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
            self.sig2rx = self.sig2ry = 1.0
            self.ybar_x = self.ybar_y = 0.0
            self.ax_e = self.ay_e = 0.0
            self.reb_x = self.reb_y = True
            rb_hold = self.RB_HOLD_N * dt / max(
                1e-6, min(0.60, self.beta0 * dt / self.DT0))
            self.reb_until_x = self.reb_until_y = det.t + rb_hold
        else:
            r = dt / self.DT0
            alpha = min(0.90, self.alpha0 * r)
            beta_s = min(0.60, self.beta0 * r)
            # CUSUM 尺度 σ̂: 原始创新二阶矩 EMA (ff_pi 原语义, 跨 JUMP 持续)
            self.sig2x += beta_s * (inx * inx - self.sig2x)
            self.sig2y += beta_s * (iny * iny - self.sig2y)
            sx = max(math.sqrt(self.sig2x), 1e-6)
            sy = max(math.sqrt(self.sig2y), 1e-6)
            # â 传感器稳健尺度 σ̂_r: 清洗创新 + Huber 截断的二阶矩 EMA —
            # 只喂 â 显著性地板与重建旗标判定, 不影响 CUSUM。
            srx = max(math.sqrt(self.sig2rx), 1e-6)
            sry = max(math.sqrt(self.sig2ry), 1e-6)
            # 传感器输入清洗 — 减掉自身已知的 Lc 过补偿伪迹 (matched 下
            # 恰好还原真实目标创新; 失配残留 ∝ Δ·a_own, 瞬态成对, 门控吸收)
            c0n = self.ch.at(det.t - cfg.L - dt)
            c1n = self.ch.at(det.t - cfg.L)
            inx_c = inx - cfg.s * ((c1[0] - c0[0]) - (c1n[0] - c0n[0]))
            iny_c = iny - cfg.s * ((c1[1] - c0[1]) - (c1n[1] - c0n[1]))
            clx = min(max(inx_c, -self.SIG_CLIP_K * srx),
                      self.SIG_CLIP_K * srx)
            cly = min(max(iny_c, -self.SIG_CLIP_K * sry),
                      self.SIG_CLIP_K * sry)
            self.sig2rx += beta_s * (clx * clx - self.sig2rx)
            self.sig2ry += beta_s * (cly * cly - self.sig2ry)
            # 自身加速度活动门 (伪创新 ∝ a_own·Δ; 门与伪迹物理对齐)
            w_own = max(1.0, cfg.L)
            st0 = self.ch.at(det.t)
            st1 = self.ch.at(det.t - w_own)
            st2 = self.ch.at(det.t - dt)
            st3 = self.ch.at(det.t - dt - w_own)
            vn_x = cfg.s * (st0[0] - st1[0]) / w_own
            vn_y = cfg.s * (st0[1] - st1[1]) / w_own
            vp_x = cfg.s * (st2[0] - st3[0]) / w_own
            vp_y = cfg.s * (st2[1] - st3[1]) / w_own
            th_a = self.max_v / (self.OW_ACTIV_K * max(1.0, cfg.L))
            aown_x = (vn_x - vp_x) / dt
            aown_y = (vn_y - vp_y) / dt
            ga_own_x = 1.0 / (1.0 + (aown_x / th_a) ** 6)
            ga_own_y = 1.0 / (1.0 + (aown_y / th_a) ** 6)
            self._aown_x = aown_x
            self._aown_y = aown_y
            # 重建旗标解除 = 重建时间常数下限 + 创新回典型水平 (用 σ̂_r)
            if (self.reb_x and det.t >= self.reb_until_x
                    and abs(inx) <= self.SIG_CLIP_K * srx):
                self.reb_x = False
            if (self.reb_y and det.t >= self.reb_until_y
                    and abs(iny) <= self.SIG_CLIP_K * sry):
                self.reb_y = False
            # 创新均值 EMA — 统一原则"无新鲜证据 → 以自然速率向零衰减;
            # 有证据 → 按 (自身活动门 × CUSUM 矛盾门) 缩放进入"
            rho = 1.0 - math.exp(-dt / (self.ACC_TAU_L * max(1.0, cfg.L)))
            self._rho = rho
            if self.reb_x:
                self.ybar_x -= rho * self.ybar_x
            else:
                g_x = ga_own_x * (1.0 - min(1.0, self.csx / self.CUSUM_H))
                self.ybar_x += rho * (g_x * clx - self.ybar_x)
            if self.reb_y:
                self.ybar_y -= rho * self.ybar_y
            else:
                g_y = ga_own_y * (1.0 - min(1.0, self.csy / self.CUSUM_H))
                self.ybar_y += rho * (g_y * cly - self.ybar_y)
            # 双向 CUSUM, 只累计与 v̂ 矛盾方向的创新 (σ 归一, ff_pi 原语义):
            #   矛盾 = 创新方向与 v̂ 相反 — 急停/变向的签名。
            #   v̂≈0 时不累计 (无可矛盾); 单帧封顶 C 拒单帧踢脚。
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
                self.reb_x = True
                self.reb_until_x = det.t + self.RB_HOLD_N * dt / beta_s
            if self.csy >= self.CUSUM_H and self.fvy != 0.0:
                self.fvy = 0.0
                self.csy = 0.0
                self.reb_y = True
                self.reb_until_y = det.t + self.RB_HOLD_N * dt / beta_s
            self.fx = px_pred + alpha * inx
            self.fy = py_pred + alpha * iny
            self.fvx += (beta_s / dt) * inx
            self.fvy += (beta_s / dt) * iny
            # â = ȳ·β/T² (创新均值自洽反演), 显著性硬门限 + 限幅; 重建期无定义
            lim_a = self.max_v / (self.ACC_TAU_L * max(1.0, cfg.L))
            self.ax_e = self._accel_est(self.ybar_x, self.sig2rx,
                                        beta_s, dt, lim_a)
            self.ay_e = self._accel_est(self.ybar_y, self.sig2ry,
                                        beta_s, dt, lim_a)
            if self.reb_x:
                self.ax_e = 0.0
            if self.reb_y:
                self.ay_e = 0.0
            self._last_T = dt
            self._last_alpha = alpha
            self._last_beta = beta_s
            self._accx = accx
            self._accy = accy
        self.prev_det_t = det.t
        self.t_pub = det.t

    def _accel_est(self, ybar: float, m2: float, beta_s: float,
                   dt: float, lim_a: float) -> float:
        """由创新均值反演加速度, 带显著性硬门限 (见 ACC_SNR 注释)。
        硬门限 (非软截断) 的帧率一致性: 超地板时 â = ȳ·β/T² 在任何帧率下
        都精确反演 a (ȳ = aT²/β 与 β/T² 自消), 软截断的通过比例随帧率漂移。"""
        sig_n = math.sqrt(max(1e-6, m2 - ybar * ybar))
        floor = self.ACC_SNR * sig_n * math.sqrt(self._rho / (2.0 - self._rho))
        eff = ybar if abs(ybar) > floor else 0.0
        return min(max(eff * beta_s / (dt * dt), -lim_a), lim_a)

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
            # 加速度偏差补偿: 速度修正 ε = â·T·(α/β − ½) (α-β 结构滞后),
            # 位置外推加 ½â·W²; FF 用 v̂+ε (type-2 精确开环指令的加速版)
            b = max(self._last_beta, 1e-9)
            eps_x = self.ax_e * self._last_T * (self._last_alpha / b - 0.5)
            eps_y = self.ay_e * self._last_T * (self._last_alpha / b - 0.5)
            vff_x = self.fvx + eps_x
            vff_y = self.fvy + eps_y
            w_win = age + Lc
            ex = (self.fx + vff_x * w_win
                  + 0.5 * self.ax_e * w_win * w_win
                  - cfg.s * (cn[0] - cp[0]))
            ey = (self.fy + vff_y * w_win
                  + 0.5 * self.ay_e * w_win * w_win
                  - cfg.s * (cn[1] - cp[1]))
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
            vx_u += ff_eff * vff_x
            vy_u += ff_eff * vff_y

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
            r = 0.0
            gate = 0.0
            ff_eff = 0.0
            ex = ey = 0.0
            w_state = self._w_state

        self.ch.add(t, cx, cy)
        self._dbg = {
            "ex": ex, "ey": ey, "r": r,
            "fvx": self.fvx, "fvy": self.fvy,
            "ax_e": self.ax_e, "ay_e": self.ay_e,
            "ybar_x": self.ybar_x, "ybar_y": self.ybar_y,
            "fx": self.fx, "fy": self.fy,
            "int_x": self.int_x, "int_y": self.int_y,
            "csx": self.csx, "csy": self.csy,
            "sigx": math.sqrt(self.sig2x), "sigy": math.sqrt(self.sig2y),
            "sigrx": math.sqrt(self.sig2rx), "sigry": math.sqrt(self.sig2ry),
            "w_state": w_state, "w_inst": self._w_inst,
            "ff_eff": ff_eff, "gate": gate,
            "age": age, "accx": self._accx, "accy": self._accy,
            "aown_x": self._aown_x, "aown_y": self._aown_y,
            "reb_x": 1.0 if self.reb_x else 0.0,
            "reb_y": 1.0 if self.reb_y else 0.0,
        }
        return cx, cy

    def debug(self) -> dict:
        return dict(self._dbg)
