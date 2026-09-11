"""arena/laws/pi_guard.py — 极点配置 PI (Smith 预测, PM 导出带宽) + 方向矛盾
CUSUM 速度记忆证据重置。无前馈的 pi_pm 结构, 加上 ff_pi 同款的矛盾检测器,
但重置对象是本律自己的两类速度记忆。

## 算法原理

控制结构 (增益全部由标定延迟 L̂ 导出, 无手调参数):
    ê = f + v̂·(age + L̂·l_comp) − s·Σcounts(in flight)     // Smith 预测误差
    wn = (90°−PM)·π/180 / L̂,  Kp = 2ζ·wn,  Ki = wn²        // 极点配置 (ζ=1)
    gate = i_gate/(i_gate+|ê|)                              // I 距离门控
    v = Kp·ê + Ki·∫ê·gate·h                                 // PI (无前馈)

速度记忆证据重置 (本律的核心, ff_pi 机制在无 FF 结构上的对应物):

  本律有两类"目标速度记忆":
    a) 显式: α-β 滤波速度 v̂ — 进 Smith 外推项 v̂·(age+L̂·l_comp);
    b) 隐性: I 积分器 — 稳态跟踪时 Ki·∫ê ≈ v_t (type-2 零拖尾的来源),
       即积分器里存的就是目标速度。
  目标模型破缺 (急停/折返/蹬墙跳 2V 反向) 时两者同时变幽灵: v̂ 污染 Smith
  外推遮蔽真实误差 (回拉发面), I 继续命令旧速度 (实测 Ki·int_x 停后 600ms
  仍 ≈0.64 px/ms, 几乎不衰减 — 过冲后 58→18px 慢尾 600ms 的主源)。
  ff_pi 给显式 FF 配了 CUSUM 证据撤回; 本律无 FF, 对应物 = 同一检测器同时
  重置两类记忆: 告警 → 该轴 v̂=0 (Smith 去遮蔽) 且 ∫=0 (旧速度作废),
  环路回到阶跃响应的初始条件: P 全程看见真实误差 (回拉 sharp), I 从 0 按
  证据重建。这是"速度记忆归零重拉"原理, 不是新增估计器。

  检测器与 ff_pi 逐字同款 (Page 序贯变化检测, σ 归一, σ 在线自标定):
    K = 0.5σ  漂移: 零均值噪声的矛盾方向期望 E[max(0,−z)]=0.399σ < K,
              净漂移为负 → 稳态不积累, 不误触发 (type-2 锁定不受扰);
    C = 3σ    单帧增量上限: 拒后坐力式单帧踢脚;
    H = 9σ    告警门限: 同号持续 ~2-3 帧触发 (~17-25ms @120fps)。
    只累计与 v̂ 矛盾方向的创新 (急停/变向签名); v̂≈0 时不累计 (无可矛盾,
    也恰好豁免自身加速瞬态的失配伪创新); σ̂ = 创新二阶矩 EMA, 噪声越大门
    自动越宽, 无绝对 px 常数。加速目标 (accel/approach) 的创新与 v̂ 同向,
    永不触发 → 跟踪类场景零扰动 (实测 accel 16.15→16.14, 噪声级差异)。
    失配下 Smith 窗错位伪创新 ∝ 自身加速度×Δ, 瞬态成对且被 σ̂ EMA 吸收
    (CUSUM 在环路自激时"致盲" = 恰好不误重置, 对带宽稳定是有效阻尼)。

## 参数表 (默认 / 来源 / 意义)

pm_deg = 60.0   [无量纲设计选择]
    相位裕度, wn=(90−pm_deg)·π/180/L̂。继承 pi_pm 的保守设计点 (PM=60 留
    足裕度吸收标定误差/Smith 残差/滤波滞后)。↑更稳更慢; ↓换速度是下策
    (PM<50 属 rejected path)。本律不改此值。

zeta = 1.0   [无量纲设计选择]
    阻尼比, 临界阻尼, 固定。

l_comp = 1.1   [EMPIRICAL: 方向有原理, 数值试出]
    Smith 补偿系数 (>1 过补偿偏向欠估计延迟的危险侧)。继承 pi_pm。

i_gate = 8.0 px   [EMPIRICAL: 划分拉枪/跟踪职责]
    I 距离衰减。继承 pi_pm。注意它同时放大加速目标的结构性滞后
    (稳态 e 解 e·ig(e)·Ki = a), 无 FF 结构下不可消除 (rejected: 加
    type-2 FF = 变成 ff_pi)。

i_frac = 1.0   [原理: 可跟踪最大目标速度 = i_frac×max_v]
    积分限幅 = i_frac×max_v/Ki。

alpha0 = 0.50 / beta0 = 0.04 / beta_exp = 1.0   [EMPIRICAL: α-β 增益]
    估计器增益, 按 dt 缩放 (帧率无关)。继承 pi_pm。beta0=0.04 保持
    <0.05 (失配自激悬崖)。

CUSUM_K/C/H = 0.5/3.0/9.0 (σ 倍数)   [原理+测试组: 与 ff_pi 逐字相同]
    无量纲, 随在线 σ̂ 自标定, 跨设备自适应。

## debug() 字段语义 (px / px/ms / σ)
  fx, fy        α-β 位置估计
  fvx, fvy      α-β 速度估计 (告警重置后从 0 重建)
  int_x, int_y  I 积分器状态 (px·ms; 告警重置后从 0 重建)
  csx, csy      CUSUM 状态 S (σ 单位)
  sigx, sigy    CUSUM 尺度 σ̂ = sqrt(创新二阶矩 EMA) (px)
  ex, ey        Smith 汇装误差 ê (驱动 P/I)
  age           当前拍距最新检测 ms

## 评测 (arena 实测, 默认种子; 对比对象 pi_pm 基线)
    matched 199.11→190.65 (maneuver 23.96→20.22, const_vel 0.69→0.67,
    accel/step/relock 逐位不动); 失配扫描 L30-70 全档改善
    (97/97/106/101/128 → 86/86/97/90/117, worst 128.02→116.69);
    宽延迟 L20-80 全档改善零发散 (L80 143.96→139.59); s 失配全档改善
    (122.80→113.58); relock 483.3ms/3.16px 逐位相同; fps_eval clean
    RMSE 27.45→24.18, 事件过冲 mean 25.0→23.3, worst 86.3→78.6;
    OVERALL 164.85→160.21。60/120fps 差 0.6%→3.3% (两个 composite 本身
    都改善; 告警延迟按帧计, 60fps 时 ms 延迟翻倍 → 120fps 获益更大)。
    已知代价 (结构性可解释): 软着陆类事件重置后重跑阶跃响应的回弹略升
    (jump_land_stop over 1.8→4.3px, bhop worst 27.9→33.2, slide over
    16.4→20.3 — 小绝对量, 换取 stop/friction/reverse 类过冲 −25~40%)。
    holdout (seeds 4,5,6): matched 191.52→181.05, maneuver 21.60→18.03,
    失配全档改善 (worst 124.97→116.46), 无发散; OVERALL +0.3% (60fps
    composite 改善少, fpsΔ 元项膨胀, 两个 composite 本身均改善)。

设计取舍: 保持 pi_pm 的"无前馈、保守带宽、全带不发散"定位, 把它缺的
模型破缺处理补上 (证据重置而非前馈)。与 ff_pi 的分工: ff_pi 用前馈换
跟踪/收敛, 用信任度管 FF; 本律零前馈零信任滤波, 结构最简, 事件过冲与
帧率无关性保持全库第一梯队。加速拖尾 a/(Ki·ig) 是无 FF 结构的原理性
极限, 本律不掩盖它。
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
from arena.core import Observation, LawConfig
from arena.laws.base import CountsHist, Law, register


@register("pi_guard")
class PiGuardLaw(Law):
    DT0 = 1000.0 / 120.0
    JUMP_GATE = 100.0
    STALE = 200.0
    # CUSUM 参数 (均为 σ 倍数, 无量纲, 与 ff_pi 逐字相同):
    # K 漂移 (0.5 > E[max(0,−z)]=0.399 → 零均值噪声净漂移为负, 稳态不误触发)
    # C 单帧增量上限 (拒单帧踢脚) / H 告警门限 (~2-3 帧持续矛盾触发)
    CUSUM_K = 0.5
    CUSUM_C = 3.0
    CUSUM_H = 9.0

    def __init__(self, zeta=1.0, pm_deg=60.0, max_v=1.5, i_gate=8.0, i_frac=1.0,
                 alpha0=0.50, beta0=0.04, l_comp=1.1, beta_exp=1.0):
        self.zeta = zeta            # 阻尼比 (无量纲设计选择, 临界阻尼=1)
        self.pm_deg = pm_deg        # 相位裕度 (无量纲设计选择), 决定导出带宽
        self._max_v = max_v
        self.i_gate = i_gate        # I 距离衰减 (EMPIRICAL: 划分拉枪/跟踪)
        self.i_frac = i_frac        # 积分限幅 = i_frac×max_v/Ki
        self.alpha0 = alpha0        # @DT0 位置修正; 按 dt 缩放
        self.beta0 = beta0          # @DT0 速度修正; 按 dt^beta_exp 缩放
        self.l_comp = l_comp        # Smith 补偿系数 (>1 偏安全侧)
        self.beta_exp = beta_exp    # β 的 dt 缩放指数 (1=帧率无关)

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
        self.sig2x = self.sig2y = 1.0   # 创新方差在线估计 (px², 自标定)
        self.csx = self.csy = 0.0       # CUSUM 状态 (σ 单位)
        self._dbg = {}

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
            beta = min(0.60, self.beta0 * r ** self.beta_exp)
            # CUSUM 尺度 σ̂: 原始创新二阶矩 EMA (ff_pi 原语义, 跨 JUMP 持续;
            # 失配伪创新自动撑宽告警门 = 不误重置的有效阻尼)
            self.sig2x += beta * (inx * inx - self.sig2x)
            self.sig2y += beta * (iny * iny - self.sig2y)
            sx = max(math.sqrt(self.sig2x), 1e-6)
            sy = max(math.sqrt(self.sig2y), 1e-6)
            # 双向 CUSUM, 只累计与 v̂ 矛盾方向的创新 (σ 归一):
            #   矛盾 = 创新方向与 v̂ 相反 — 急停/变向的签名。
            #   v̂≈0 时不累计 (无可矛盾, 豁免自身加速瞬态); 单帧封顶 C。
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
            # 告警 → 该轴速度记忆全部归零: v̂=0 (Smith 去遮蔽) 且
            # ∫=0 (旧速度作废) — 环路重跑阶跃响应, I 从 0 按证据重建
            if self.csx >= self.CUSUM_H and self.fvx != 0.0:
                self.fvx = 0.0
                self.int_x = 0.0
                self.csx = 0.0
            if self.csy >= self.CUSUM_H and self.fvy != 0.0:
                self.fvy = 0.0
                self.int_y = 0.0
                self.csy = 0.0
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
        ex = ey = 0.0
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
        self._dbg = {
            "fx": self.fx, "fy": self.fy,
            "fvx": self.fvx, "fvy": self.fvy,
            "int_x": self.int_x, "int_y": self.int_y,
            "csx": self.csx, "csy": self.csy,
            "sigx": math.sqrt(self.sig2x), "sigy": math.sqrt(self.sig2y),
            "ex": ex, "ey": ey, "age": age,
        }
        return cx, cy

    def debug(self) -> dict:
        return dict(self._dbg)
