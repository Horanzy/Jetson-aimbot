"""arena/laws/ballistic_ff.py — 时间最优两段式自瞄 + type-2 速度前馈
(弹道段开环 + 收敛段三通道分责: P 快通道 / v̂ 前馈 / I 低速残差;
ghost-v̂ 三层防护: att 信任度 + 当拍惊讶门 + CUSUM 归零)。

算法原理
========
延迟 L 是硬约束: 现在发的指令 L 之后才被看见, 时间最优结构被强制为两段。

1. 弹道段 (开环, 延迟免疫): 预测误差 |ê| 大于制动距离时, 以 Vmax 全速朝预测
   目标甩枪。此段指令只取决于 ê 的方向 (饱和), 与幅值无关 → 对延迟与估计
   误差免疫。
2. 收敛段: 临界阻尼 PI + type-2 速度前馈无过冲收尾。

制动点 = 物理值 Vmax·L (brake_factor=1.0):
   以 Vmax 接近目标时, 现在发的指令 L 后才生效, 那 L 内还会滑行 Vmax·L。所以
   必须在距目标约 Vmax·L 处开始制动——提前一个延迟长度。这是物理值, 不是调出
   来的。(brake_factor<1 = 制动更晚 = 拿延迟裕度换 arena 速度分, 是被拒绝的
   过拟合。)

收敛段三通道分责 (对纯 PI 两段式的核心方法差异):
   P  Kp·ê          位置快通道: flick 与瞬态修正。
   FF v̂ (type-2)    速度通道: 被积对象是纯积分器, v̂ 是匀速目标零拖尾的精确
                    开环指令 (gain=1 由对象模型导出, 非调参)。匀速/加速目标
                    的拖尾由它扛, 不再逼 I 逐帧学习。
   I  Ki·∫ê·gate    低速残差通道: 只修正 v̂ 的偏置 (如目标加速度下的估计滞后)。
   I 充电非对称门:  充电 (ê·int ≥ 0) 乘以速度余量 (1−|v_pi|/Vmax), 放电不设门。
      依据: 接近段指令贴速度帽, I 在那里记账只存"虚假接近动力学"——误差过零
      后回路被迫停在 e ≈ −int·Ki/Kp 的放电平台上 (放电速率 wn/2·ig, 数百 ms),
      这是纯 PI step 拖尾 (settle ≫ first) 的根源。门掉充电即消除平台, 而 I
      的本职 (低速残差) 不受影响 (放电期零均值噪声引起的均衡点偏移 ~0.04px)。

FF 的 ghost-v̂ 防护 (三层, 辖区不相交):
   1. att 自身加速度信任度: Smith 窗错位幻影 ∝ (L_true−L̂)·a_own —— 失配下
      自身加减速的指令瞬态被窗错位转成假创新。标定只保证 |Δ| ≲ 0.5·L̂
      (d_tol_frac, 设计值), 幻影上界 junk = s·0.5·L̂·|a_own|·fdt (每帧)。
      att = σ̂/(σ̂+junk) (EMA τ=2fdt): 瞬态期 (自身加减速) FF 按信任度撤出 —
      瞬态本就由 P/弹道段主导; 稳态跟踪 (a_own≈0) 满强度。junk 与 fdt 同阶
      缩放 → 门控帧率对称 (实测 fpsΔ 1%)。a_own 由 counts 历史差分 (被控
      对象即准星的真实速度): v_own 基线 2fdt, 差分间隔 4fdt (counts 量化噪声压到
      0.001 px/ms² 量级, 低于真实目标加速度)。
   2. ff_sur 当拍惊讶门 (只在 att ≥ 0.5 时武装): 单帧创新超 2σ = 模型破缺的
      当拍证据 (急停/落地/折返)。FF 是软撤出 (乘性, 无重置踢脚), 可以当拍
      执行; CUSUM 累计告警的防误报延迟 ~3-4 帧正是 ghost-FF 推力下界, 惊讶
      门把跳跃落地类事件的 ghost 推力缩到 ~1 帧。
   3. CUSUM 方向矛盾归零 (Page 序贯检测, σ 归一): 双向 CUSUM 只累计与 v̂
      矛盾方向的创新 (σ 归一, K/C/H = 漂移/单帧封顶/告警门限, 均为 σ 倍数),
      告警即该轴 v̂ 归零 (位置保留) — 环路重跑评测组最优的阶跃响应。
   σ̂ (创新方差 EMA) 在 att < 0.5 时冻结: junk 期把大创新计入 σ̂ 会抬高噪声
   地板 (σ̂ 致盲), 使 att/ff_sur/CUSUM 同时失去标尺。

估计器: alpha-beta (按 dt/DT0 归一 → 帧率无关), Smith 前推 (age+L) 并扣除
在途 counts, 前推时域 = 相信的物理延迟 L, 不做过预测 (过预测配高速度增益
会在 L_true=30 制造稳定悬崖, 实测结论)。α0=0.70: 幻影稳态误差
= (1−α)/α·junk — α 是幻影放大器, 传感器噪声 (~0.5px) 远小于失配幻影, 位置
通道偏向信任观测。β0=0.08: v̂ 噪声经 FF 直通指令, 又决定加速目标的 v̂ 滞后,
二者折中。

边界层 = Vmax·L 的比例 (bound_frac): 制动点外侧薄带内指令从全速弹道平滑混到
收敛值, 防噪声下 bang-bang 抖动。厚度按 Vmax·L 缩放 (帧率/延迟无关)。

参数表 (默认 / 来源)
====================
| 参数         | 默认 | 来源                                                          |
|--------------|------|---------------------------------------------------------------|
| pm_deg       | 60   | 原理导出: 目标相位裕度, wn=(90−PM)π/180/L; 60 = 失配带实测保护点 (PM=50 提速 matched 但在 L30/L70 触发幻影极限环, 实测否决) |
| zeta         | 1.0  | 原理导出: 临界阻尼无过冲 (Kp=2ζωn, Ki=ωn²)                    |
| brake_factor | 1.0  | 物理值: 制动点=Vmax·L (一个延迟长度); <1 是被拒绝的过拟合      |
| bound_frac   | 0.20 | 经验(缩放): 边界层=0.2·Vmax·L; 0.15-0.25 等效(不敏感)         |
| i_gate_frac  | 0.20 | 经验(缩放): I 距离门标尺 = 0.2·Vmax·L                         |
| vmax_frac    | 0.95 | 经验(小): 留 5% 速度余量防量化截断                            |
| alpha0       | 0.70 | 设计选择: 幻影放大器 (1−α)/α = 0.43; 0.50 (0.67) 时 L70 边缘幻影自激, 实测否决 |
| beta0        | 0.08 | 继承 ballistic; accel 滞后与 FF 直通噪声的折中 (0.05/0.12 双向实测变差) |
| ff_gain      | 1.0  | 原理: type-2 精确前馈                                         |
| d_tol_frac   | 0.5  | 设计值: 标定容差 |L_true−L̂| ≲ 0.5·L̂, 幻影上界的 Δ            |
| CUSUM K/C/H  | .5/3/9σ | 原理: σ 倍数, 无量纲 (K 漂移/C 单帧封顶/H 告警)          |
| ff_sur 阈值  | 2σ   | 设计: 单帧破缺证据门限 (σ 倍数)                               |
| att EMA τ    | 2fdt | 设计: 门控平滑时间尺度 (帧率对称)                             |

实机调参
========
1. 标定 s,L。wn 自动随 L 缩放, 无需手调 τ。
2. 失配振荡 → 升 pm_deg (降 wn)。
3. 拉枪慢 → 升 vmax_frac (→1.0); 制动点已是物理极限, 不要降 brake_factor 到 <1。
4. 机动目标拖尾 → 先查检测噪声 (σ̂); 不要动 i_gate_frac 与非对称门 (它们已把
   accel/step 的历史矛盾解耦)。

评测 (arena 实测, 默认种子): matched 125.8 (step settle 167ms / 过冲 2.75px,
accel rmse 3.9px, maneuver rmse 18.8px); 失配扫描 L30-70 全过 (最坏 135.0);
宽延迟 L20-70 全过; s0.70-1.30 全过; relock 219ms; fpsΔ 2.3%; FPS 行为组
RMSE 21.6px / 事件过冲 mean 29.0px worst 74.0px。

debug() 字段 (trace 采集为 dbg_* 列, 无副作用, 评测路径不调用):
    ex/ey      Smith 预测误差 ê (px)
    r          |ê| (px)
    vx/vy      本拍合成指令速度 (px/ms, FF 后、限幅前)
    fvx/fvy    alpha-beta 目标速度估计 v̂ (px/ms)
    int_x/y    积分器状态 (Ki·int 参与指令)
    mode       0=弹道段 1=边界层混合 2=收敛段PI 3=无帧/STALE
    age        距最近一帧的 ms
    cgx/cgy    本拍 I 充电系数 (0..1, 非对称门后)
    csx/csy    CUSUM 状态 (σ 单位)
    attx/atty  FF 自身加速度信任度 (0..1)
    ffsx/ffsy  FF 当拍惊讶门 (0..1)
    aox/aoy    自身加速度估计 (px/ms²)
"""
from __future__ import annotations
import math
from typing import Optional, Tuple
from arena.core import Observation, LawConfig
from arena.laws.base import CountsHist, Law, register


@register("ballistic_ff")
class BallisticFFLaw(Law):
    DT0 = 1000.0 / 120.0
    JUMP_GATE = 100.0
    STALE = 200.0
    # CUSUM 参数 (均为 σ 倍数, 无量纲): K 漂移 / C 单帧增量上限 / H 告警
    CUSUM_K = 0.5
    CUSUM_C = 3.0
    CUSUM_H = 9.0
    FF_SUR_SIGMA = 2.0   # 当拍惊讶门限 (σ 倍数)
    ATT_SIGMA_ARM = 0.5  # 惊讶门武装所需 att 下限

    def __init__(self, pm_deg=60.0, zeta=1.0, brake_factor=1.0,
                 bound_frac=0.20, i_gate_frac=0.20, vmax_frac=0.95,
                 alpha0=0.70, beta0=0.08, ff_gain=1.0, d_tol_frac=0.5):
        self._pm_deg = pm_deg
        self._zeta = zeta
        self._brake_factor = brake_factor
        self._bound_frac = bound_frac
        self._i_gate_frac = i_gate_frac
        self._vmax_frac = vmax_frac
        self._alpha0 = alpha0
        self._beta0 = beta0
        self._ff_gain = ff_gain
        self._d_tol_frac = d_tol_frac

    def reset(self, cfg: LawConfig):
        self.cfg = cfg
        self.vmax = cfg.max_v * self._vmax_frac
        L = max(1.0, cfg.L)
        wn = (90.0 - self._pm_deg) * math.pi / 180.0 / L
        self.kp = 2.0 * self._zeta * wn
        self.ki = wn * wn
        self.i_lim = self.vmax / max(self.ki, 1e-9)
        self.brake_dist = self.vmax * L * self._brake_factor
        self.boundary = self._bound_frac * self.vmax * L
        self.i_gate = self._i_gate_frac * self.vmax * L
        self.ch = CountsHist()
        self.filt = False
        self.fx = self.fy = self.fvx = self.fvy = 0.0
        self.prev_det_t = None
        self.t_pub = -1e9
        self.int_x = self.int_y = 0.0
        self.rem_x = self.rem_y = 0.0
        self.sig2x = self.sig2y = 1.0   # 创新方差在线估计 (px², 自标定)
        self.csx = self.csy = 0.0       # CUSUM 状态 (σ 单位)
        self.att_x = self.att_y = 1.0   # FF 自身加速度信任度
        self.ffsx = self.ffsy = 1.0     # FF 当拍惊讶门
        self.aox = self.aoy = 0.0       # 自身加速度估计 (px/ms²)
        self.mode = 3.0
        self.dex = self.dey = self.dr = 0.0
        self.age = 1e9
        self.cgx = self.cgy = 0.0
        self.rem_debug_vx = 0.0
        self.rem_debug_vy = 0.0

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
        c0 = self.ch.at(det.t - cfg.L - dt)
        c1 = self.ch.at(det.t - cfg.L)
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
            alpha = min(0.90, self._alpha0 * r)
            beta = min(0.60, self._beta0 * r)
            # 当拍惊讶门: 单帧创新超 FF_SUR_SIGMA·σ = 模型破缺的当拍证据
            sx0 = max(math.sqrt(self.sig2x), 1e-6)
            sy0 = max(math.sqrt(self.sig2y), 1e-6)
            self.ffsx = sx0 / (sx0 + max(0.0, abs(inx) - self.FF_SUR_SIGMA * sx0))
            self.ffsy = sy0 / (sy0 + max(0.0, abs(iny) - self.FF_SUR_SIGMA * sy0))
            # σ̂ 只在估计器可信期更新 (att 冻结防 junk 抬高噪声地板)
            if self.att_x >= self.ATT_SIGMA_ARM:
                self.sig2x += beta * (inx * inx - self.sig2x)
            if self.att_y >= self.ATT_SIGMA_ARM:
                self.sig2y += beta * (iny * iny - self.sig2y)
            # 双向 CUSUM, 只累计与 v̂ 矛盾方向的创新 (σ 归一):
            #   矛盾 = 创新方向与 v̂ 相反 — 急停/变向的签名。
            #   v̂≈0 时不累计 (无可矛盾); 单帧封顶 C 拒单帧踢脚。
            sx = max(math.sqrt(self.sig2x), 1e-6)
            sy = max(math.sqrt(self.sig2y), 1e-6)
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
            if self.csy >= self.CUSUM_H and self.fvy != 0.0:
                self.fvy = 0.0
                self.csy = 0.0
            self.fx = px_pred + alpha * inx
            self.fy = py_pred + alpha * iny
            self.fvx += (beta / dt) * inx
            self.fvy += (beta / dt) * iny
        self.prev_det_t = det.t
        self.t_pub = det.t

    def _a_own_update(self):
        """自身加速度 (px/ms², 每轴) + FF 幻影信任度 att。

        Smith 窗错位幻影 ∝ (L_true−L̂)·a_own, 标定只保证 |Δ| ≲ 0.5·L̂,
        幻影上界 junk = s·0.5·L̂·|a_own|·fdt (每帧)。a_own 由 counts 历史
        差分 (被控对象（准星）真实速度): v_own 基线 2fdt, 差分间隔 4fdt — counts 量化
        噪声压到低于真实目标加速度的量级。"""
        cfg = self.cfg
        fdt = cfg.frame_dt if cfg.frame_dt > 0 else self.DT0
        now = self.ch.t[-1] if self.ch.t else 0.0
        s = max(0.05, min(20.0, cfg.s))
        c0 = self.ch.at(now - 2.0 * fdt)
        c1 = self.ch.at(now - 6.0 * fdt)
        v_own_x = s * (self.ch.cumx - c0[0]) / (2.0 * fdt)
        v_own_y = s * (self.ch.cumy - c0[1]) / (2.0 * fdt)
        v_pre_x = s * (c0[0] - c1[0]) / (4.0 * fdt)
        v_pre_y = s * (c0[1] - c1[1]) / (4.0 * fdt)
        self.aox = (v_own_x - v_pre_x) / (4.0 * fdt)
        self.aoy = (v_own_y - v_pre_y) / (4.0 * fdt)
        d_tol = self._d_tol_frac * max(1.0, cfg.L)
        jx = s * d_tol * abs(self.aox) * fdt
        jy = s * d_tol * abs(self.aoy) * fdt
        sx = max(math.sqrt(self.sig2x), 1e-6)
        sy = max(math.sqrt(self.sig2y), 1e-6)
        ax_raw = sx / (sx + jx) if jx > 0.0 else 1.0
        ay_raw = sy / (sy + jy) if jy > 0.0 else 1.0
        w_a = 1.0 - math.exp(-cfg.h / (2.0 * fdt))   # EMA τ = 2fdt
        self.att_x += w_a * (ax_raw - self.att_x)
        self.att_y += w_a * (ay_raw - self.att_y)

    def step(self, t: float, obs: Optional[Observation]) -> Tuple[int, int]:
        cfg = self.cfg
        if obs is not None and obs.new:
            self._update_filter(obs)

        cx = cy = 0
        age = t - self.t_pub
        if self.filt and age < self.STALE:
            self.age = age
            cp = self.ch.at(self.t_pub - cfg.L)
            cn = self.ch.cum()
            ex = self.fx + self.fvx * (age + cfg.L) - cfg.s * (cn[0] - cp[0])
            ey = self.fy + self.fvy * (age + cfg.L) - cfg.s * (cn[1] - cp[1])
            r = math.hypot(ex, ey)
            self.dex, self.dey, self.dr = ex, ey, r

            brake_outer = self.brake_dist + self.boundary
            if r > brake_outer:
                scale = self.vmax / r
                vx = ex * scale
                vy = ey * scale
                self.int_x = 0.0
                self.int_y = 0.0
                self.mode = 0.0
                self.cgx = self.cgy = 0.0
            else:
                vcx_u = self.kp * ex + self.ki * self.int_x
                vcy_u = self.kp * ey + self.ki * self.int_y
                if r > cfg.fov_radius:
                    self.int_x = self.int_y = 0.0
                else:
                    ig = self.i_gate / (self.i_gate + r)
                    wx = (vcx_u > self.vmax and ex > 0) or (vcx_u < -self.vmax and ex < 0)
                    wy = (vcy_u > self.vmax and ey > 0) or (vcy_u < -self.vmax and ey < 0)
                    # 非对称充放门: 充电 (ê·int ≥ 0) 乘以速度余量, 放电不设门
                    if not wx:
                        cgx = 1.0
                        if ex * self.int_x >= 0.0:
                            cgx = max(0.0, 1.0 - min(1.0, abs(vcx_u) / self.vmax))
                        self.int_x = max(-self.i_lim, min(self.i_lim,
                                                          self.int_x + ex * cfg.h * ig * cgx))
                        self.cgx = cgx
                    if not wy:
                        cgy = 1.0
                        if ey * self.int_y >= 0.0:
                            cgy = max(0.0, 1.0 - min(1.0, abs(vcy_u) / self.vmax))
                        self.int_y = max(-self.i_lim, min(self.i_lim,
                                                          self.int_y + ey * cfg.h * ig * cgy))
                        self.cgy = cgy
                vcx = max(-self.vmax, min(self.vmax, vcx_u))
                vcy = max(-self.vmax, min(self.vmax, vcy_u))
                if r > self.brake_dist:
                    b = (r - self.brake_dist) / self.boundary
                    bscale = self.vmax / r
                    vx = b * (ex * bscale) + (1.0 - b) * vcx
                    vy = b * (ey * bscale) + (1.0 - b) * vcy
                    self.mode = 1.0
                else:
                    vx = vcx
                    vy = vcy
                    self.mode = 2.0
                # type-2 速度前馈: att (幻影高危期撤出) × 惊讶门 (破缺当拍
                # 撤出, 仅 att≥0.5 武装) × 检测间隔衰减 (丢帧按 L 时间尺度退)。
                self._a_own_update()
                wsx = max(0.0, min(1.0, (self.att_x - self.ATT_SIGMA_ARM) * 2.0))
                wsy = max(0.0, min(1.0, (self.att_y - self.ATT_SIGMA_ARM) * 2.0))
                trx = self.att_x * (wsx * self.ffsx + (1.0 - wsx))
                tryy = self.att_y * (wsy * self.ffsy + (1.0 - wsy))
                frame_dt = cfg.frame_dt if cfg.frame_dt > 0 else self.DT0
                gap = 1.0 - max(0.0, min(1.0, (age - frame_dt) / max(1.0, cfg.L)))
                vx += self._ff_gain * trx * gap * self.fvx
                vy += self._ff_gain * tryy * gap * self.fvy
                vx = max(-self.vmax, min(self.vmax, vx))
                vy = max(-self.vmax, min(self.vmax, vy))

            s = max(0.05, min(20.0, cfg.s))
            self.rem_x += vx * cfg.h / s
            self.rem_y += vy * cfg.h / s
            cx, cy, self.rem_x, self.rem_y = self._counts(
                self.rem_x, self.rem_y, s, cfg.count_limit)
        else:
            self.rem_x = self.rem_y = 0.0
            self.int_x = self.int_y = 0.0
            self.mode = 3.0
            self.dex = self.dey = self.dr = 0.0
            self.age = 1e9
            self.cgx = self.cgy = 0.0
            vx = vy = 0.0

        self.rem_debug_vx, self.rem_debug_vy = vx, vy
        self.ch.add(t, cx, cy)
        return cx, cy

    def debug(self) -> dict:
        return {
            "ex": self.dex,
            "ey": self.dey,
            "r": self.dr,
            "vx": self.rem_debug_vx,
            "vy": self.rem_debug_vy,
            "fvx": self.fvx,
            "fvy": self.fvy,
            "int_x": self.int_x,
            "int_y": self.int_y,
            "mode": self.mode,
            "age": self.age,
            "cgx": self.cgx,
            "cgy": self.cgy,
            "csx": self.csx,
            "csy": self.csy,
            "attx": self.att_x,
            "atty": self.att_y,
            "ffsx": self.ffsx,
            "ffsy": self.ffsy,
            "aox": self.aox,
            "aoy": self.aoy,
        }
