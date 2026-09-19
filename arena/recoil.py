"""arena/recoil.py — 后坐力扰动 (相机被推动) + 后坐力场景族。

后坐力不是目标运动, 也不是律的输出 —— 它是**相机被外力推动**: 每开一枪, 瞄准轴
被踢起一个角度, 屏幕上表现为"目标视位置相对准星发生了位移", 律只能靠自己的输出
反向补偿 (人类靠提前匀速下拉)。因此它以外部扰动 (core.ArenaConfig.disturbance) 的
形式进入被控对象, 不进 law 的接口 —— 律看不到它, 只能从误差里发现它。

符号约定 (与 fps.py 的跳跃弧同一套屏幕坐标)
    屏幕 +y = 世界竖直向上 (fps.py 的跳跃弧 vz0>0 → _jy>0 → y 增大, 即 +y 向上)。
    枪口上跳把瞄准轴往上推 → 瞄准点 +y → 目标视位置相对准星向 −y 偏移, 即
    e_y = target.y − (cross.y + oy) 变小 (目标掉到准星下方) → **律必须下发负
    counts (向下压) 才能抵消**, 这正是人类压枪的方向 (向下拉)。

形态与标定 (每一项都是可复现的口径, 不是手感)
    参考距离 REF_DIST_M = 10m: 与 fps.py 的行为库、metrics.body_px 的投影换算
    同一套基准 (f≈960px @1080p/90°hFOV)。每发落差 kick_px 定义在 10m 处, 按
    1/d 折算到交战距离 (与 fps.py 的 _jump_target 同一条规则: 屏幕位移 ∝ 1/d)。
    注: 纯相机**角度**踢在屏幕上是与距离无关的 (px = f·θ), 与"相机被平移"的
    1/d 读法不同; 本库取 1/d 读法, 使 kick 与库里其它幅值参数同一量纲口径, 在
    参考距离 10m 处两种读法数值相等。哪一种符合真实游戏需要实测 (未验证)。
    落差档 KICK_GRID_PX = 6 / 20 / 60 px @10m —— 在 f≈960px 下折合 0.36°/1.2°/
    3.6° 的瞄准轴仰角, 覆盖"单发几乎看不出"到"单发把准星踢出 FOV 半径的 40%";
    换成每发间隔内的补偿速度 v = K·d_ref/d / interval, 网格跨 0.03 → 0.9 px/ms
    (速度帽 1.5 px/ms 的 2% → 60%)。
    射速档 RPM_GRID = 300 / 450 / 900 (发间隔 200 / 133.3 / 66.7 ms): 与标定
    延迟 L=50ms 的比值 4.0 / 2.7 / 1.3 —— 从"律有 4 个延迟的时间把每发压完"到
    "不到 1.3 个延迟就要面对下一发", 覆盖"每发独立压枪"到"压枪与下一发叠加"的
    转变。慢射是重点工况: 发间隔越长, 律越有把握在间隔的前一小段里把整发落差
    压完 → 剩下的时间指令速度归零, 画面呈"压一下、停住等下一发"的矩形脉冲;
    人类则是把同样的位移摊到整个间隔 (匀速), 完成后比例 ≈ 1。
    每发形态 rise_ms: 默认 0 = 瞬时阶跃 (枪械的 view-punch 在开火那一拍施加,
    可见的上升段是它自己与后坐恢复的合成, 不在单发模型里); >0 = 短促斜坡。
    recover_tau: 相机自动回正的时间常数 (0 = 不回正, 需要玩家/律自己拉回来)。
    单发形态对不对需要实测 (未验证), 两个参数都暴露给场景构造。

套件结构 (recoil_suite)
    时间轴各组合完全一致: 0-800ms 静置 (律先压上目标, fps.py 同类场景的事件也
    在 800ms) → 800-2400ms 射击段 (BURST_MS=1600ms) → 2400-3200ms 停火观察段。
    800 与 1600 都是三个发间隔的公倍数 (200·4=133.3·6=66.7·12=800), 因此三档
    射速的发数恰好是整数 8/12/24, 且射击段/停火段的物理时长逐档相同 —— 射速
    是唯一的自变量。
    13 个组合: 3 射速 × 3 落差 (连发) + 点射 (3 发一组) + 斜坡形态 +
    相机自动回正 + 近距档。
    目标在射击段静止: 本族的自变量只有后坐力通道, 目标运动由 standard/fps
    两族覆盖 (否则每发对齐的指标会被目标运动稀释, 无法归因)。
    近距档是 1/d 口径的自洽性检查: 10m 处 6px/发 折到 3m 就是 20px/发, 它与
    rc_300rpm_20px 的"每发落差"在屏幕上完全相同, 因此两者的平滑度指标应当逐位
    一致, 只有 on_body 不同 (命中带随距离展宽: 3m 处躯干半宽 80px, 10m 处 24px)。
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from arena.scenarios import Scenario, StaticTarget

REF_DIST_M = 10.0                 # 每发落差与射速档的参考交战距离 m
BURST_MS = 1600.0                 # 射击段时长 ms (三个发间隔的公倍数)
SETTLE_MS = 800.0                 # 射击前静置段 / 停火观察段 ms (同上, 公倍数)
BURST_GROUP = 3                   # 点射模式每组的发数
BURST_PAUSE_MS = 400.0            # 点射组间停顿 ms (> 现役律阶跃稳定时间 277ms)

RPM_GRID = (300.0, 450.0, 900.0)
KICK_GRID_PX = (6.0, 20.0, 60.0)


def shot_interval_ms(rpm: float) -> float:
    """发间隔 ms = 60000/RPM。"""
    return 60000.0 / rpm


class RecoilPattern:
    """周期性后坐力脉冲串 (纯时间函数, 无隐藏状态)。

    offset(t) -> (ox, oy): t 时刻瞄准轴相对"未被推动"位置的偏移 px (ox 恒为 0,
    后坐力只作用在竖直方向)。每发按 rise_ms 上升, 再按 recover_tau 指数回正;
    rise_ms=0 即瞬时阶跃, recover_tau=0 即不回正 (位移一直留着等被补偿)。
    """

    def __init__(self, shots, kick_px: float, rise_ms: float = 0.0,
                 recover_tau: float = 0.0):
        self.shots = tuple(shots)
        for a, b in zip(self.shots, self.shots[1:]):
            if b < a:
                raise ValueError("shots 必须递增")
        self.kick_px = kick_px
        self.rise_ms = rise_ms
        self.recover_tau = recover_tau

    def offset(self, t: float):
        oy = 0.0
        for ts in self.shots:
            if t <= ts:
                break
            dt = t - ts
            a = 1.0 if self.rise_ms <= 0.0 else min(1.0, dt / self.rise_ms)
            if self.recover_tau > 0.0:
                a *= math.exp(-dt / self.recover_tau)
            oy += self.kick_px * a
        return 0.0, oy


@dataclass
class RecoilScenario(Scenario):
    """后坐力场景: 除 Scenario 的字段外带一个 RecoilPattern (扰动源) 与每发
    时刻/落差/发间隔 (供 metrics.recoil_metrics 做每发对齐统计)。

    kind="track", steady_from = 射击段起点; phases 声明射击段为 "recoil" 段,
    因此 phase_metrics 能直接给出该段的 rmse/p95/on_body。
    """
    recoil: object = None
    burst_ms: float = BURST_MS
    rpm: float = 300.0
    kick_ref_px: float = 0.0

    @property
    def shots(self):
        return self.recoil.shots

    @property
    def kick_eff_px(self):
        """交战距离处的每发落差 px (kick ∝ 1/d, 与 fps.py 同一规则) —— 也是
        扰动本身实际施加的幅值 (RecoilPattern.kick_px)。"""
        return self.recoil.kick_px

    @property
    def interval_ms(self):
        return shot_interval_ms(self.rpm)


def _auto_shots(t0: float, burst_ms: float, interval: float):
    """连发: 从 t0 起按发间隔排满射击段 (burst_ms 是发间隔的整数倍)。"""
    return tuple(t0 + i * interval
                 for i in range(int(round(burst_ms / interval))))


def _burst_shots(t0: float, burst_ms: float, interval: float,
                 group: int = BURST_GROUP, pause_ms: float = BURST_PAUSE_MS):
    """点射: 每 group 发一组, 组间停顿 pause_ms。"""
    shots = []
    t = t0
    while t < t0 + burst_ms:
        for _ in range(group):
            if t >= t0 + burst_ms:
                break
            shots.append(t)
            t += interval
        t += pause_ms
    return tuple(shots)


def make_recoil_scenario(name: str, rpm: float, kick_px: float,
                         kick_dist_m: float = REF_DIST_M,
                         shots=None, rise_ms: float = 0.0,
                         recover_tau: float = 0.0, t0: float = SETTLE_MS):
    """组装一个后坐力场景。kick_px 定义在 REF_DIST_M 处, 场景交战距离为
    kick_dist_m (落差按 1/d 折算, 目标本身静止)。时长统一 = t0+BURST_MS+
    SETTLE_MS, 各组合的射击段/观察段物理时长逐档相同。"""
    interval = shot_interval_ms(rpm)
    if shots is None:
        shots = _auto_shots(t0, BURST_MS, interval)
    kick = kick_px * REF_DIST_M / max(1e-6, kick_dist_m)   # 1/d 折算
    pattern = RecoilPattern(shots, kick, rise_ms=rise_ms,
                            recover_tau=recover_tau)
    return RecoilScenario(
        name, t0 + BURST_MS + SETTLE_MS, lambda rng: StaticTarget(40.0, 0.0),
        (0.0, 0.0), "track", steady_from=t0,
        phases=((t0, t0 + BURST_MS, "recoil"),), dist_m=kick_dist_m,
        recoil=pattern, rpm=rpm, kick_ref_px=kick_px)


def recoil_suite():
    """后坐力场景族: 3 射速 × 3 落差 + 点射 / 斜坡 / 相机回正 / 近距。"""
    suite = []
    for rpm in RPM_GRID:
        for k in KICK_GRID_PX:
            suite.append(make_recoil_scenario(
                f"rc_{rpm:g}rpm_{k:g}px", rpm, k))
    suite.append(make_recoil_scenario(
        "rc_900rpm_20px_burst3", 900.0, 20.0,
        shots=_burst_shots(SETTLE_MS, BURST_MS, shot_interval_ms(900.0))))
    suite.append(make_recoil_scenario(
        "rc_300rpm_20px_ramp12", 300.0, 20.0, rise_ms=12.0))
    suite.append(make_recoil_scenario(
        "rc_300rpm_20px_rec300", 300.0, 20.0, recover_tau=300.0))
    # 近距档: 10m 处 6px/发 → 3m 处 20px/发 (1/d), 与 rc_300rpm_20px 同幅值
    suite.append(make_recoil_scenario(
        "rc_300rpm_20px_3m", 300.0, 6.0, kick_dist_m=3.0))
    return suite
