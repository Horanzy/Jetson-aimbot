"""arena/fps.py — FPS 角色运动行为库 (屏幕空间轨迹生成器) + 标准行为套件。

为什么是 2D 屏幕空间 (设计依据, 不要在此引入 3D 世界):
    整个感知-控制回路只存在于屏幕像素空间: 采集(2D) → 检测(2D bbox) → dx,dy(px)
    → law → counts → 准星(px)。3D 游戏世界只是屏幕轨迹的一个生成器, law 永远
    看不到世界, 只看得到屏幕轨迹。因此"模拟 3D 游戏"的正确方式是让轨迹库覆盖
    3D 行为投影到屏幕后的形态, 而不是在 arena 里建 3D 世界。
    唯一未建模的 3D 效应是 tan 投影非线性 (s 随偏角 sec² 变化): 跟踪发生在
    FOV 圈内 ±150px ≈ 8.9° (f≈960px, 90°hFOV) → s 变化 ~2.4%, 可忽略; 只有
    大角度拉枪才有感, 那部分由 JUMP_GATE/标定兜底。以后如需精确投影, 再加一个
    内部跑 3D 世界+相机+投影的 Target 子类即可 (core 零改动)。

物理量纲换算 (参数标定的依据, 距离 d 米, f≈960px @1080p/90°hFOV):
    世界速度 v (m/s) → 屏幕速度 ≈ 960·v/d px/s。
    例: CS 250u/s strafe ≈ 4.8m/s → 10m 处 ≈ 460px/s = 0.46 px/ms。
    跳跃: 起跳 ≈5.7m/s → 屏幕竖直速度 ≈ 0.55 px/ms @10m; 重力 15.24m/s²
    (Source 800u/s²) → 屏幕重力 ≈ 0.0015 px/ms²; 滞空 ≈ 0.73s, 弧顶 ≈ 100px。
    (蹬墙跳/滑铲/冲刺速度取 Apex/Titanfall 量级, 同一换算。)

关键投影事实 (决定各类的实现方式):
    - 平移 strafe 的屏幕方向 heading 是任意的 (取决于玩家朝向与几何), 因此
      每个行为都接受 heading 角。
    - 跳跃是世界竖直运动, 投影后永远是屏幕竖直 (y) 方向, 与 strafe 方向无关。
      所以跳跃弧只叠加在屏幕 y 上, 与 strafe 分量正交累加 (忠实于投影, 而非
      "沿 strafe 方向的弧")。
    - 绕玩家转圈 (orbit) 在固定相机下投影近似匀速直线 (tan 修正到 ±45° 才
      明显), 已被 const_vel/maneuver 覆盖, 不设专门场景。

事件标注: 每个场景在 FpsScenario.events 里声明控制学意义重大的时刻
(急停/落地/折返/变向/超帽), 供 metrics.event_metrics 计算事件后过冲与
恢复时间。丢帧 (检测闪烁) 是传感器属性, 在 core.ArenaConfig.drop_p。
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from arena.scenarios import Scenario, Target


@dataclass
class FpsScenario(Scenario):
    """带事件标注的场景。events: ((t_ms, kind), ...), kind 为自由文本。"""
    events: tuple = ()


G_SCREEN = 0.0015      # 屏幕竖直重力 px/ms² (Source 800u/s² @10m)
VZ_JUMP = 0.55         # 起跳屏幕竖直速度 px/ms (≈5.7m/s @10m)


class _FpsBase(Target):
    """公共骨架: base 位置按当前速度增量积分 (heading 可变, 转向连续);
    跳跃弧是绝对 y 偏移, 叠加在 base y 上。"""

    def __init__(self, x, y, heading_deg=0.0):
        super().__init__(x, y)
        self.vx, self.vy = 0.0, 0.0           # 屏幕 px/ms (检视用)
        self.hd = math.radians(heading_deg)
        self.cx_, self.sy_ = math.cos(self.hd), math.sin(self.hd)
        self._bx, self._by = x, y
        self._t = 0.0

    def _speed(self) -> float:
        """当前 strafe 速率 (px/ms)。子类按自身运动学覆写。"""
        return 0.0

    def _jump_dy(self) -> float:
        """跳跃弧的屏幕 y 偏移 (px)。有跳跃的子类覆写, 默认无。"""
        return 0.0

    def _pre_advance(self, h, t):
        """每拍钩子: 推进状态机 (折返/落地/衰减), 在积分前调用。"""
        self._t = t

    def advance(self, h, t):
        self._pre_advance(h, t)
        sp = self._speed()
        self.vx, self.vy = sp * self.cx_, sp * self.sy_
        self._bx += self.vx * h
        self._by += self.vy * h
        self.x, self.y = self._bx, self._by + self._jump_dy()


def _flip_speed(t, v, first_flip_t, interval, n_flips, switch_ms, done_sign):
    """方波折返的速度插值: 第 i 次折返在 first_flip_t + i·interval,
    每次 switch_ms 内速度从旧符号线性过渡到新符号 (地面/空中加速限制)。"""
    if n_flips <= 0 or t < first_flip_t:
        return v
    k = int((t - first_flip_t) // interval) + 1     # 正在/已完成第 k 次折返
    if k > n_flips:
        return v * done_sign(n_flips)
    frac = min(1.0, (t - (first_flip_t + (k - 1) * interval)) / switch_ms)
    prev = done_sign(k - 1)
    nxt = done_sign(k)
    return v * (prev + (nxt - prev) * frac)


class StopTarget(_FpsBase):
    """地面匀速 strafe → t_stop 急停。friction_tau=0 硬停 (理想化),
    >0 按地面摩擦指数衰减 (Source friction≈5.2 → τ≈190ms)。"""

    def __init__(self, x, vx, t_stop, friction_tau=0.0, heading_deg=0.0, y=0.0):
        super().__init__(x, y, heading_deg)
        self.v = vx
        self.t_stop = t_stop
        self.tau = friction_tau

    def _speed(self):
        if self.t_stop is None or self._t < self.t_stop:
            return self.v
        if self.tau <= 0.0:
            return 0.0
        return self.v * math.exp(-(self._t - self.t_stop) / self.tau)


class JumpLandTarget(_FpsBase):
    """strafe → t_jump 起跳 (屏幕 y 抛物线) → t_land 落地。
    land_mode: 'stop' 落地无输入 → strafe 速度按 land_tau 衰减 (0=硬停),
               即"落地即急停"; 'keep' 落地继续 strafe (仅 y 轴急停)。
    air_vx_delta: CS2 空中加速, 弧顶给 strafe 速率一个小阶跃
                  (≈30u/s → 10m 处 ≈0.06 px/ms)。"""

    def __init__(self, x, vx, t_jump, vz0=VZ_JUMP, g=G_SCREEN,
                 land_mode="stop", land_tau=0.0, air_vx_delta=0.0,
                 heading_deg=0.0, y=0.0):
        super().__init__(x, y, heading_deg)
        self.v = vx
        self.t_jump = t_jump
        self.vz0, self.g = vz0, g
        self.land_mode = land_mode
        self.land_tau = land_tau
        self.air_vx_delta = air_vx_delta
        self.t_air = 2.0 * vz0 / g
        self.t_land = t_jump + self.t_air
        self._jvy, self._jy = 0.0, 0.0
        self._airborne = False
        self._delta_done = False
        self._extra = 0.0

    @staticmethod
    def land_time(t_jump, vz0=VZ_JUMP, g=G_SCREEN):
        return t_jump + 2.0 * vz0 / g

    def _speed(self):
        if self.land_mode == "stop" and self._t >= self.t_land:
            v0 = self.v + self._extra
            if self.land_tau <= 0.0:
                return 0.0
            return v0 * math.exp(-(self._t - self.t_land) / self.land_tau)
        return self.v + self._extra

    def _jump_dy(self):
        return self._jy

    def _pre_advance(self, h, t):
        self._t = t
        if t >= self.t_jump and not self._airborne:
            self._airborne = True
            self._jvy = self.vz0
        if self._airborne:
            self._jvy -= self.g * h
            self._jy += self._jvy * h
            if (not self._delta_done and self.air_vx_delta != 0.0
                    and t >= self.t_jump + self.t_air / 2.0):
                self._extra += self.air_vx_delta
                self._delta_done = True
            if t >= self.t_land:
                self._airborne = False
                self._jvy, self._jy = 0.0, 0.0


class WallBounceTarget(_FpsBase):
    """Apex 蹬墙跳: 空中 t_bounce 时刻 strafe 速度全额折返 (bounce_keep=1
    即 +V→−V, 阶跃幅度 2V — 比急停更狠) 并蹬出一次向上的新弧 (kick×vz0)。
    蹬墙点高度由第一跳抛物线决定, 落地时刻从该高度解出。"""

    def __init__(self, x, vx, t_jump, vz0=VZ_JUMP, g=G_SCREEN, t_bounce=None,
                 bounce_keep=1.0, kick=0.6, heading_deg=0.0, y=0.0):
        super().__init__(x, y, heading_deg)
        self.v = vx
        self.t_jump = t_jump
        self.vz0, self.g = vz0, g
        self.t_bounce = t_bounce if t_bounce is not None else t_jump + vz0 / g
        self.bounce_keep = bounce_keep
        self.kick = kick
        y_b = vz0 * (self.t_bounce - t_jump) \
            - 0.5 * g * (self.t_bounce - t_jump) ** 2      # 蹬墙点高度
        vk = kick * vz0
        self.t_land = self.t_bounce + (vk + math.sqrt(vk * vk + 2 * g * y_b)) / g
        self._jvy, self._jy = 0.0, 0.0
        self._airborne = False
        self._sign = 1.0

    def _speed(self):
        return self.v * self._sign

    def _jump_dy(self):
        return self._jy

    def _pre_advance(self, h, t):
        self._t = t
        if t >= self.t_jump and not self._airborne:
            self._airborne = True
            self._jvy = self.vz0
        if self._airborne and self._sign > 0 and t >= self.t_bounce:
            self._sign = -self.bounce_keep
            self._jvy = self.vz0 * self.kick
        if self._airborne:
            self._jvy -= self.g * h
            self._jy += self._jvy * h
            if t >= self.t_land:
                self._airborne = False
                self._jvy, self._jy = 0.0, 0.0


class StrafeSwitchTarget(_FpsBase):
    """地面变向: 每 period 一次 +V→−V 反转, 切换耗时 switch_ms
    (地面加速限制, CS ≈50-100ms)。"""

    def __init__(self, x, vx, period=900.0, n_switches=3, switch_ms=70.0,
                 heading_deg=0.0, y=0.0):
        super().__init__(x, y, heading_deg)
        self.v = vx
        self.period = period
        self.n = n_switches
        self.switch_ms = switch_ms

    def _speed(self):
        return _flip_speed(self._t, self.v, self.period, self.period,
                           self.n, self.switch_ms, lambda k: -1.0 if k % 2 else 1.0)


class JiggleTarget(_FpsBase):
    """小幅快速抖动 (peek spam / 拉扯): 绕掩体位往复, 每腿 amp px、半周期
    period/2, 共 n_flips 条腿 (偶数为好, 终点回掩体), 腿间 switch_ms 内
    换向 — 高频小幅度 reverse, 考验滞后跟平。"""

    def __init__(self, x, amp=25.0, period=500.0, n_flips=6, switch_ms=40.0,
                 heading_deg=0.0, y=0.0):
        super().__init__(x, y, heading_deg)
        self.amp = amp
        self.v = amp / (period / 2.0)
        self.period = period
        self.legs = n_flips
        self.switch_ms = switch_ms
        self.events = [(period / 2.0 * i, "reverse")
                       for i in range(1, self.legs)]
        self.events.append((period / 2.0 * self.legs, "stop"))

    def _speed(self):
        t = self._t
        half = self.period / 2.0
        leg = int(t // half)
        if leg >= self.legs:
            return 0.0
        frac = min(1.0, (t - leg * half) / self.switch_ms)
        sgn = 1.0 if leg % 2 == 0 else -1.0
        prev = 0.0 if leg == 0 else -sgn * self.v
        return prev + (sgn * self.v - prev) * frac


class BhopTarget(_FpsBase):
    """连跳/滑跳链 (CS bhop / Titanfall slide-hop): 每 period 一跳,
    strafe 速度全程保持, 每次落地 = 一次 y 轴硬停。period 必须 >= 滞空
    2·vz0/g (落地才起跳)。"""

    def __init__(self, x, vx, t0_jump, period=750.0, n_jumps=3,
                 vz0=VZ_JUMP, g=G_SCREEN, heading_deg=0.0, y=0.0):
        super().__init__(x, y, heading_deg)
        self.v = vx
        self.t0 = t0_jump
        self.period = period
        self.n = n_jumps
        self.vz0, self.g = vz0, g
        self.t_air = 2.0 * vz0 / g
        self._jvy, self._jy = 0.0, 0.0
        self._cur = None

    def land_times(self):
        return [self.t0 + i * self.period + self.t_air
                for i in range(self.n)]

    def _speed(self):
        return self.v

    def _jump_dy(self):
        return self._jy

    def _pre_advance(self, h, t):
        self._t = t
        grounded = True
        if t >= self.t0:
            i = int((t - self.t0) // self.period)
            if i < self.n and t < self.t0 + i * self.period + self.t_air:
                grounded = False
                if self._cur != i:
                    self._cur = i           # 落地即起跳 (bhop 无停顿)
                    self._jvy = self.vz0
                    self._jy = 0.0
                self._jvy -= self.g * h
                self._jy = max(0.0, self._jy + self._jvy * h)
        if grounded:
            self._cur = None
            self._jvy, self._jy = 0.0, 0.0


class SlideTarget(_FpsBase):
    """滑铲 (Apex/CFHD): t_slide 以 boost×V 起滑, 摩擦指数衰减 (tau),
    slide_ms 后 end='stop' 停 / 'keep' 恢复 V; 屏幕 y 同时下蹲 crouch_dy
    (瞄准点下移, crouch_ms 内平滑过渡), 结束回站。"""

    def __init__(self, x, vx, t_slide, boost=1.5, tau=350.0, slide_ms=700.0,
                 crouch_dy=-40.0, crouch_ms=150.0, end="stop",
                 heading_deg=0.0, y=0.0):
        super().__init__(x, y, heading_deg)
        self.v = vx
        self.t_slide = t_slide
        self.boost, self.tau, self.slide_ms = boost, tau, slide_ms
        self.crouch_dy, self.crouch_ms = crouch_dy, crouch_ms
        self.end = end

    def _speed(self):
        t = self._t
        if t < self.t_slide:
            return self.v
        if t < self.t_slide + self.slide_ms:
            return (self.v * self.boost
                    * math.exp(-(t - self.t_slide) / self.tau))
        return 0.0 if self.end == "stop" else self.v

    def _jump_dy(self):
        t = self._t
        t0, t1 = self.t_slide, self.t_slide + self.slide_ms
        if t0 <= t < t0 + self.crouch_ms:
            return self.crouch_dy * (t - t0) / self.crouch_ms
        if t0 + self.crouch_ms <= t < t1:
            return self.crouch_dy
        if t1 <= t < t1 + self.crouch_ms:
            return self.crouch_dy * (1.0 - (t - t1) / self.crouch_ms)
        return 0.0


class TurnTarget(_FpsBase):
    """路径转向 (贴墙跑进出/绕障碍): t_turn 起 heading 在 turn_ms 内转
    phi, 速率不变 — 平滑的速度方向旋转 (区别于变向的折返)。"""

    def __init__(self, x, speed, phi_deg=90.0, turn_ms=120.0, t_turn=1000.0,
                 heading_deg=0.0, y=0.0):
        super().__init__(x, y, heading_deg)
        self.v = speed
        self.phi = math.radians(phi_deg)
        self.turn_ms = turn_ms
        self.t_turn = t_turn

    def _speed(self):
        return self.v

    def _pre_advance(self, h, t):
        self._t = t
        frac = max(0.0, min(1.0, (t - self.t_turn) / self.turn_ms))
        ang = self.hd + self.phi * frac
        self.cx_, self.sy_ = math.cos(ang), math.sin(ang)


class ApproachTarget(_FpsBase):
    """目标带横向分量向玩家逼近: 屏幕速度随距离缩短线性上升 v=v0+a·t,
    标注越过速度帽 1.5 px/ms 的时刻 (追得上的分界)。"""

    def __init__(self, x, v0=0.3, a=0.00065, duration=2600.0,
                 heading_deg=0.0, y=0.0):
        super().__init__(x, y, heading_deg)
        self.v0, self.a = v0, a
        self.events = []
        t_cap = (1.5 - v0) / a if a > 0 else float("inf")
        if 0.0 < t_cap < duration:
            self.events.append((t_cap, "cap-cross"))

    def _speed(self):
        return self.v0 + self.a * self._t


class DashTarget(_FpsBase):
    """能力冲刺 (闪现突进/战术冲刺): t_dash 起 burst×V 突冲 dash_ms,
    结束后 end='stop' 急停 / 'keep' 恢复巡航。突冲速度通常超速度帽 →
    饱和追赶接急停, 最恶劣复合工况。"""

    def __init__(self, x, vx, t_dash, burst=4.0, dash_ms=120.0, end="stop",
                 heading_deg=0.0, y=0.0):
        super().__init__(x, y, heading_deg)
        self.v = vx
        self.t_dash = t_dash
        self.burst, self.dash_ms, self.end = burst, dash_ms, end

    def _speed(self):
        t = self._t
        if self.t_dash <= t < self.t_dash + self.dash_ms:
            return self.v * self.burst
        if t >= self.t_dash + self.dash_ms:
            return 0.0 if self.end == "stop" else self.v
        return self.v


def fps_suite():
    """标准 FPS 行为套件。速度取 10m 交战距离的屏幕换算 (见模块 docstring):
    strafe 0.45-0.7 px/ms, 跳跃 vz0≈0.55 / g≈0.0015, 蹬墙跳折返幅度 2V。"""
    suite = []
    add = suite.append

    add(FpsScenario(
        "fps_stop_hard", 2500.0,
        lambda rng: StopTarget(40.0, 0.7, 1500.0),
        (0.0, 0.0), "track", steady_from=0.0,
        events=((1500.0, "stop"),)))
    add(FpsScenario(
        "fps_stop_friction", 2500.0,
        lambda rng: StopTarget(40.0, 0.7, 1500.0, friction_tau=190.0),
        (0.0, 0.0), "track", steady_from=0.0,
        events=((1500.0, "stop"),)))
    add(FpsScenario(
        "fps_jump_land_stop", 2600.0,
        lambda rng: JumpLandTarget(40.0, 0.5, 800.0, land_mode="stop",
                                   land_tau=120.0),
        (0.0, 0.0), "track", steady_from=0.0,
        events=((JumpLandTarget.land_time(800.0), "land"),)))
    add(FpsScenario(
        "fps_jump_land_keep", 2600.0,
        lambda rng: JumpLandTarget(40.0, 0.5, 800.0, land_mode="keep"),
        (0.0, 0.0), "track", steady_from=0.0,
        events=((JumpLandTarget.land_time(800.0), "land"),)))
    add(FpsScenario(
        "fps_jump_airaccel", 2600.0,
        lambda rng: JumpLandTarget(40.0, 0.5, 800.0, land_mode="keep",
                                   air_vx_delta=0.06),
        (0.0, 0.0), "track", steady_from=0.0,
        events=((800.0 + 0.55 / 0.0015, "air-accel"),
                (JumpLandTarget.land_time(800.0), "land"))))
    add(FpsScenario(
        "fps_wall_bounce", 2800.0,
        lambda rng: WallBounceTarget(40.0, 0.6, 600.0, t_bounce=950.0),
        (0.0, 0.0), "track", steady_from=0.0,
        events=((950.0, "bounce-2V"),
                (WallBounceTarget(
                    0.0, 0.6, 600.0, t_bounce=950.0).t_land, "land"))))
    add(FpsScenario(
        "fps_strafe_switch", 3000.0,
        lambda rng: StrafeSwitchTarget(40.0, 0.5, period=900.0, n_switches=3),
        (0.0, 0.0), "track", steady_from=0.0,
        events=((900.0, "reverse"), (1800.0, "reverse"),
                (2700.0, "reverse"))))
    add(FpsScenario(
        "fps_jiggle", 3200.0,
        lambda rng: JiggleTarget(40.0, amp=25.0, period=500.0, n_flips=6),
        (0.0, 0.0), "track", steady_from=0.0,
        events=tuple((250.0 * i, "reverse") for i in range(1, 6))
        + ((1500.0, "stop"),)))
    add(FpsScenario(
        "fps_bhop", 3200.0,
        lambda rng: BhopTarget(40.0, 0.5, 600.0, period=750.0, n_jumps=3),
        (0.0, 0.0), "track", steady_from=0.0,
        events=tuple((t, "land") for t in
                     BhopTarget(0.0, 0.0, 600.0, period=750.0,
                                n_jumps=3).land_times())))
    add(FpsScenario(
        "fps_slide", 2500.0,
        lambda rng: SlideTarget(40.0, 0.5, 700.0),
        (0.0, 0.0), "track", steady_from=0.0,
        events=((700.0, "slide"), (1400.0, "rise"))))
    add(FpsScenario(
        "fps_turn_90", 2200.0,
        lambda rng: TurnTarget(40.0, 0.35, phi_deg=90.0, turn_ms=120.0,
                               t_turn=1000.0, heading_deg=0.0, y=10.0),
        (0.0, 0.0), "track", steady_from=0.0,
        events=((1000.0, "turn"),)))
    add(FpsScenario(
        "fps_approach", 2600.0,
        lambda rng: ApproachTarget(40.0, v0=0.3, a=0.00065, duration=2600.0),
        (0.0, 0.0), "track", steady_from=0.0, events=()))
    add(FpsScenario(
        "fps_dash", 2500.0,
        lambda rng: DashTarget(40.0, 0.4, 800.0, burst=4.0, dash_ms=120.0,
                               end="stop"),
        (0.0, 0.0), "track", steady_from=0.0,
        events=((920.0, "stop"),)))
    return suite
