"""arena/scenarios.py — 目标运动模型 + 标准场景套件。

目标运动: 静止 / 匀速 / 匀加速 (诊断) + 随机机动 (主场景)。
随机机动: 速度方向与大小随机变化, 变化频率有上限 (~3 次/秒), 速度有上限, 活动范围受限。
场景是中立的目标真值生成器, 不含任何控制逻辑。
"""
from __future__ import annotations
import math
from dataclasses import dataclass


class Target:
    def __init__(self, x=0.0, y=0.0, vx=0.0, vy=0.0):
        self.x, self.y = x, y
        self.vx, self.vy = vx, vy

    def advance(self, h: float, t: float):
        self.x += self.vx * h
        self.y += self.vy * h


class StaticTarget(Target):
    def advance(self, h, t):
        pass


class AccelTarget(Target):
    def __init__(self, x, y, vx, vy, ax, ay):
        super().__init__(x, y, vx, vy)
        self.ax, self.ay = ax, ay

    def advance(self, h, t):
        self.vx += self.ax * h
        self.vy += self.ay * h
        self.x += self.vx * h
        self.y += self.vy * h


class JumpTarget(Target):
    """重新锁定: 起始静止于 (x0,y0), 在 t=jump_t 瞬间跳到 (x1,y1) (切目标/丢失重锁)。
    测试 law 的跳变重置与二次拉枪。"""

    def __init__(self, x0, y0, x1, y1, jump_t):
        super().__init__(x0, y0)
        self.x1, self.y1 = x1, y1
        self.jump_t = jump_t
        self.jumped = False

    def advance(self, h, t):
        if not self.jumped and t >= self.jump_t:
            self.x, self.y = self.x1, self.y1
            self.jumped = True


class ManeuverTarget(Target):
    """随机机动: 分段常速度, 每段随机方向/大小; 变化间隔 >= min_interval (限频);
    速度 <= vmax; 出界则朝中心反弹。"""

    def __init__(self, rng, x=0.0, y=0.0, vmax=0.4, min_interval=333.0,
                 bound=120.0):
        super().__init__(x, y)
        self.rng = rng
        self.vmax = vmax            # px/ms (0.4 px/ms = 400 px/s)
        self.min_interval = min_interval  # ms (~3 次/秒)
        self.bound = bound
        self._next_change = 0.0
        self._pick_velocity()

    def _pick_velocity(self):
        ang = self.rng.uniform(0, 2 * math.pi)
        spd = self.rng.uniform(0.3, 1.0) * self.vmax
        self.vx = math.cos(ang) * spd
        self.vy = math.sin(ang) * spd

    def advance(self, h, t):
        if t >= self._next_change:
            self._pick_velocity()
            self._next_change = t + self.rng.uniform(
                self.min_interval, self.min_interval * 2.5)
        self.x += self.vx * h
        self.y += self.vy * h
        if self.x > self.bound:
            self.x = self.bound; self.vx = -abs(self.vx)
        elif self.x < -self.bound:
            self.x = -self.bound; self.vx = abs(self.vx)
        if self.y > self.bound:
            self.y = self.bound; self.vy = -abs(self.vy)
        elif self.y < -self.bound:
            self.y = -self.bound; self.vy = abs(self.vy)


@dataclass
class Scenario:
    name: str
    duration: float
    make_target: object          # callable(rng) -> Target
    cross0: tuple                # 初始准星位置
    kind: str                    # "step" | "track"
    step_dir: tuple = (1.0, 0.0) # step 场景的阶跃方向 (供过冲投影)
    settle_band: float = 3.0     # 稳定判据 px
    steady_from: float = 0.0     # 跟踪场景从何时起算稳态指标


def _step(rng, dist=80.0, ang=0.0):
    return StaticTarget(x=math.cos(ang) * dist, y=math.sin(ang) * dist)


def standard_suite(seed_base=0):
    """标准多组场景。所有 law 在完全相同的这组场景下评测。"""
    suite = []
    suite.append(Scenario("step_80px", 1200.0,
                          lambda rng: _step(rng, 80.0, 0.0),
                          (0.0, 0.0), "step", (1.0, 0.0)))
    suite.append(Scenario("step_80px_diag", 1200.0,
                          lambda rng: _step(rng, 80.0, math.pi / 4),
                          (0.0, 0.0), "step",
                          (math.cos(math.pi / 4), math.sin(math.pi / 4))))
    suite.append(Scenario("const_vel", 3000.0,
                          lambda rng: Target(60.0, 0.0, 0.0, 0.25),
                          (0.0, 0.0), "track", steady_from=800.0))
    suite.append(Scenario("accel", 2500.0,
                          lambda rng: AccelTarget(50.0, 0.0, 0.0, 0.0, 0.0, 0.0006),
                          (0.0, 0.0), "track", steady_from=700.0))
    suite.append(Scenario("maneuver", 6000.0,
                          lambda rng: ManeuverTarget(rng, 40.0, 20.0),
                          (0.0, 0.0), "track", steady_from=1000.0))
    return suite
