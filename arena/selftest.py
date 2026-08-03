"""arena/selftest.py — 用参考 law 自检 arena。

验证: 参考 law (Smith+PI 基线) 在 arena 中能稳定收敛、表现出手调 τ 的爬坡特性,
延迟准确时不发散。若发散/不收敛 → arena 植物/传感器没对齐, 先修 arena。
"""
from __future__ import annotations
from arena.core import ArenaConfig
from arena.laws.base import get_law
from arena import runner


def main():
    RefLaw = get_law("reference")
    cfg = ArenaConfig(noise_std=0.5, fps=120)

    print("### 自检: 参考 law, 延迟准确 (L_true=L_belief=50ms), 120fps, 噪声0.5px")
    res = runner.run_suite(lambda: RefLaw(tau=80, tau_i=60, max_v=1.5),
                           arena_cfg=cfg, s_belief=1.0, L_belief=50.0,
                           max_v=1.5, s_true=1.0, L_true=50.0)
    runner.print_suite("reference @ L=50 (matched)", res,
                       runner.composite_score(res))

    print("\n### 帧率无关性: 同一 law 同样参数, 60fps")
    cfg60 = ArenaConfig(noise_std=0.5, fps=60)
    res60 = runner.run_suite(lambda: RefLaw(tau=80, tau_i=60, max_v=1.5),
                             arena_cfg=cfg60, s_belief=1.0, L_belief=50.0,
                             max_v=1.5, s_true=1.0, L_true=50.0)
    runner.print_suite("reference @ 60fps", res60, runner.composite_score(res60))

    print("\n### 延迟失配: L_true=70, L_belief=50 (低估 20ms)")
    res_mm = runner.run_suite(lambda: RefLaw(tau=80, tau_i=60, max_v=1.5),
                              arena_cfg=cfg, s_belief=1.0, L_belief=50.0,
                              max_v=1.5, s_true=1.0, L_true=70.0)
    runner.print_suite("reference @ L_true=70/belief=50", res_mm,
                       runner.composite_score(res_mm))


if __name__ == "__main__":
    main()
