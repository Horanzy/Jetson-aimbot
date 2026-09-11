"""arena/laws/__init__.py — 导入所有 law 模块以触发注册。
新增 law: 在本目录加文件, 用 @register("名字"), 并在下面加一行 import。
"""
from arena.laws import reference   # noqa: F401
from arena.laws import pi_pm       # noqa: F401
from arena.laws import kalman_pi   # noqa: F401
from arena.laws import ballistic   # noqa: F401
from arena.laws import reseed_pi    # noqa: F401
from arena.laws import imm_pi      # noqa: F401
from arena.laws import sliding     # noqa: F401
from arena.laws import sliding_obs  # noqa: F401
from arena.laws import smith_filt  # noqa: F401
from arena.laws import ff_pi_acc   # noqa: F401
from arena.laws import ballistic_ff    # noqa: F401
from arena.laws import mpc_osc     # noqa: F401
from arena.laws import pi_guard    # noqa: F401

# WIP 实验 law (调试用临时副本): 自动导入 _wip_*.py, 让并行实验不必碰这个
# 共享文件。单个文件坏了只跳过自身 (stderr 提示), 不拖垮其它 law 的评测。
# 正式落地的新 law 仍走上面的显式 import + AUTHORING 流程。
import glob as _glob
import importlib as _importlib
import os as _os
import sys as _sys
for _p in sorted(_glob.glob(_os.path.join(_os.path.dirname(__file__), "_wip_*.py"))):
    _mod = f"{__name__}.{_os.path.basename(_p)[:-3]}"
    try:
        _importlib.import_module(_mod)
    except Exception as _e:  # noqa: BLE001 — 实验文件允许坏, 不影响正式 law
        print(f"[arena.laws] 跳过无法导入的 WIP law {_mod}: {_e}", file=_sys.stderr)
