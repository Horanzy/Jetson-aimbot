"""arena/laws/__init__.py — 导入所有 law 模块以触发注册。
新增 law: 在本目录加文件, 用 @register("名字"), 并在下面 import。
"""
from arena.laws import reference  # noqa: F401

try:
    from arena.laws import pi_pm        # noqa: F401
except Exception:
    pass
try:
    from arena.laws import kalman_pi    # noqa: F401
except Exception:
    pass
try:
    from arena.laws import ballistic    # noqa: F401
except Exception:
    pass
try:
    from arena.laws import ff_pi        # noqa: F401
except Exception:
    pass
try:
    from arena.laws import sliding      # noqa: F401
except Exception:
    pass
try:
    from arena.laws import smith_filt   # noqa: F401
except Exception:
    pass
try:
    from arena.laws import mpc          # noqa: F401
except Exception:
    pass
