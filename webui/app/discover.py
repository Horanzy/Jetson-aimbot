"""结构化发现: 从部署根按约定目录派生 游戏 profile / 模型 / 转换源 / 采集设备。

只按固定结构发现 (scripts/game/, engine/, onnx/, /dev/v4l/by-id/), 不递归扫全盘。
game 脚本只读解析 (顶部 VAR=value 块), WebUI 从不改写脚本 —— 唯一的脚本写回
仍是固件经 -S 的标定回写机制。profile 参数持久化在 webui/data/profiles/<脚本名>.json,
首扫时以脚本值播种; 之后保存值与脚本值分叉即"漂移", 由 UI 提示。
"""
import json
import re
import time
from pathlib import Path

from . import config

VAR_RE = re.compile(r"^\s*([A-Z_][A-Z0-9_]*)\s*=\s*(.*?)\s*$")
SCRIPT_STEM_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")

# 参数定义: 类型/范围与服务端校验 (与 CLI/固件钳制一致, 双保险)
PARAM_DEFS = {
    "model":           dict(kind="path",  default=None),
    "class_id":        dict(kind="int",   lo=0, hi=255, default=0),
    "conf":            dict(kind="float", lo=0.0, hi=1.0, default=0.5),
    "y_offset":        dict(kind="float", lo=0.0, hi=100.0, default=65.0),
    "cam_dev":         dict(kind="str",   default="Asus"),
    "cam_fps":         dict(kind="int",   lo=1, hi=240, default=120),
    "max_speed":       dict(kind="float", lo=100.0, hi=20000.0, default=1500.0),
    "aim_key":         dict(kind="enum",  choices=("fire", "ads", "both"), default="both"),
    "fov":             dict(kind="float", lo=10.0, hi=1000.0, default=150.0),
    "preview":         dict(kind="bool",  default=False),
    "capture_enabled": dict(kind="bool",  default=False),
    "capture_dir":     dict(kind="dsdir", default=None),
    "fire_ms":         dict(kind="int",   lo=50, hi=60000, default=800),
    "auto_s":          dict(kind="float", lo=1.0, hi=3600.0, default=10.0),
    "cooldown_ms":     dict(kind="int",   lo=0, hi=60000, default=800),
    "jpeg_q":          dict(kind="int",   lo=1, hi=100, default=95),
}
# 热参数白名单: param key → 固件通道 key (对应 src/aimbot.cu hotctl_thread)
HOT_WIRE_KEYS = {"conf": "t", "y_offset": "y", "max_speed": "x", "fov": "fov", "aim_key": "k"}

SCRIPT_VARS = {
    "CLASS_ID": "class_id", "CONF_THRESH": "conf", "Y_OFFSET": "y_offset",
    "CAM_DEV": "cam_dev", "CAM_FPS": "cam_fps", "MAX_SPEED": "max_speed",
    "AIM_KEY": "aim_key", "PREVIEW": "preview", "CAPTURE": "capture_enabled",
    "OUT_DIR": "capture_dir", "FIRE_MS": "fire_ms", "AUTO_S": "auto_s",
    "COOLDOWN_MS": "cooldown_ms", "JPEG_Q": "jpeg_q", "MODEL_PATH": "model",
}


def root_status(root: Path):
    """部署根有效性: 存在且含 scripts/game 结构。返回 (status, 错误说明)。"""
    if not root.is_dir():
        return "missing", "部署根不存在: %s" % root
    if not (root / "scripts").is_dir():
        return "invalid", "该目录不像部署根 (缺 scripts/): %s" % root
    if not (root / "scripts" / "game").is_dir():
        return "invalid", "缺 scripts/game/ (无游戏 profile 可启动)"
    return "ok", ""


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _relativize(p: Path, root: Path) -> str:
    """绝对路径且位于部署根内 → 相对 root 的 posix 串 (可移植); 否则原样。
    相对输入本身就是根相对约定, 禁止按服务端 CWD 解析。"""
    if not p.is_absolute():
        return p.as_posix()
    try:
        return p.resolve().relative_to(root.resolve()).as_posix()
    except (OSError, ValueError):
        return p.as_posix()


def _coerce(key: str, val: str, root: Path):
    d = PARAM_DEFS[key]
    try:
        if d["kind"] == "int":
            return int(_clamp(float(val), d["lo"], d["hi"]))
        if d["kind"] == "float":
            return _clamp(float(val), d["lo"], d["hi"])
        if d["kind"] == "bool":
            return val.strip().lower() in ("y", "yes", "true", "1")
        if d["kind"] == "enum":
            v = val.strip().lower()
            return v if v in d["choices"] else d["default"]
        if d["kind"] == "path":
            return _relativize(Path(val), root)
        if d["kind"] == "dsdir":
            return _relativize(Path(val), root)
    except (TypeError, ValueError):
        return d["default"]
    return val.strip()


def parse_script(path: Path, root: Path):
    """只读解析 game 脚本顶部 VAR=value 块。返回 (params, calib)。"""
    params = {k: d["default"] for k, d in PARAM_DEFS.items()}
    calib = {"s": None, "l": None}
    try:
        text = path.read_text(encoding="utf-8-sig", errors="replace")   # 脚本带 BOM
    except OSError:
        return params, calib
    for line in text.splitlines():
        line = re.split(r"\s#", line, maxsplit=1)[0].strip()            # 脚本约定: # 前有空格的行内注释
        m = VAR_RE.match(line)
        if not m:
            continue
        name, raw = m.group(1), m.group(2)
        v = raw.strip()
        if len(v) >= 2 and v[0] == v[-1] and v[0] in "\"'":
            v = v[1:-1]
        v = v.replace("$ROOT", str(root))
        if name in ("S_EST", "L_EST"):
            try:
                calib["s" if name == "S_EST" else "l"] = float(v)
            except ValueError:
                pass
            continue
        key = SCRIPT_VARS.get(name)
        if key is not None:
            params[key] = _coerce(key, v, root)
    return params, calib


def validate_params(user: dict, root: Path) -> dict:
    """UI 提交的参数 → 白名单化 + 类型化 + 钳制后的完整参数集 (非法项回退默认)。"""
    out = {k: d["default"] for k, d in PARAM_DEFS.items()}
    for k, v in (user or {}).items():
        if k not in PARAM_DEFS:
            continue
        d = PARAM_DEFS[k]
        try:
            if d["kind"] == "int":
                out[k] = int(_clamp(float(v), d["lo"], d["hi"]))
            elif d["kind"] == "float":
                out[k] = _clamp(float(v), d["lo"], d["hi"])
            elif d["kind"] == "bool":
                out[k] = bool(v)
            elif d["kind"] == "enum":
                out[k] = v if v in d["choices"] else d["default"]
            elif d["kind"] == "path":
                s = str(v or "").strip()
                if s and ".." not in Path(s).parts:
                    out[k] = _relativize(Path(s), root)
            elif d["kind"] == "dsdir":
                s = str(v or "").strip()
                if s and ".." not in Path(s).parts:
                    out[k] = _relativize(Path(s), root)
            else:
                out[k] = str(v).strip()
        except (TypeError, ValueError):
            pass
    return out


# ---------- profile JSON 持久化 (data/profiles/<脚本名>.json) ----------

def _profile_path(stem: str) -> Path:
    return config.PROFILE_DIR / (stem + ".json")


def load_profile(stem: str):
    p = _profile_path(stem)
    if not p.is_file():
        return None
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def save_profile(stem: str, data: dict) -> None:
    config.PROFILE_DIR.mkdir(parents=True, exist_ok=True)
    tmp = _profile_path(stem).with_suffix(".json.tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    tmp.replace(_profile_path(stem))        # 原子替换


def _same(a, b) -> bool:
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(float(a) - float(b)) < 1e-6
    return a == b


def scan_profiles(root: Path) -> list:
    out = []
    game_dir = root / "scripts" / "game"
    for p in sorted(game_dir.glob("*.sh")):
        if not SCRIPT_STEM_RE.match(p.stem):
            continue
        script_params, calib = parse_script(p, root)
        saved_doc = load_profile(p.stem)
        saved = (saved_doc or {}).get("params") or {}
        drift = []
        if saved_doc:
            # 只有保存过才有"漂移"概念; 首次见到脚本时 saved 尚未播种
            for k, d in PARAM_DEFS.items():
                sv = saved.get(k, d["default"])
                pv = script_params.get(k, d["default"])
                if not _same(sv, pv):
                    drift.append(k)
        out.append({
            "file": p.stem,
            "display_name": ((saved_doc or {}).get("display_name") or p.stem),
            "script_params": script_params,
            "calib": calib,                 # 标定值永远以脚本为权威 (固件 -S 回写目标)
            "saved": saved,
            "has_saved": saved_doc is not None,
            "drift": drift,
        })
    return out


# ---------- 其它发现 ----------

def list_engines(root: Path) -> list:
    out = []
    edir = root / "engine"
    if edir.is_dir():
        for p in sorted(edir.rglob("*.engine")):
            try:
                st = p.stat()
            except OSError:
                continue
            out.append({"path": p.relative_to(root).as_posix(),
                        "size": st.st_size, "mtime": st.st_mtime})
    return out


def list_onnx(root: Path) -> list:
    out = []
    odir = root / "onnx"
    if odir.is_dir():
        for p in sorted(odir.rglob("*.onnx")):
            rel = p.relative_to(root)
            eng = root / "engine" / rel.with_suffix(".engine")
            try:
                ost = p.stat()
                est = eng.stat() if eng.is_file() else None
            except OSError:
                continue
            out.append({"path": rel.as_posix(), "size": ost.st_size, "mtime": ost.st_mtime,
                        "engine": rel.with_suffix(".engine").as_posix() if est else None,
                        "stale": bool(est and ost.st_mtime > est.st_mtime)})
    return out


def list_cameras() -> list:
    """枚举 /dev/v4l/by-id/*-video-index0 → /dev/videoN;
    Hagibis/Asus 别名按固件 resolve_cam_device 同规则 (小写子串, 唯一命中) 测试后加入。"""
    by_id = Path("/dev/v4l/by-id")
    entries = []
    try:
        for p in sorted(by_id.iterdir()):
            if p.name.endswith("-video-index0"):
                try:
                    entries.append({"id": p.name, "node": str(p.resolve())})
                except OSError:
                    pass
    except OSError:
        return []
    cams = [{"value": e["node"], "label": "%s  (%s)" % (e["node"], e["id"]),
             "alias": False} for e in entries]
    for alias in ("Hagibis", "Asus"):
        hits = [e for e in entries if alias.lower() in e["id"].lower()]
        if len(hits) == 1:
            cams.insert(0, {"value": alias, "label": "%s → %s" % (alias, hits[0]["node"]),
                            "alias": True})
    return cams


def list_dataset_dirs(root: Path) -> list:
    ds = root / "dataset"
    if not ds.is_dir():
        return []
    try:
        return sorted(d.name for d in ds.iterdir() if d.is_dir())
    except OSError:
        return []


def binary_info(root: Path) -> dict:
    """bin/aimbot 存在性 + 热参能力探测 (二进制内查握手串, 用于认领实例的兜底)。"""
    p = root / "bin" / "aimbot"
    if not p.is_file():
        return {"exists": False, "hot_capable": False}
    hot = False
    needle = "热参数通道".encode("utf-8")
    try:
        with p.open("rb") as f:
            while True:
                chunk = f.read(1 << 20)
                if not chunk:
                    break
                if needle in chunk:
                    hot = True
                    break
        st = p.stat()
    except OSError:
        return {"exists": True, "hot_capable": hot}
    return {"exists": True, "hot_capable": hot, "size": st.st_size, "mtime": st.st_mtime}


def scan(root: Path) -> dict:
    """全量发现一次。永不抛异常 —— 任何失败转成 status/error/warnings 在页面可见。"""
    status, err = root_status(root)
    out = {"status": status, "error": err, "scanned_at": time.time(),
           "profiles": [], "engines": [], "onnx": [], "cameras": [],
           "dataset_dirs": [], "binary": {"exists": False, "hot_capable": False},
           "warnings": []}
    if status != "ok":
        return out
    out["profiles"] = scan_profiles(root)
    out["engines"] = list_engines(root)
    out["onnx"] = list_onnx(root)
    out["cameras"] = list_cameras()
    out["dataset_dirs"] = list_dataset_dirs(root)
    out["binary"] = binary_info(root)
    if not out["engines"]:
        out["warnings"].append("engine/ 下没有 *.engine —— 先在「模型与运维」页转换, 才能选择模型")
    if not out["profiles"]:
        out["warnings"].append("scripts/game/ 下没有 *.sh —— 没有可启动的游戏 profile")
    if not out["cameras"]:
        out["warnings"].append("未发现采集设备 (/dev/v4l/by-id 为空), 启动会被固件拒绝")
    if not out["binary"]["exists"]:
        out["warnings"].append("bin/aimbot 不存在 —— 先执行 compile, 否则无法启动")
    elif not out["binary"]["hot_capable"]:
        out["warnings"].append("bin/aimbot 是旧版 (无热参数通道) —— 建议重编译以启用热参数")
    return out
