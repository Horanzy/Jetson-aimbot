"""结构化发现: 从部署根按约定目录派生 游戏 profile / 模型 / 转换源 / 采集设备。

只按固定结构发现 (scripts/game/, engine/, onnx/, /dev/v4l/by-id/), 不递归扫全盘。
**脚本 = 唯一事实源**: profile 参数就是 game 脚本头部的 VAR=value 块, 每次扫描现场解析,
没有独立存储; WebUI 的【保存】通过 write_script_params 原子写回脚本 (只改目标变量的值,
注释/引号风格/其余行逐字保留)。脚本把手改量写成 `${VAR:-默认}` 守卫 (缺行也能起), 所以
读取时解析到守卫里的有效默认值, 见到的是值而不是字面量。唯一的另一处脚本写回是固件经
-S 的标定回写机制。
"""
import json
import os
import re
import time
from pathlib import Path

from . import config

VAR_RE = re.compile(r"^\s*([A-Z_][A-Z0-9_]*)\s*=\s*(.*?)\s*$")
SCRIPT_STEM_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")
# 输出模式: 三套输出互斥, 各自独占 UDC (顺序也是 UI 里分组的顺序)
OUTPUT_MODES = ("hid", "pad", "p5g")
# 模板约定: 每个手改量写成 ${VAR:-默认}, 缺行也能起。读脚本时必须解析到**有效值**,
#   否则 UI 看到的是 ${CLASS_ID:-0} 这样的字面量而不是 0。
_GUARD_RE = re.compile(r"^\$\{[A-Za-z_][A-Za-z0-9_]*:-?(.*)\}$", re.S)

# 参数定义: 类型/范围与服务端校验 (与 CLI/固件钳制一致, 双保险)
PARAM_DEFS = {
    "model":           dict(kind="path",  default=None),
    "class_id":        dict(kind="int",   lo=0, hi=255, default=0),
    "conf":            dict(kind="float", lo=0.0, hi=1.0, default=0.5),
    "y_offset":        dict(kind="float", lo=0.0, hi=100.0, default=65.0),
    "cam_dev":         dict(kind="str",   default="Asus"),
    "cam_fps":         dict(kind="int",   lo=1, hi=240, default=120),
    "max_speed":       dict(kind="float", lo=100.0, hi=20000.0, default=1500.0),
    # 输出模式 (冷: 它决定整条输出后端, 运行中不可换) 与各模式的设备选择
    "output_mode":     dict(kind="enum",  choices=OUTPUT_MODES, default="hid"),
    "mouse_keyword":   dict(kind="str",   default=""),
    "pad_keyword":     dict(kind="str",   default=""),
    # 手柄触发阈值 (% 满量程, RT/LT 共享; 只门控触发判定, 扳机模拟量仍 1:1 透传)
    "pad_trig_thr":    dict(kind="float", lo=0.0, hi=100.0, default=6.0),
    "pad_dump":        dict(kind="bool",  default=False),
    "aim_key":         dict(kind="enum",  choices=("fire", "ads", "both"), default="both"),
    "aim_enabled":     dict(kind="bool",  default=True),
    "fov":             dict(kind="float", lo=10.0, hi=1000.0, default=150.0),
    "preview":         dict(kind="bool",  default=False),
    "capture_enabled": dict(kind="bool",  default=False),
    "cap_fire":        dict(kind="bool",  default=True),
    "cap_det":         dict(kind="bool",  default=True),
    "cap_auto":        dict(kind="bool",  default=True),
    "capture_dir":     dict(kind="dsdir", default=None),
    "fire_ms":         dict(kind="int",   lo=50, hi=60000, default=800),
    "auto_s":          dict(kind="float", lo=1.0, hi=3600.0, default=10.0),
    "cooldown_ms":     dict(kind="int",   lo=0, hi=60000, default=800),
    "jpeg_q":          dict(kind="int",   lo=1, hi=100, default=95),
}
# 三套输出各占一组 5 个值: 延迟 1 + 拉枪倍率 4。分槽是必须的 —— 同一个倍率数在两套
#   口径下含义不同 (hid 基线 1.0 px/count, 手柄基线 3000 px/s 满偏屏速), 延迟也各是一个
#   物理量 (p5g 还多一跳真加密狗的签名往返), 共用一套等于让一个模式的取值去服务另一套
#   基线。切换 output_mode 就是整套自动换, 保存/标定回写只落进本模式那一格。
# 倍率刻度: 整数, 100 = 基线, 与有效灵敏度成反比 (有效灵敏度 = 基线/(倍率/100)),
#   范围 = 固件 spd_clamp 的夹取带 (防误输入), 有意义的带是 5..2000。
SPEED_SLOTS = (("spd_x", "SPDX"), ("spd_y", "SPDY"),
               ("ads_spd_x", "ADS_SPDX"), ("ads_spd_y", "ADS_SPDY"))
for _m in OUTPUT_MODES:
    PARAM_DEFS["l_%s" % _m] = dict(kind="float", lo=0.0, hi=120.0, default=60.0)
    for _a, _v in SPEED_SLOTS:
        PARAM_DEFS["%s_%s" % (_m, _a)] = dict(kind="int", lo=1, hi=10000, default=100)
# 热参数白名单: param key → 固件通道 key (对应 src/io/hotctl.cu hotctl_thread)。
#   固件侧的倍率热参只有一套 (spdx/spdy/adsspdx/adsspdy, 作用于运行中实例的当前输出
#   模式), 所以槽键→线上键的映射按模式取 (mode_spd_keys): 保存 pad 槽而实例跑 hid 时,
#   改的是下次启动的取值, 不外发热参。
HOT_WIRE_KEYS = {"conf": "t", "y_offset": "y", "max_speed": "x", "fov": "fov", "aim_key": "k",
                 "aim_enabled": "aim", "cap_fire": "cap_fire", "cap_det": "cap_det",
                 "cap_auto": "cap_auto", "pad_trig_thr": "padthr"}


def mode_spd_keys(mode) -> dict:
    """某输出模式的四个倍率参数键 → 热参线上键 (模式是槽唯一的身份)。"""
    m = mode if mode in OUTPUT_MODES else "hid"
    return {"%s_%s" % (m, a): w for a, w in
            (("spd_x", "spdx"), ("spd_y", "spdy"),
             ("ads_spd_x", "adsspdx"), ("ads_spd_y", "adsspdy"))}


SCRIPT_VARS = {
    "CLASS_ID": "class_id", "CONF_THRESH": "conf", "Y_OFFSET": "y_offset",
    "CAM_DEV": "cam_dev", "CAM_FPS": "cam_fps", "MAX_SPEED": "max_speed",
    "OUTPUT_MODE": "output_mode",                        # 冷: 决定整条输出后端
    "MOUSE_KEYWORD": "mouse_keyword",                    # hid: -D
    "PAD_KEYWORD": "pad_keyword",                        # pad/p5g: -P
    "PAD_TRIG_THR": "pad_trig_thr",                      # pad/p5g: -T (热参 padthr)
    "PAD_DUMP": "pad_dump",                              # pad/p5g: --pad-dump
    "AIM_KEY": "aim_key", "AIM_ENABLED": "aim_enabled", "PREVIEW": "preview",
    "CAPTURE": "capture_enabled", "CAP_FIRE": "cap_fire", "CAP_DET": "cap_det",
    "CAP_AUTO": "cap_auto", "OUT_DIR": "capture_dir", "FIRE_MS": "fire_ms",
    "AUTO_S": "auto_s", "COOLDOWN_MS": "cooldown_ms", "JPEG_Q": "jpeg_q",
    "MODEL_PATH": "model", "FOV_R": "fov",
}
# 三套输出的 5 个值各自的 VAR 名: <模式>_L_EST (固件标定回写目标) + <模式>_SPDX/SPDY/
#   ADS_SPDX/ADS_SPDY; 模式前缀是必需的身份 —— 同一个数在两套口径下含义不同, 所以每个值
#   只有一个拼写, 且它自带模式。
for _m in OUTPUT_MODES:
    SCRIPT_VARS["%s_L_EST" % _m.upper()] = "l_%s" % _m
    for _a, _v in SPEED_SLOTS:
        SCRIPT_VARS["%s_%s" % (_m.upper(), _v)] = "%s_%s" % (_m, _a)
# 固件标定回写量 (脚本 VAR → 参数键): 标定只写延迟, 速度倍率是手动项。三套输出各一条,
#   互不覆盖 (delay 是三者各自的物理量: p5g 的回路还含一跳加密狗签名往返)。
CALIB_VARS = {"%s_L_EST" % m.upper(): "l_%s" % m for m in OUTPUT_MODES}
# 模式 → 本模式那条延迟的键 (冷参数 output_mode 决定, 与固件 -l 的来源同一条规则)
MODE_L_KEY = {m: "l_%s" % m for m in OUTPUT_MODES}


def mode_l_key(output_mode) -> str:
    return MODE_L_KEY.get(str(output_mode or "hid"), "l_hid")


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
    相对输入本身就是根相对约定, 禁止按服务端 CWD 解析。

    判定是**词法**的 (normpath 后比前缀), 不解析符号链接: 部署根里的 engine/ 指到别处
    是合法部署形态, 按 resolve() 判会把根内路径说成根外路径, 读侧与写侧就此不一致。"""
    if not p.is_absolute():
        return p.as_posix()
    root_s = os.path.normpath(str(root))
    p_s = os.path.normpath(str(p))
    if p_s == root_s:
        return "."
    if p_s.startswith(root_s + os.sep):
        return p_s[len(root_s) + 1:].replace(os.sep, "/")
    return Path(p_s).as_posix()


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
    """只读解析 game 脚本顶部 VAR=value 块 → 参数集。

    延迟与四个倍率一样是**可编辑参数** (三套输出各占一槽), 只是它的落点同时是固件标定
    的回写目标 (CALIB_VARS) —— 同一份值只解析一次, 没有第二处存储。"""
    params = {k: d["default"] for k, d in PARAM_DEFS.items()}
    try:
        text = path.read_text(encoding="utf-8-sig", errors="replace")   # 脚本带 BOM
    except OSError:
        return params
    for line in text.splitlines():
        line = re.split(r"\s#", line, maxsplit=1)[0].strip()            # 脚本约定: # 前有空格的行内注释
        m = VAR_RE.match(line)
        if not m:
            continue
        name, raw = m.group(1), m.group(2)
        v = raw.strip()
        if len(v) >= 2 and v[0] == v[-1] and v[0] in "\"'":
            v = v[1:-1]
        g = _GUARD_RE.match(v)              # ${VAR:-默认} → 有效默认 (模板的守卫写法)
        if g:
            v = g.group(1)
        v = v.replace("$ROOT", str(root))
        key = SCRIPT_VARS.get(name)
        if key is not None:
            params[key] = _coerce(key, v, root)
    return params


def validate_params(user: dict, root: Path, partial: bool = False) -> dict:
    """UI 提交的参数 → 白名单化 + 类型化 + 钳制后的参数集 (非法项回退默认)。

    partial=True 只回传提交里出现的键 (逐项校验后就地夹取), 用于【保存】的补丁语义:
    没提交的键保持脚本现值, 不套默认值 —— 否则一次只改 spd 的提交会把其余 VAR 全部
    打回默认。"""
    out = {} if partial else {k: d["default"] for k, d in PARAM_DEFS.items()}
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


# ---------- 脚本写回 (脚本 = 唯一事实源) ----------

_ASSIGN_RE = re.compile(r"^(\s*)([A-Z_][A-Z0-9_]*)(\s*=\s*)(.*)$")
_GUARD_NAME_RE = re.compile(r"^\$\{([A-Za-z_][A-Za-z0-9_]*):-?")
# 能原样放进 ${VAR:-…} 默认位的字面量 (无空白, 只含数字/字母与 _./$,:@%+-):
#   模板里的路径 ($ROOT/…) 与数值都合得上。合不上就退回裸值 —— 守卫只是模板的写法,
#   不能为保住它把值写坏。
_GUARD_SAFE_RE = re.compile(r"^[A-Za-z0-9_./$,:@%+-]*$")
VAR_OF = {v: k for k, v in SCRIPT_VARS.items()}


def _fmt_value(key: str, val, root: Path) -> str:
    """参数值 → 脚本里的字面量 (整数不带小数点; bool 用 y/n; 路径一律挂 $ROOT)。

    脚本尾部以路径变量直接展开传参, 相对路径会随 cwd 漂 —— 所以部署根内的路径一律写成
    `$ROOT/<相对>`, 解析时再还原。这一对是往返恒等的, 一次保存不会把用户写好的
    `$ROOT/engine/x.engine` 改写成绝对路径 (那会让"只改目标 VAR"变成两处改动)。

    延迟是唯一的浮点手改量, 写法固定一位小数 (= 固件标定回写的格式): 它是往返恒等的,
    于是保存 pad 槽时 hid/p5g 槽的延迟行逐字不动。"""
    d = PARAM_DEFS[key]
    if key in CALIB_VARS.values():          # 三套输出各一槽延迟 (L_MIN..L_MAX)
        return "%.1f" % float(val)
    if d["kind"] == "int":
        return str(int(val))
    if d["kind"] == "float":
        f = float(val)
        return str(int(f)) if f.is_integer() else repr(f)
    if d["kind"] == "bool":
        return "y" if val else "n"
    if d["kind"] in ("path", "dsdir"):
        s = str(val or "")
        if not s:
            return s
        # 与 _relativize 同一套词法规则, 保证 render(parse("$ROOT/x")) == "$ROOT/x":
        # 相对值 (含根内绝对路径被归一后的结果) 挂 $ROOT, 根外绝对路径原样。
        rel = _relativize(Path(s), root)
        return rel if rel.startswith("/") else "$ROOT/" + rel
    return str(val)


def write_script_params(path: Path, params: dict, root: Path) -> None:
    """把 params 原子写回脚本头部的 VAR=value 块。

    只改目标变量的值: 行内注释、引号风格、其余每一行逐字保留;
    脚本里缺失的变量追加到最后一个已知变量行之后; 执行位不变。

    两条例外规则, 都是"不破坏脚本"优先:
    - 值为 None 的项跳过 (我们手上没有这个值, 不能把用户的模型路径/输出目录抹成空串);
    - 原行写成 `${VAR:-默认}` 守卫时照原样保回 (值落在默认位上), 模板的写法在一次网页
      保存后仍然成立。

    权限/属主随原子替换一起带回来: 服务通常以 root 跑, 不还属主的话一次保存就把脚本变成
    root 所有, 用户 SSH 上就再也改不动自己的启动脚本了。"""
    st = path.stat()
    raw = path.read_bytes()
    has_bom = raw.startswith(b"\xef\xbb\xbf")
    text = raw.decode("utf-8-sig", errors="replace")
    trailing_nl = text.endswith("\n")
    body = text[:-1].split("\n") if trailing_nl else text.split("\n")

    seen = {}                            # VAR → 行号
    last_var_idx = -1
    for i, line in enumerate(body):
        m = _ASSIGN_RE.match(line)
        if m and m.group(2) in SCRIPT_VARS:   # SCRIPT_VARS 的键就是 VAR 名
            seen[m.group(2)] = i
            last_var_idx = i

    def render(key: str, quote: str, guard: str = "") -> str:
        s = _fmt_value(key, params[key], root)
        if guard and _GUARD_SAFE_RE.match(s):
            s = "${%s:-%s}" % (guard, s)
        return quote + s + quote if quote else s

    missing = []
    for key in params:
        var = VAR_OF.get(key)              # 参数名 → VAR 名 (SCRIPT_VARS 的逆映射)
        if var is None or params[key] is None:
            continue
        if var not in seen:
            missing.append(key)
            continue
        i = seen[var]
        m = _ASSIGN_RE.match(body[i])
        rhs = m.group(4)
        cm = re.search(r"\s+#", rhs)     # 脚本约定: 值后空白 + # 才是行内注释 (\s+ 保住对齐空格)
        comment = rhs[cm.start():] if cm else ""
        vs = (rhs if not cm else rhs[:cm.start()]).strip()
        quote = vs[0] if len(vs) >= 2 and vs[0] == vs[-1] and vs[0] in "\"'" else ""
        inner = vs[1:-1] if quote else vs
        gm = _GUARD_NAME_RE.match(inner) if quote else None
        guard = gm.group(1) if (gm and gm.group(1) == var) else ""
        body[i] = m.group(1) + m.group(2) + m.group(3) + render(key, quote, guard) + comment

    if missing:
        ins = []
        for k in missing:
            v = VAR_OF[k]
            s = _fmt_value(k, params[k], root)
            if _GUARD_SAFE_RE.match(s):
                s = "${%s:-%s}" % (v, s)
            ins.append('%s="%s"' % (v, s))
        if last_var_idx >= 0:
            body[last_var_idx + 1:last_var_idx + 1] = ins
        else:
            body.extend(ins)

    out = "\n".join(body) + ("\n" if trailing_nl else "")
    data = (b"\xef\xbb\xbf" if has_bom else b"") + out.encode("utf-8")
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)
    os.chmod(path, st.st_mode)           # tmp 是新文件, 执行位要显式带回来
    try:
        os.chown(path, st.st_uid, st.st_gid)
    except (AttributeError, OSError):
        pass                             # 非 root 且文件不属自己: 保持现状, 不改写流程结果


def _same(a, b) -> bool:
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(float(a) - float(b)) < 1e-6
    return a == b


def scan_profiles(root: Path) -> list:
    """每次扫描现场解析脚本 —— 没有独立参数存储, SSH 侧改动天然可见。"""
    out = []
    game_dir = root / "scripts" / "game"
    for p in sorted(game_dir.glob("*.sh")):
        if not SCRIPT_STEM_RE.match(p.stem):
            continue
        script_params = parse_script(p, root)
        out.append({
            "file": p.stem,
            "display_name": p.stem,
            "script_params": script_params,   # 三套输出的 5 个值都在里面 (脚本 = 唯一事实源)
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


_bin_info_cache = {}                     # (path, mtime_ns, size) → info; 重编译才失效


def binary_info(root: Path) -> dict:
    """bin/aimbot 存在性 + 热参能力探测 (二进制内查握手串, 用于认领实例的兜底)。
    结果按 (mtime, size) 缓存 —— WS 周期重扫时避免反复读大二进制。"""
    p = root / "bin" / "aimbot"
    if not p.is_file():
        return {"exists": False, "hot_capable": False}
    try:
        st = p.stat()
    except OSError:
        return {"exists": True, "hot_capable": False}
    key = (str(p), st.st_mtime_ns, st.st_size)
    hit = _bin_info_cache.get(key)
    if hit is not None:
        return hit
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
    except OSError:
        return {"exists": True, "hot_capable": hot}
    info = {"exists": True, "hot_capable": hot, "size": st.st_size, "mtime": st.st_mtime}
    _bin_info_cache.clear()
    _bin_info_cache[key] = info
    return info


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
