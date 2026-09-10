"""实例管理: 单实例状态机 + 启动序列 + 日志环 + 孤儿认领。

启动序列与 game 脚本完全同构: jetson_clocks → setup_mouse.sh → bin/aimbot
(全量参数, -S 指向 profile 脚本使标定回写照旧落进脚本)。euid==0 时直接执行
(推荐, systemd root 服务); 否则加 sudo 前缀 (需 NOPASSWD, 见 webui/README.md)。

状态机: stopped → starting → running → exited (→ stopped 由下次启动覆盖)。
异常退出 (非用户停止且退出码非 0) 置 abnormal, UI 横幅可查。
WebUI 自身重启时扫描 /proc 认领已存活的 aimbot (adopted, 日志不可见但可停止)。
"""
import json
import os
import re
import shlex
import signal
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

from . import discover

HANDSHAKE_RE = re.compile(r"热参数通道: 127\.0\.0\.1:(\d+)")
FPS_RE = re.compile(r"\[AI FPS\] (\d+) fps")
CALIB_RE = re.compile(r"\[标定\] s=([0-9.eE+-]+) px/count, L=([0-9.eE+-]+) ms")

RUNNING_STATES = ("starting", "running", "stopping")

STEP_NAMES = ("jetson_clocks 频率锁定", "USB Gadget 鼠标", "aimbot 进程")


def sudo_prefix():
    return [] if (hasattr(os, "geteuid") and os.geteuid() == 0) else ["sudo"]


def fmt_num(v):
    if isinstance(v, float) and float(v).is_integer():
        return str(int(v))
    return str(v)


def build_argv(root: Path, params: dict, calib: dict, script_path: Path) -> list:
    """拼装 aimbot 命令行 (与脚本同构)。calib 缺项时省略 -s/-l, 固件按默认兜底。"""
    model = str(params.get("model") or "")
    model_abs = model if os.path.isabs(model) else str(root / model)
    argv = [str(root / "bin" / "aimbot"),
            "-m", model_abs,
            "-c", fmt_num(params.get("class_id", 0)),
            "-t", fmt_num(params.get("conf", 0.5)),
            "-y", fmt_num(params.get("y_offset", 65.0)),
            "-d", str(params.get("cam_dev", "Asus")),
            "-f", fmt_num(params.get("cam_fps", 120)),
            "-x", fmt_num(params.get("max_speed", 1500.0)),
            "-S", str(script_path),
            "-k", str(params.get("aim_key", "both")),
            "-r", fmt_num(params.get("fov", 150.0)),
            "-v", "y" if params.get("preview") else "n"]
    if calib.get("s") is not None:
        argv += ["-s", fmt_num(calib["s"])]
    if calib.get("l") is not None:
        argv += ["-l", fmt_num(calib["l"])]
    if params.get("capture_enabled"):
        od = str(params.get("capture_dir") or "dataset")
        od_abs = od if os.path.isabs(od) else str(root / od)
        argv += ["-o", od_abs,
                 "-F", fmt_num(params.get("fire_ms", 800)),
                 "-A", fmt_num(params.get("auto_s", 10.0)),
                 "-C", fmt_num(params.get("cooldown_ms", 800)),
                 "-q", fmt_num(params.get("jpeg_q", 95))]
    return argv


def cmd_string(argv: list) -> str:
    return " ".join(shlex.quote(a) for a in argv)


def find_aimbot_pids(bin_path: str) -> list:
    """/proc 扫描 exe==bin_path 的进程 (精确路径匹配, 覆盖 SSH 手跑实例)。"""
    pids = []
    try:
        entries = list(Path("/proc").iterdir())
    except OSError:
        return pids
    for d in entries:
        if not d.name.isdigit():
            continue
        try:
            if os.path.realpath(str(d / "exe")) == bin_path:
                pids.append(int(d.name))
        except OSError:
            continue
    return sorted(pids)


def send_hot(port: int, wire: dict) -> None:
    """向固件热参通道发一条 UDP (fire-and-forget; 生效回执看日志 [热参] 行)。"""
    import socket
    payload = ";".join("%s=%s" % (k, v) for k, v in wire.items()).encode("utf-8")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.sendto(payload, ("127.0.0.1", int(port)))
    finally:
        s.close()


class InstanceManager:
    def __init__(self, history_path: Path):
        self._hist_path = history_path
        self._lock = threading.RLock()
        self._log = deque(maxlen=5000)      # [(seq, line)]
        self._seq = 0
        self._proc = None
        self._user_stop = False
        self.state = "stopped"
        self.profile = None                 # 脚本 stem
        self.display_name = None
        self.pid = None
        self.started_at = None              # wall time (s)
        self.ended_at = None
        self.exit_code = None
        self.exit_signal = None
        self.abnormal = False
        self.adopted = False                # True = 非 WebUI 启动, 被认领
        self.error = None
        self.steps = []
        self.hot_capable = False
        self.hot_port = None
        self.fps = None
        self.calib_live = None              # 运行中标定回执 {"s","l"}
        self.history = self._load_history()

    # ---------- 日志 ----------

    def _append_log(self, line: str) -> None:
        with self._lock:
            self._seq += 1
            self._log.append((self._seq, line))

    def log_since(self, seq: int) -> list:
        with self._lock:
            return [(s, t) for (s, t) in self._log if s > seq]

    def log_all(self) -> str:
        with self._lock:
            return "\n".join(t for (_, t) in self._log)

    # ---------- 历史记录 ----------

    def _load_history(self) -> list:
        try:
            return json.loads(self._hist_path.read_text(encoding="utf-8"))[-20:]
        except (OSError, ValueError):
            return []

    def _push_history(self, entry: dict) -> None:
        self.history = (self.history + [entry])[-20:]
        try:
            self._hist_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._hist_path.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(self.history, ensure_ascii=False, indent=1),
                           encoding="utf-8")
            tmp.replace(self._hist_path)
        except OSError:
            pass

    # ---------- 快照 ----------

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "state": self.state, "profile": self.profile,
                "display_name": self.display_name, "pid": self.pid,
                "started_at": self.started_at, "ended_at": self.ended_at,
                "exit_code": self.exit_code, "exit_signal": self.exit_signal,
                "abnormal": self.abnormal, "adopted": self.adopted,
                "error": self.error, "steps": [dict(s) for s in self.steps],
                "hot_capable": self.hot_capable, "hot_port": self.hot_port,
                "fps": self.fps, "calib_live": self.calib_live,
                "log_seq": self._seq,
            }

    # ---------- 启动 ----------

    def start(self, root: Path, profile: str, display_name: str,
              params: dict, calib: dict, script_path: Path):
        with self._lock:
            if self.state in RUNNING_STATES:
                return False, "已有实例在启动或运行 (先停止, 或直接再点【启动】= 以新设置重启)"
            bin_path = root / "bin" / "aimbot"
            if not bin_path.is_file():
                return False, "bin/aimbot 不存在 —— 先在「模型与运维」页编译"
            model = params.get("model")
            if not model:
                return False, "未选择模型 engine"
            model_abs = model if os.path.isabs(model) else str(root / model)
            if not Path(model_abs).is_file():
                return False, "模型不存在: %s (先 convert 或重选)" % model
            argv = build_argv(root, params, calib, script_path)
            self._user_stop = False
            self.state = "starting"
            self.profile = profile
            self.display_name = display_name
            self.adopted = False
            self.error = None
            self.exit_code = None
            self.exit_signal = None
            self.abnormal = False
            self.ended_at = None
            self.started_at = None
            self.fps = None
            self.calib_live = None
            self.steps = [{"name": n, "status": "pending", "detail": "", "ms": 0}
                          for n in STEP_NAMES]
            self._append_log("════ 启动 %s (%s) ════" % (display_name, profile))
            self._append_log("$ " + cmd_string(argv))
        threading.Thread(target=self._run, args=(root, argv), daemon=True).start()
        return True, ""

    def _run_step(self, cmd: list, timeout: int):
        try:
            p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               stdin=subprocess.DEVNULL, timeout=timeout)
            return p.returncode, (p.stdout or b"").decode("utf-8", "replace").strip()
        except FileNotFoundError:
            return 127, "命令未找到: %s" % cmd[-1]
        except subprocess.TimeoutExpired:
            return 124, "超时 (%ds)" % timeout
        except OSError as e:
            return 126, str(e)

    def _run(self, root: Path, argv: list) -> None:
        pre = sudo_prefix()
        # ① jetson_clocks —— 失败不阻断 (与脚本行为一致, 只警示)
        self.steps[0]["status"] = "running"
        t0 = time.time()
        if self._check_user_stop():
            return
        rc, out = self._run_step(pre + ["jetson_clocks"], 90)
        self.steps[0]["ms"] = int((time.time() - t0) * 1000)
        self.steps[0]["status"] = "ok" if rc == 0 else "warn"
        self.steps[0]["detail"] = out[-200:] if rc == 0 else ("jetson_clocks 失败 (rc=%s), 已跳过: %s" % (rc, out[-160:]))
        if rc != 0:
            self._append_log("⚠ jetson_clocks 失败 (rc=%s): %s" % (rc, out))
        # ② setup_mouse.sh —— 失败则中止 (没有 hidg0 起进程必然失败)
        self.steps[1]["status"] = "running"
        t0 = time.time()
        if self._check_user_stop():
            return
        rc, out = self._run_step(pre + ["bash", str(root / "scripts" / "setup_mouse.sh")], 60)
        self.steps[1]["ms"] = int((time.time() - t0) * 1000)
        if rc != 0:
            self.steps[1]["status"] = "fail"
            self.steps[1]["detail"] = out[-200:]
            self._append_log("✗ setup_mouse.sh 失败 (rc=%s):\n%s" % (rc, out))
            return self._finish_error("setup_mouse.sh 失败 (rc=%s), 启动中止" % rc)
        self.steps[1]["status"] = "ok"
        self.steps[1]["detail"] = out[-160:]
        # ③ aimbot
        self.steps[2]["status"] = "running"
        if self._check_user_stop():
            return
        try:
            proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, cwd=str(root))
        except OSError as e:
            self.steps[2]["status"] = "fail"
            self.steps[2]["detail"] = str(e)
            return self._finish_error("无法启动 aimbot: %s" % e)
        with self._lock:
            self._proc = proc
            self.pid = proc.pid
            self.state = "running"
            self.started_at = time.time()
        self._append_log("✅ aimbot 已启动 (pid %d)" % proc.pid)
        threading.Thread(target=self._pump, args=(proc,), daemon=True).start()

    def _check_user_stop(self) -> bool:
        """启动序列中途被叫停 → 干净回退到 stopped。"""
        with self._lock:
            if not self._user_stop:
                return False
            self.state = "stopped"
            for s in self.steps:
                if s["status"] in ("pending", "running"):
                    s["status"] = "fail"
                    s["detail"] = "已被用户取消"
            self._append_log("■ 启动被用户取消")
            return True

    def _finish_error(self, msg: str) -> None:
        with self._lock:
            self.state = "exited"
            self.error = msg
            self.ended_at = time.time()
            self.abnormal = True
            self.steps[2]["status"] = "fail"
            self.steps[2]["detail"] = msg
            self._append_log("✗ " + msg)

    def _pump(self, proc: subprocess.Popen) -> None:
        """读子进程 stdout/stderr 合并流 → 日志环 + 状态解析 (握手/FPS/标定)。"""
        for raw in iter(proc.stdout.readline, b""):
            line = raw.decode("utf-8", "replace").rstrip("\r\n")
            m = HANDSHAKE_RE.search(line)
            if m:
                self.hot_capable = True
                self.hot_port = int(m.group(1))
            m = FPS_RE.search(line)
            if m:
                self.fps = int(m.group(1))
            m = CALIB_RE.search(line)
            if m:
                try:
                    self.calib_live = {"s": float(m.group(1)), "l": float(m.group(2))}
                except ValueError:
                    pass
            self._append_log(line)
        try:
            proc.stdout.close()
        except OSError:
            pass
        rc = proc.wait()
        with self._lock:
            self._proc = None
            self.pid = None
            self.ended_at = time.time()
            self.exit_code = rc if rc >= 0 else None
            self.exit_signal = -rc if rc < 0 else None
            self.state = "exited"
            self.abnormal = (not self._user_stop) and rc != 0
            duration = (self.ended_at - self.started_at) if self.started_at else 0
            if self._user_stop:
                self.steps[2]["status"] = "ok"
                self.steps[2]["detail"] = "用户停止 (退出码 %s)" % rc
            elif rc == 0:
                self.steps[2]["status"] = "ok"
                self.steps[2]["detail"] = "正常退出"
            else:
                self.steps[2]["status"] = "fail"
                sig = (" (信号 %s)" % self.exit_signal) if self.exit_signal else ""
                self.steps[2]["detail"] = "异常退出: rc=%s%s" % (rc, sig)
            self._push_history({
                "ts": self.ended_at, "profile": self.profile,
                "display_name": self.display_name,
                "exit_code": self.exit_code, "exit_signal": self.exit_signal,
                "abnormal": self.abnormal, "duration_s": round(duration, 1),
            })

    # ---------- 停止 ----------

    def stop(self, root: Path):
        """异步停止: 杀掉 aimbot 进程 (含认领/SSH 手跑的); 完成后状态经 pump 或兜底落定。"""
        with self._lock:
            if self.state not in RUNNING_STATES:
                return False, "实例未在运行"
            self._user_stop = True
            self.state = "stopping"
        threading.Thread(target=self._do_stop, args=(root,), daemon=True).start()
        return True, ""

    def _do_stop(self, root: Path) -> None:
        self.kill_all(root)
        # 认领实例没有 pump 线程, 收尾在这里落定; 自启实例由 pump 的 wait() 先到先落
        time.sleep(1.0)
        with self._lock:
            if self.state != "stopping":
                return
            self.state = "exited"
            self.ended_at = time.time()
            self.pid = None
            self.exit_code = None
            self.exit_signal = signal.SIGTERM
            self.abnormal = False
            self.steps = [{"name": "停止", "status": "ok",
                           "detail": "已停止 (认领实例, 无退出码)", "ms": 0}]
            self._append_log("■ 已停止 (认领实例)")
            self._push_history({
                "ts": self.ended_at, "profile": self.profile,
                "display_name": self.display_name,
                "exit_code": None, "exit_signal": signal.SIGTERM,
                "abnormal": False, "duration_s": None,
            })

    # ---------- 孤儿认领 ----------

    def adopt(self, root: Path, hot_port_default: int) -> bool:
        bin_path = str(root / "bin" / "aimbot")
        pids = find_aimbot_pids(bin_path)
        if not pids:
            return False
        info = discover.binary_info(root)
        with self._lock:
            self.state = "running"
            self.adopted = True
            self.profile = None
            self.display_name = None
            self.pid = pids[0]
            self.started_at = None
            self.error = None
            self.hot_capable = bool(info.get("hot_capable"))
            self.hot_port = hot_port_default if self.hot_capable else None
            self.steps = [{"name": "孤儿认领", "status": "ok",
                           "detail": "发现已运行的 aimbot (pid %s, 非本 WebUI 启动): 日志不可见; "
                                     "按【启动】将以当前设置接管" % ",".join(map(str, pids)),
                           "ms": 0}]
            self._append_log("════ 认领已运行实例 pid %s (非本 WebUI 启动) ════" % ",".join(map(str, pids)))
        return True

    def kill_all(self, root: Path) -> int:
        """杀掉所有 aimbot 进程 (启动前清场)。返回杀掉的数量。"""
        bin_path = str(root / "bin" / "aimbot")
        pids = find_aimbot_pids(bin_path)
        if not pids:
            return 0
        self._append_log("清理在跑实例: pid %s" % ",".join(map(str, pids)))
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                pass
        deadline = time.time() + 5
        while time.time() < deadline:
            if not [p for p in pids if Path("/proc/%d" % p).exists()]:
                break
            time.sleep(0.2)
        for pid in pids:
            try:
                if Path("/proc/%d" % pid).exists():
                    os.kill(pid, signal.SIGKILL)
                    self._append_log("pid %d SIGTERM 未退, 已 SIGKILL" % pid)
            except OSError:
                pass
        return len(pids)
