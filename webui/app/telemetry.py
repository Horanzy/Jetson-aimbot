"""宿主遥测: /proc CPU/内存, 热区温度, tegrastats GPU 负载 (尽力而为)。

全部做成"读不到就返回 None": 非 Linux (Windows 开发镜像) 或字段缺失时
前端隐藏对应卡片, 不报错不阻塞。帧率不在这里 —— 它来自固件日志 [AI FPS] 行,
由 InstanceManager 解析。
"""
import re
import shutil
import subprocess
import threading
from pathlib import Path

_cpu_prev = None
_gpu_load = None
_gpu_lock = threading.Lock()


def cpu_percent():
    """两次调用间的 CPU 占用率 (第一次无前置样本, 返回 None)。"""
    global _cpu_prev
    fields = None
    try:
        with open("/proc/stat") as f:
            for line in f:
                if line.startswith("cpu "):
                    fields = [int(x) for x in line.split()[1:8]]
                    break
    except (OSError, ValueError, IndexError):
        return None
    if fields is None:
        return None
    idle = fields[3] + fields[4]
    total = sum(fields)
    if _cpu_prev is None:
        _cpu_prev = (idle, total)
        return None
    di, dt = idle - _cpu_prev[0], total - _cpu_prev[1]
    _cpu_prev = (idle, total)
    if dt <= 0:
        return None
    return round(100.0 * (1.0 - float(di) / dt), 1)


def mem_info():
    try:
        info = {}
        with open("/proc/meminfo") as f:
            for line in f:
                k, _, v = line.partition(":")
                if k in ("MemTotal", "MemAvailable"):
                    info[k] = int(v.strip().split()[0])
        total, avail = info.get("MemTotal"), info.get("MemAvailable")
        if total and avail is not None:
            return {"total_mb": total // 1024,
                    "used_mb": (total - avail) // 1024,
                    "percent": round(100.0 * (total - avail) / total, 1)}
    except (OSError, ValueError, IndexError):
        pass
    return None


def temps() -> list:
    out = []
    base = Path("/sys/class/thermal")
    try:
        zones = sorted(base.glob("thermal_zone*"))
    except OSError:
        return out
    for z in zones:
        try:
            t = int((z / "temp").read_text().strip())
            name = (z / "type").read_text().strip()
            out.append({"zone": z.name, "type": name, "temp": t / 1000.0})
        except (OSError, ValueError):
            continue
    return out


def _tegrastats_loop() -> None:
    global _gpu_load
    try:
        p = subprocess.Popen(["tegrastats", "--interval", "2000"],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             stdin=subprocess.DEVNULL)
    except OSError:
        return
    for raw in iter(p.stdout.readline, b""):
        m = re.search(rb"GR3D_FREQ (\d+)%", raw)
        if m:
            with _gpu_lock:
                _gpu_load = int(m.group(1))
    p.kill()


def start_tegrastats() -> None:
    """Jetson 专属 GPU 负载源; 找不到命令就静默放弃 (前端隐藏 GPU 卡片)。"""
    if shutil.which("tegrastats"):
        threading.Thread(target=_tegrastats_loop, daemon=True).start()


def gpu_percent():
    with _gpu_lock:
        return _gpu_load


def snapshot() -> dict:
    zs = temps()
    return {
        "cpu": cpu_percent(),
        "mem": mem_info(),
        "gpu": gpu_percent(),
        "temps": zs[:6],
        "soc_temp": max((z["temp"] for z in zs), default=None),
    }
