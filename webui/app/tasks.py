"""运维任务: convert / compile 异步执行 (两者都慢, 绝不阻塞页面), 带日志流与历史。

同一时刻只允许一个运维任务 (互斥); compile 在实例运行中时生成"失败任务"说明原因
(运行中的二进制无法被覆盖, ETXTBSY), 而不是默默开始然后炸在链接阶段。
"""
import json
import shlex
import subprocess
import threading
import time
from collections import OrderedDict, deque
from pathlib import Path


class TaskManager:
    def __init__(self, keep: int = 20):
        self._lock = threading.RLock()
        self._tasks = OrderedDict()         # id → task dict (log 为 [(seq, line)] deque)
        self._keep = keep
        self._counter = 0

    def start(self, kind: str, script_path: Path, block_reason: str = None):
        """启动任务, 返回 (task_id 或 None, 错误)。block_reason 非空时生成一个直接失败的任务。"""
        with self._lock:
            for t in self._tasks.values():
                if t["status"] == "running":
                    return None, "已有运维任务在运行 (%s), 等它完成再试" % t["kind"]
            if not Path(script_path).is_file():
                return None, "脚本不存在: %s" % script_path
            self._counter += 1
            tid = "t%d_%d" % (int(time.time()), self._counter)
            task = {"id": tid, "kind": kind, "script": str(script_path),
                    "status": "running", "started_at": time.time(), "ended_at": None,
                    "exit_code": None, "seq": 0, "log": deque(maxlen=3000)}
            self._tasks[tid] = task
            while len(self._tasks) > self._keep:
                self._tasks.popitem(last=False)
        threading.Thread(target=self._run, args=(task, script_path, block_reason),
                         daemon=True).start()
        return tid, ""

    def _append(self, task: dict, line: str) -> None:
        with self._lock:
            task["seq"] += 1
            task["log"].append((task["seq"], line))

    def _run(self, task: dict, script_path: Path, block_reason: str) -> None:
        if block_reason:
            self._append(task, "✗ " + block_reason)
            with self._lock:
                task["status"] = "failed"
                task["ended_at"] = time.time()
            return
        self._append(task, "$ bash %s" % shlex.quote(str(script_path)))
        try:
            p = subprocess.Popen(["bash", str(script_path)],
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 stdin=subprocess.DEVNULL)
        except OSError as e:
            with self._lock:
                task["status"] = "failed"
                task["ended_at"] = time.time()
            self._append(task, "✗ 启动失败: %s" % e)
            return
        for raw in iter(p.stdout.readline, b""):
            self._append(task, raw.decode("utf-8", "replace").rstrip("\r\n"))
        try:
            p.stdout.close()
        except OSError:
            pass
        rc = p.wait()
        with self._lock:
            task["exit_code"] = rc
            task["status"] = "ok" if rc == 0 else "failed"
            task["ended_at"] = time.time()
        self._append(task, "✅ 完成" if rc == 0 else "✗ 失败 (退出码 %d)" % rc)

    def summaries(self) -> list:
        with self._lock:
            return [{"id": t["id"], "kind": t["kind"], "status": t["status"],
                     "started_at": t["started_at"], "ended_at": t["ended_at"],
                     "exit_code": t["exit_code"], "seq": t["seq"]}
                    for t in reversed(self._tasks.values())]

    def log_since(self, tid: str, seq: int) -> list:
        with self._lock:
            t = self._tasks.get(tid)
            if not t:
                return []
            return [(s, line) for (s, line) in t["log"] if s > seq]
