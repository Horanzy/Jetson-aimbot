"""HTTP API + WebSocket 推送。

鉴权: 除首页/静态资源外全部要求 token (X-WebUI-Token 头或 ?token=), WS 走 ?token=。
推送: 单条 WS 轮询合流 —— 实例快照+任务摘要每 0.4s、日志按序号增量、遥测每 2s;
客户端断线重连后用 /api/state 全量重建, 再以 log_seq 续传。
"""
import asyncio
import json
import os
import secrets
import time
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from . import config, discover, proc, tasks, telemetry

STATIC_DIR = config.WEBUI_DIR / "static"


def _is_root() -> bool:
    return hasattr(os, "geteuid") and os.geteuid() == 0


class WebUIState:
    """进程级单例。uvicorn 必须单 worker (README 说明)。"""

    def __init__(self):
        self.cfg = config.load()
        self._scan = None
        self.inst = proc.InstanceManager(config.HISTORY_PATH)
        self.op = tasks.TaskManager()

    def root(self) -> Path:
        return Path(self.cfg["deploy_root"])

    def scan(self) -> dict:
        if self._scan is None:
            self._scan = discover.scan(self.root())
        return self._scan

    def rescan(self) -> dict:
        self._scan = discover.scan(self.root())
        return self._scan


S = WebUIState()
app = FastAPI(title="aimbot-webui", docs_url=None, redoc_url=None)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"],
                   allow_headers=["*"])


@app.middleware("http")
async def _auth_mw(request, call_next):
    path = request.url.path
    if path == "/" or path.startswith("/static") or path == "/favicon.ico":
        return await call_next(request)
    token = request.headers.get("x-webui-token") or request.query_params.get("token")
    if token != S.cfg.get("token"):
        return PlainTextResponse("unauthorized", status_code=401)
    return await call_next(request)


# ---------- 基础 ----------

@app.get("/")
def index():
    return FileResponse(STATIC_DIR / "index.html")


app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


@app.get("/api/auth/check")
def api_auth_check():
    return {"ok": True, "root": S.cfg["deploy_root"]}


@app.get("/api/state")
def api_state():
    inst = S.inst.snapshot()
    return {
        "config": {"deploy_root": S.cfg["deploy_root"], "bind": S.cfg["bind"],
                   "port": S.cfg["port"], "hot_port": S.cfg["hot_port"],
                   "token": S.cfg["token"], "is_root": _is_root()},
        "scan": S.scan(),
        "instance": inst,
        "logs": S.inst.log_since(0),
        "tasks": S.op.summaries(),
        "history": S.inst.history,
        "telemetry": telemetry.snapshot(),
        "now": time.time(),
    }


class ConfigIn(BaseModel):
    deploy_root: Optional[str] = None
    bind: Optional[str] = None
    port: Optional[int] = None


@app.post("/api/config")
def api_config(inp: ConfigIn):
    kw = {}
    if inp.deploy_root is not None:
        root = Path(inp.deploy_root.strip()).expanduser()
        status, err = discover.root_status(root)
        if status != "ok":
            raise HTTPException(400, err)
        kw["deploy_root"] = str(root)
    if inp.bind is not None:
        if inp.bind not in ("0.0.0.0", "127.0.0.1"):
            raise HTTPException(400, "bind 仅支持 0.0.0.0 (局域网) 或 127.0.0.1 (仅本机)")
        kw["bind"] = inp.bind
    if inp.port is not None:
        if not (1 <= inp.port <= 65535):
            raise HTTPException(400, "端口无效 (1-65535)")
        kw["port"] = inp.port
    if not kw:
        raise HTTPException(400, "没有要修改的字段")
    S.cfg = config.update(**kw)
    S.rescan()
    return {"ok": True, "needs_restart": ("bind" in kw or "port" in kw),
            "config": {"deploy_root": S.cfg["deploy_root"], "bind": S.cfg["bind"],
                       "port": S.cfg["port"], "token": S.cfg["token"]}}


@app.post("/api/scan")
def api_scan():
    return S.rescan()


@app.post("/api/token/reset")
def api_token_reset():
    S.cfg = config.update(token=secrets.token_urlsafe(16))
    return {"ok": True, "token": S.cfg["token"]}


# ---------- 游戏 profile ----------

def _find_profile(stem: str):
    for p in S.scan()["profiles"]:
        if p["file"] == stem:
            return p
    return None


@app.get("/api/profiles/{stem}")
def api_profile(stem: str):
    p = _find_profile(stem)
    if p is None:
        raise HTTPException(404, "未知的游戏 profile: %s (试一次重新扫描)" % stem)
    return p


class ProfileIn(BaseModel):
    display_name: Optional[str] = None
    params: Optional[dict] = None


def _wire_value(pk: str, v):
    return str(v) if pk == "aim_key" else proc.fmt_num(v)


@app.put("/api/profiles/{stem}")
def api_profile_put(stem: str, inp: ProfileIn):
    p = _find_profile(stem)
    if p is None:
        raise HTTPException(404, "未知的游戏 profile: %s" % stem)
    saved_doc = discover.load_profile(stem) or {}
    old_params = saved_doc.get("params") or {}
    new_params = discover.validate_params(inp.params if inp.params is not None
                                          else old_params, S.root())
    disp = (inp.display_name if inp.display_name is not None
            else saved_doc.get("display_name") or stem).strip() or stem
    discover.save_profile(stem, {"display_name": disp, "params": new_params,
                                 "updated_at": time.time()})
    # 热参数: 本次保存中变化的热项 → 直接下发运行中实例 (即时生效, 不重启)
    applied, reason = {}, None
    inst = S.inst.snapshot()
    if inst["state"] == "running":
        if inst["adopted"]:
            reason = "认领实例 (非本 WebUI 启动): 不盲发热参; 按【启动】以新设置接管"
        elif not inst["hot_capable"]:
            reason = "运行中的二进制不支持热参数通道 (旧版固件, 重编译后启动即可)"
        elif inst["profile"] == stem:
            for pk, wk in discover.HOT_WIRE_KEYS.items():
                if pk in (inp.params or {}) and not discover._same(
                        old_params.get(pk), new_params.get(pk)):
                    applied[wk] = _wire_value(pk, new_params[pk])
            if applied:
                try:
                    proc.send_hot(inst["hot_port"] or S.cfg["hot_port"], applied)
                except OSError as e:
                    reason = "热参发送失败: %s (实例可能刚好退出)" % e
        else:
            reason = "运行中的是其它 profile, 本 profile 的改动将在下次【启动】生效"
    S.rescan()
    return {"ok": True, "hot_applied": applied, "hot_reason": reason}


class CopyIn(BaseModel):
    display_name: Optional[str] = None
    file_name: Optional[str] = None


@app.post("/api/profiles/{stem}/copy")
def api_profile_copy(stem: str, inp: CopyIn):
    src_prof = _find_profile(stem)
    if src_prof is None:
        raise HTTPException(404, "未知的游戏 profile: %s" % stem)
    src = S.root() / "scripts" / "game" / (stem + ".sh")
    fname = (inp.file_name or (stem + "_copy")).strip()
    if fname.endswith(".sh"):
        fname = fname[:-3]
    if not discover.SCRIPT_STEM_RE.match(fname):
        raise HTTPException(400, "文件名只能以字母/数字开头, 含字母数字._-")
    dst = S.root() / "scripts" / "game" / (fname + ".sh")
    if dst.exists():
        raise HTTPException(400, "目标脚本已存在: %s.sh" % fname)
    # 完整复制脚本 (不改一字节) + 新 profile JSON (显示名按弹窗输入, 参数随源)
    dst.write_bytes(src.read_bytes())
    disp = (inp.display_name or (src_prof["display_name"] + " 副本")).strip()
    params = src_prof["saved"] if src_prof["has_saved"] else src_prof["script_params"]
    discover.save_profile(fname, {"display_name": disp,
                                  "params": discover.validate_params(params, S.root()),
                                  "updated_at": time.time()})
    S.rescan()
    return {"ok": True, "file": fname, "display_name": disp}


# ---------- 实例 ----------

@app.get("/api/instance/cmd")
def api_instance_cmd(profile: str):
    p = _find_profile(profile)
    if p is None:
        raise HTTPException(404, "未知的游戏 profile: %s" % profile)
    params = p["saved"] if p["has_saved"] and p["saved"] else p["script_params"]
    argv = proc.build_argv(S.root(), params, p["calib"],
                           S.root() / "scripts" / "game" / (profile + ".sh"))
    return {"cmd": proc.cmd_string(argv)}


@app.post("/api/instance/start")
def api_instance_start(body: dict):
    stem = (body or {}).get("profile")
    p = _find_profile(stem)
    if p is None:
        raise HTTPException(404, "未知的游戏 profile: %s" % stem)
    params = p["saved"] if p["has_saved"] and p["saved"] else p["script_params"]
    ok, err = S.inst.start(S.root(), p["file"], p["display_name"], params, p["calib"],
                           S.root() / "scripts" / "game" / (stem + ".sh"))
    if not ok:
        raise HTTPException(409, err)
    return {"ok": True}


@app.post("/api/instance/stop")
def api_instance_stop():
    ok, err = S.inst.stop(S.root())
    if not ok:
        raise HTTPException(409, err)
    return {"ok": True}


@app.get("/api/logs/current")
def api_logs_current():
    return PlainTextResponse(
        S.inst.log_all(),
        media_type="text/plain; charset=utf-8",
        headers={"Content-Disposition": 'attachment; filename="aimbot-webui.log"'})


# ---------- 运维任务 ----------

@app.post("/api/tasks")
def api_task_start(body: dict):
    kind = (body or {}).get("kind")
    if kind not in ("convert", "compile"):
        raise HTTPException(400, "kind 须为 convert 或 compile")
    script = S.root() / "scripts" / (kind + ".sh")
    block = None
    if kind == "compile" and S.inst.snapshot()["state"] in proc.RUNNING_STATES:
        block = ("实例正在运行: 运行中的 bin/aimbot 无法被覆盖 (text file busy)。"
                 "先【停止】实例再编译。")
    tid, err = S.op.start(kind, script, block)
    if tid is None:
        raise HTTPException(409, err)
    return {"ok": True, "id": tid}


@app.get("/api/tasks")
def api_tasks():
    return S.op.summaries()


@app.get("/api/tasks/{tid}/log")
def api_task_log(tid: str, since: int = 0):
    lines = S.op.log_since(tid, since)
    return {"lines": lines, "seq": lines[-1][0] if lines else since}


# ---------- WebSocket ----------

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket, token: str = Query(""), since: int = 0):
    if token != S.cfg.get("token"):
        await ws.close(code=4401)
        return
    await ws.accept()
    inst_seq = since
    task_seqs = {}
    last_tel = 0.0
    try:
        while True:
            await asyncio.sleep(0.4)
            inst = S.inst.snapshot()
            lines = S.inst.log_since(inst_seq)
            if lines:
                inst_seq = lines[-1][0]
            msg = {"type": "tick", "instance": inst, "tasks": S.op.summaries()}
            if lines:
                msg["log"] = lines
            now = time.time()
            if now - last_tel >= 2.0:
                last_tel = now
                msg["telemetry"] = telemetry.snapshot()
            tlog = []
            for t in msg["tasks"]:
                prev = task_seqs.get(t["id"], 0)
                new = S.op.log_since(t["id"], prev)
                if new:
                    task_seqs[t["id"]] = new[-1][0]
                    tlog.append({"id": t["id"], "lines": new})
            if tlog:
                msg["task_log"] = tlog
            await ws.send_json(msg)
    except WebSocketDisconnect:
        pass
    except Exception:
        try:
            await ws.close()
        except Exception:
            pass


@app.on_event("startup")
def _startup():
    # 孤儿认领: WebUI 自身 (重) 启动时, 已存活的 aimbot 必须可见, 不允许显示"已停止"
    try:
        S.inst.adopt(S.root(), S.cfg["hot_port"])
    except Exception:
        pass
    telemetry.start_tegrastats()
