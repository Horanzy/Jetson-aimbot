"""webui 配置与状态路径。

遵循库约定: 所有路径从本文件自身位置解析, 与调用 cwd 无关。
data/ 是 WebUI 私有状态 (config / profile 参数 / 历史), 不入 git。
"""
import json
import secrets
import threading
from pathlib import Path

WEBUI_DIR = Path(__file__).resolve().parent.parent          # webui/
DATA_DIR = WEBUI_DIR / "data"
PROFILE_DIR = DATA_DIR / "profiles"
CONFIG_PATH = DATA_DIR / "config.json"
HISTORY_PATH = DATA_DIR / "history.json"

DEFAULTS = {
    "deploy_root": str(WEBUI_DIR.parent),   # 默认 = webui/ 上一层 (含 scripts/ 的那层)
    "token": "",
    "bind": "0.0.0.0",                      # 局域网管理; 收紧可改 127.0.0.1
    "port": 80,                             # root 服务直接绑 80, 免端口访问
    "hot_port": 47700,                      # 固件热参通道端口 (头部常量 HOT_CTL_PORT)
}

_lock = threading.Lock()
_cache = None


def _save(cfg: dict) -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    tmp = CONFIG_PATH.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(cfg, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    tmp.replace(CONFIG_PATH)                # 原子替换, 与固件标定回写同一口径


def load() -> dict:
    """读配置 (首次调用时播种并生成 token); 进程内单例缓存。"""
    global _cache
    with _lock:
        if _cache is None:
            cfg = dict(DEFAULTS)
            if CONFIG_PATH.exists():
                try:
                    cfg.update(json.loads(CONFIG_PATH.read_text(encoding="utf-8")))
                except (OSError, ValueError):
                    pass                    # 配置损坏 → 回退默认, token 将重新生成
            if not cfg.get("token"):
                cfg["token"] = secrets.token_urlsafe(16)
                _save(cfg)
            PROFILE_DIR.mkdir(parents=True, exist_ok=True)
            _cache = cfg
        return _cache


def update(**kv) -> dict:
    """持久化更新若干字段 (None 的字段跳过), 返回更新后的配置。"""
    cfg = load()
    with _lock:
        cfg.update({k: v for k, v in kv.items() if v is not None})
        _save(cfg)
        return dict(cfg)
