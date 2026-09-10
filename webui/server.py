"""WebUI 服务入口。

用法 (Jetson, 推荐 systemd): 见 webui/deploy/install.sh
手动:  cd webui && sudo python3 server.py

启动后按配置 bind:port 监听 (默认 0.0.0.0:80, root 直接绑定),
token 打印在横幅里, 浏览器打开 http://<jetson-ip>/?token=... 即可。
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from app import config                    # noqa: E402
from app.api import app                   # noqa: E402


def main() -> None:
    cfg = config.load()
    print("════════════════════════════════════════════")
    print("  aimbot WebUI")
    print("════════════════════════════════════════════")
    print("  地址:   http://%s:%d  (绑定 %s)" % (_lan_hint(), cfg["port"], cfg["bind"]))
    print("  token:  %s" % cfg["token"])
    print("  打开:   http://<jetson-ip>/?token=%s" % cfg["token"])
    print("  部署根: %s" % cfg["deploy_root"])
    print("  (设置页可改根目录/端口/绑定, 可重置 token)")
    print("════════════════════════════════════════════")
    if cfg["bind"] == "0.0.0.0":
        print("  ⚠ 服务对整个局域网开放, 靠 token 门槛; 不需要时把 bind 改为 127.0.0.1")
    import uvicorn
    uvicorn.run(app, host=cfg["bind"], port=int(cfg["port"]), log_level="warning")


def _lan_hint() -> str:
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "127.0.0.1"


if __name__ == "__main__":
    main()
