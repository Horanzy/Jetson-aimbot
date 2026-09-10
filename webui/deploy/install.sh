#!/bin/bash
# ==============================================================================
#  安装/更新 WebUI 的 systemd 服务 (在 Jetson 上以 root 运行一次)。
#  路径从脚本自身位置解析 — 部署目录改名/移动后重跑一次本脚本即可。
# ==============================================================================
set -e
WEBUI_DIR="$(cd "$(dirname "$(realpath "$0")")/.." && pwd)"
SERVICE=aimbot-webui
PY="${PY:-python3}"

echo "WebUI 目录: $WEBUI_DIR"

# ---- 依赖 (联网时直接装; 离线见 webui/README.md 的 wheels 方式) ----
if ! "$PY" -c "import fastapi, uvicorn, websockets" 2>/dev/null; then
    echo "[依赖] 缺 fastapi/uvicorn/websockets, 尝试 pip 安装…"
    "$PY" -m pip install -r "$WEBUI_DIR/requirements.txt" \
        || { echo "✗ pip 安装失败。若系统提示 externally-managed, 参考 webui/README.md "
             echo "  (离线 wheels / venv) 手动安装后重跑本脚本。"; exit 1; }
fi

# ---- 生成 unit (User=root 是刻意的, 理由见 webui/README.md 权限模型一节) ----
cat > /etc/systemd/system/$SERVICE.service <<EOF
[Unit]
Description=aimbot WebUI (Jetson aimbot management panel)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=root
WorkingDirectory=$WEBUI_DIR
ExecStart=$PY $WEBUI_DIR/server.py
Restart=on-failure
RestartSec=3
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable $SERVICE.service
systemctl restart $SERVICE.service
sleep 1
systemctl --no-pager -l status $SERVICE.service | head -n 10 || true

echo
echo "✅ 安装完成: sudo systemctl {status|restart|stop} $SERVICE"
echo "   日志:   journalctl -u $SERVICE -f   (token 打印在启动横幅里)"
echo "   浏览器: http://<jetson-ip>/?token=<token>"
