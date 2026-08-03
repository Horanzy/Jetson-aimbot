#!/bin/bash
# ==============================================================================
#  战地启动脚本 — AI 视觉自瞄 (ffpi 控制律)
#  控制律带宽由标定延迟 L 自动导出, 免手调。路径相对脚本自身解析, 与部署位置无关。
#  标定: 游戏内对准有细节的静止背景, 双侧键长按 5 秒 (画正方形→点头/摇头),
#        成功后自动回写下方 S_EST / L_EST。
# ==============================================================================

# ---- 路径 ----
ROOT="$(cd "$(dirname "$(realpath "$0")")/../.." && pwd)"
APP="$ROOT/bin/aimbot"
MODEL_PATH="$ROOT/engine/best23.engine"

# ---- 自瞄参数 ----
CLASS_ID=0          # 目标类别 (依模型标签: 0=头 1=身)
CONF_THRESH=0.5     # 置信度阈值
Y_OFFSET=65         # 瞄准高度 (0=脚 50=中心 65=胸颈 100=头)
CAM_DEV="Asus"      # 采集卡: Hagibis / Asus (或 /dev/videoN)
CAM_FPS=120         # 采集帧率: 120 / 60
MAX_SPEED=1500      # 速度上限 px/s
AIM_KEY=both        # 触发键: fire / ads / both
PREVIEW="n"         # 预览窗口: y / n

# ---- 训练数据采集 (开关) ----
CAPTURE="n"                 # y=开启截图采集  n=纯自瞄
OUT_DIR="$ROOT/dataset"     # 输出目录 (自动建 fire/ det/ auto/ 子目录)
FIRE_MS=800                 # 开火截图间隔 ms
AUTO_S=10                   # 定时截图间隔 s (随机 0.5x~1.5x)
COOLDOWN_MS=800             # 检测/定时截图冷却 ms (开火不受限)
JPEG_Q=95                   # JPEG 质量

# ---- 标定数据 [标定成功后自动回写, 请勿手改] ----
S_EST=0.315
L_EST=68.0

# ==============================================================================
sudo jetson_clocks
sudo bash "$ROOT/scripts/setup_mouse.sh"

SCRIPT_PATH="$(realpath "$0")"

CAPTURE_ARGS=""
if [ "$CAPTURE" = "y" ]; then
  CAPTURE_ARGS="-o $OUT_DIR -F $FIRE_MS -A $AUTO_S -C $COOLDOWN_MS -q $JPEG_Q"
fi

sudo "$APP" \
  -m "$MODEL_PATH" \
  -c "$CLASS_ID" \
  -t "$CONF_THRESH" \
  -y "$Y_OFFSET" \
  -d "$CAM_DEV" \
  -f "$CAM_FPS" \
  -x "$MAX_SPEED" \
  -s "$S_EST" \
  -l "$L_EST" \
  -S "$SCRIPT_PATH" \
  -k "$AIM_KEY" \
  -v "$PREVIEW" \
  $CAPTURE_ARGS
