#! /bin/bash
# ==============================================================================
#  编译脚本 — 在 Jetson 上执行, 产物输出到 <根目录>/bin/
#  路径相对脚本自身解析, 与部署位置无关。
# ==============================================================================
set -e
ROOT="$(cd "$(dirname "$(realpath "$0")")/.." && pwd)"
SRC="$ROOT/src"
BIN="$ROOT/bin"
mkdir -p "$BIN"

NVCC=/usr/local/cuda/bin/nvcc
NVCC_FLAGS="-O3 -DNDEBUG -std=c++17 --use_fast_math"
INCLUDES="-I/usr/include/opencv4 -I/usr/local/cuda/include"
LIBS="-L/usr/local/cuda/lib64 -L/usr/lib/aarch64-linux-gnu"
OCV="-lopencv_core -lopencv_videoio -lopencv_highgui -lopencv_imgproc -lopencv_video"
TRT="-lnvinfer -lnvinfer_plugin -lcudart -Xcompiler -pthread"

# 主程序: ffpi 控制律 + 可选训练数据采集 (截图写盘需要 imgcodecs)
$NVCC "$SRC/aimbot.cu" $NVCC_FLAGS $INCLUDES $LIBS \
    $OCV -lopencv_imgcodecs $TRT -o "$BIN/aimbot"

# 备选控制律: 纯自瞄, 无采集
$NVCC "$SRC/aimbot_ballistic.cu" $NVCC_FLAGS $INCLUDES $LIBS \
    $OCV $TRT -o "$BIN/aimbot_ballistic"

$NVCC "$SRC/aimbot_sliding.cu" $NVCC_FLAGS $INCLUDES $LIBS \
    $OCV $TRT -o "$BIN/aimbot_sliding"

echo "✅ 编译完成 → $BIN"
