#! /bin/bash
# ==============================================================================
#  编译脚本 — 在 Jetson 上执行, 产物输出到 <根目录>/bin/
#  路径相对脚本自身解析, 与部署位置无关。
#  模块结构: main (入口) + core/ (控制律/估计器/标定/TRT 辅助/共享状态)
#            + io/ (采集/USB 鼠标/raw_gadget 会话/手柄输入与合并层/热参);
#            逐个编译到 build/ 再链接。
# ==============================================================================
set -e
ROOT="$(cd "$(dirname "$(realpath "$0")")/.." && pwd)"
SRC="$ROOT/src"
BUILD="$ROOT/build"
BIN="$ROOT/bin"
mkdir -p "$BUILD" "$BIN"

NVCC=/usr/local/cuda/bin/nvcc
NVCC_FLAGS="-O3 -DNDEBUG -std=c++17 --use_fast_math"
INCLUDES="-I$SRC -I/usr/include/opencv4 -I/usr/local/cuda/include"
LIBS="-L/usr/local/cuda/lib64 -L/usr/lib/aarch64-linux-gnu"
OCV="-lopencv_core -lopencv_videoio -lopencv_highgui -lopencv_imgproc -lopencv_video"
TRT="-lnvinfer -lnvinfer_plugin -lcudart -Xcompiler -pthread"

MODULES="main \
         core/control core/estimator core/calib core/trt core/state \
         io/capture io/hid_mouse io/usbraw io/pad_input io/pad_output io/pad_xinput io/hotctl"

OBJS=""
for m in $MODULES; do
    obj="$BUILD/$(echo "$m" | tr '/' '_').o"
    # shellcheck disable=SC2086
    $NVCC -c "$SRC/$m.cu" $NVCC_FLAGS $INCLUDES -o "$obj"
    OBJS="$OBJS $obj"
done

# 链接: 控制律 ff_pi_acc + 可选训练数据采集 (截图写盘需要 imgcodecs)
# shellcheck disable=SC2086
$NVCC $OBJS $LIBS $OCV -lopencv_imgcodecs $TRT -o "$BIN/aimbot"

# 标定墙钟单测: 仅 calib.h 头常数断言, 无需链接模块对象
# shellcheck disable=SC2086
$NVCC -c "$SRC/core/calib_test.cu" $NVCC_FLAGS $INCLUDES -o "$BUILD/calib_test.o"
$NVCC "$BUILD/calib_test.o" $LIBS -o "$BUILD/calib_test"
"$BUILD/calib_test"

# pad 输入/合并层/XInput 输出后端单测 (注入换算/合并钳制/账本/触发门控/账本路由/
#   未在位不阻塞/发布点/XInput 线格式与设备字节) — 随编译构建并执行一次, 不参与
#   aimbot 链接; 链接除 main 外的全部模块对象。断言失败 (退出码非 0) 时 set -e
#   终止整个编译。
# shellcheck disable=SC2086
$NVCC -c "$SRC/io/pad_test.cu" $NVCC_FLAGS $INCLUDES -o "$BUILD/pad_test.o"
TEST_OBJS=$(printf '%s\n' $OBJS | grep -v "build/main.o" | tr '\n' ' ')
# shellcheck disable=SC2086
$NVCC "$BUILD/pad_test.o" $TEST_OBJS $LIBS $OCV -lopencv_imgcodecs $TRT -o "$BUILD/pad_test"
"$BUILD/pad_test"

echo "✅ 编译完成 → $BIN"
