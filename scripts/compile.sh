#! /bin/bash
# ==============================================================================
#  编译脚本 — 在 Jetson 上执行, 产物输出到 <根目录>/bin/
#  路径相对脚本自身解析, 与部署位置无关。
#  模块结构: main (入口) + core/ (共享状态/控制律/估计器/标定/TRT 辅助)
#            + io/ (采集/鼠标输入与 USB 输出/热参); 逐编译单元编译到 build/ 再链接。
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

# 模块清单 = src/ 下的全部编译单元 (main.cu 与 core/io 各 .cu 逐一对应)
MODULES="main \
         core/control core/estimator core/calib core/trt core/state \
         io/capture io/hid_mouse io/usbraw io/hotctl io/pad_input io/pad_output io/pad_xinput"

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

# 控制拍单测 (拉枪速度倍率 spd 的落点): spd=100 即基线 (1 count = 1 px), 逐轴独立,
#   ADS 键按住那一拍整套切换 + g_ads_down 导出, 注入换算/在飞补偿/估计器自身运动
#   补偿共用同一份逐轴有效灵敏度 (含随 spd 成比例变化), spd_clamp 的夹取带,
#   热参路径 (spdx 下一拍生效 / 非法值被拒绝) — 链接除 main.o 外的模块对象并执行,
#   断言失败 (退出码非 0) 时 set -e 终止整个编译。
# shellcheck disable=SC2086
$NVCC -c "$SRC/core/control_test.cu" $NVCC_FLAGS $INCLUDES -o "$BUILD/control_test.o"
CONTROL_TEST_OBJS=$(printf '%s\n' $OBJS | grep -v "build/main.o" | tr '\n' ' ')
# shellcheck disable=SC2086
$NVCC "$BUILD/control_test.o" $CONTROL_TEST_OBJS $LIBS $OCV -lopencv_imgcodecs $TRT \
    -o "$BUILD/control_test"
"$BUILD/control_test"

# 手柄模式单测 (输入映射 8→16 位 / 注入合并几何 / 账本与发布点契约 / XInput 线格式
#   与设备字节): 与控制拍单测同一链接方式 (除 main.o 外的模块对象), 断言失败即
#   set -e 终止整个编译。
# shellcheck disable=SC2086
$NVCC -c "$SRC/io/pad_test.cu" $NVCC_FLAGS $INCLUDES -o "$BUILD/pad_test.o"
# shellcheck disable=SC2086
$NVCC "$BUILD/pad_test.o" $CONTROL_TEST_OBJS $LIBS $OCV -lopencv_imgcodecs $TRT \
    -o "$BUILD/pad_test"
"$BUILD/pad_test"

echo "✅ 编译完成 → $BIN"
