#!/bin/bash

# ================= 配置路径 (相对脚本自身解析, 与部署位置无关) =================
ROOT="$(cd "$(dirname "$(realpath "$0")")/.." && pwd)"
ONNX_DIR="$ROOT/onnx"
ENGINE_DIR="$ROOT/engine"
TRTEXEC_PATH="/usr/src/tensorrt/bin/trtexec"
CACHE_FILE="$ROOT/trt_timing_cache.cache"  # 全局时序缓存
# ============================================

# 自动尝试锁定 Jetson 频率
if [ "$EUID" -eq 0 ]; then
    echo "[提示] 检测到 Root 权限，正在自动锁定 Jetson 核心频率以加速编译..."
    jetson_clocks
else
    echo "=========================================================="
    echo "提示: 如果编译太慢，建议使用 sudo 运行此脚本，或者在运行前手动执行:"
    echo "      sudo jetson_clocks"
    echo "=========================================================="
fi

if [ ! -f "$TRTEXEC_PATH" ]; then
    echo "错误: 未找到 trtexec 可执行文件: $TRTEXEC_PATH"
    exit 1
fi

# ===== 关键修复：彻底清除可能来自其他设备的缓存 =====
if [ -f "$CACHE_FILE" ]; then
    echo "[清理] 检测到旧有的时序缓存文件，为避免 '不是本机生成的引擎' 错误，将其删除并重建。"
    rm -f "$CACHE_FILE"
fi
# 提前 touch 一个空文件，确保 trtexec 能正确写入
touch "$CACHE_FILE"

echo "开始执行 TensorRT 10 最佳优化增量转换..."
echo "------------------------------------------------"

# 计数器
skipped=0
converted=0
failed=0

# 遍历所有 .onnx 文件（IFS= 确保带空格的中文路径不出错）
while IFS= read -r onnx_path; do
    relative_path="${onnx_path#$ONNX_DIR/}"
    relative_base="${relative_path%.*}"
    engine_path="$ENGINE_DIR/${relative_base}.engine"
    target_engine_dir=$(dirname "$engine_path")

    if [ -f "$engine_path" ]; then
        echo "[跳过] $relative_path -> 对应的 Engine 已存在."
        skipped=$((skipped + 1))
    else
        echo "[转换] 正在优化转换: $relative_path ..."
        mkdir -p "$target_engine_dir"
        
        # ================= TRT 增强编译命令 =================
        # 核心优化参数说明：
        #   --fp16                 : 启用 FP16 精度
        #   --useCudaGraph         : 启用 CUDA Graph 加速推理
        #   --builderOptimizationLevel=3 : 最高构建优化级别
        #   --timingCacheFile      : 使用本机新缓存，加速后续模型构建
        #   --skipInference        : 跳过性能测试，只生成引擎（想要精度数据可改为 --iterations=N）
        # ========================================================
        "$TRTEXEC_PATH" \
            --onnx="$onnx_path" \
            --saveEngine="$engine_path" \
            --fp16 \
            --useCudaGraph \
            --timingCacheFile="$CACHE_FILE" \
            --memPoolSize=workspace:4096 \
            --builderOptimizationLevel=3 \
            --iterations=100
        
        if [ $? -eq 0 ]; then
            echo "[成功] 已生成: $engine_path"
            converted=$((converted + 1))
        else
            echo "[失败] $relative_path 转换失败！"
            failed=$((failed + 1))
        fi
        echo "------------------------------------------------"
    fi
done < <(find "$ONNX_DIR" -type f -name "*.onnx")

echo "任务执行完毕！"
echo "成功: $converted  跳过: $skipped  失败: $failed"