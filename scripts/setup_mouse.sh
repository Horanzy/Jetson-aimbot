#!/bin/bash
# ==============================================================================
#  setup_mouse.sh — raw_gadget 鼠标通道就绪 (Jetson, sudo):
#    ① 载入 raw_gadget 模块 (内核须已具备该模块 — 发行版包或按内核文档
#      Documentation/usb/raw_gadget.rst 出树编译, 各机自行部署)
#    ② UDC 独占腾空: 解绑 configfs 遗留 gadget; raw_gadget 会话占用须先停其进程
#    ③ /dev/raw-gadget 权限 (666, 非 root 直跑 aimbot 也可打开)
#
#  UDC 两级名字由固件从 sysfs 发现 (v6.8.12 raw_gadget 无 "空名 = 任意 UDC"
#  通配); 会话 open 即绑定 UDC, close 即解绑 — 本脚本只保证 UDC 空闲、节点可用。
#  路径相对脚本自身解析 (realpath 向上找 ROOT), 与部署位置无关。
# ==============================================================================

# ① raw_gadget 模块
if ! modprobe raw_gadget 2>/dev/null; then
    echo "❌ raw_gadget 模块不可用 — 本机内核须先具备该模块 (modprobe raw_gadget;"
    echo "   缺失时按内核文档 Documentation/usb/raw_gadget.rst 自行构建安装)"
    exit 1
fi
if [ ! -e /dev/raw-gadget ]; then
    echo "❌ /dev/raw-gadget 未出现 (模块已载入但设备节点缺失)"
    exit 1
fi

# ② UDC 独占腾空 — configfs 遗留 gadget 写空 UDC 文件即解绑 (无 gadget 则整段跳过)
modprobe libcomposite 2>/dev/null || true     # configfs gadget 目录可见性
for g in /sys/kernel/config/usb_gadget/*; do
    [ -e "$g/UDC" ] || continue
    if echo "" > "$g/UDC" 2>/dev/null; then
        echo "✅ 已解绑遗留 gadget: $(basename "$g")"
    else
        echo "⚠ 解绑 $g 失败 (手动: echo \"\" | sudo tee $g/UDC)"
    fi
done
echo "ℹ 若 UDC 仍被占 (aimbot 报 EBUSY): 先停占用 /dev/raw-gadget 的进程 (如另一 aimbot 实例)"

# ③ 节点权限
chmod 666 /dev/raw-gadget 2>/dev/null || echo "⚠ chmod 666 /dev/raw-gadget 失败 (非 root?)"

UDC_NAME=$(ls /sys/class/udc 2>/dev/null | head -n 1)
if [ -z "$UDC_NAME" ]; then
    echo "❌ 找不到 UDC 控制器 (/sys/class/udc 为空)"
    exit 1
fi

echo "================================================="
echo "✅ raw_gadget 鼠标通道就绪: /dev/raw-gadget (UDC: $UDC_NAME)"
echo "✅ UDC 已腾空, 会话由固件自行绑定/解绑"
echo "================================================="
