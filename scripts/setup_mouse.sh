#!/bin/bash

# setup_mouse_ultimate.sh
# 微软IE3.0复刻版模拟器 - 彻底摆脱第三方驱动拦截

modprobe libcomposite
modprobe usb_f_hid

CONFIGFS="/sys/kernel/config/usb_gadget"
GADGET="$CONFIGFS/g_mouse"

# 深度清理
if [ -d "$GADGET" ]; then
    echo "" > "$GADGET/UDC" 2>/dev/null || true
    rm -f $GADGET/configs/c.1/hid.usb* 2>/dev/null
    rmdir $GADGET/configs/c.1/strings/0x409 2>/dev/null
    rmdir $GADGET/configs/c.1 2>/dev/null
    rmdir $GADGET/functions/hid.usb* 2>/dev/null
    rmdir $GADGET/strings/0x409 2>/dev/null
    rmdir $GADGET 2>/dev/null
fi

mkdir -p $GADGET
cd $GADGET || exit

# 微软IE3.0复刻版 - 纯单一接口，无任何额外功能
echo 0x045E > idVendor     # Microsoft VID - 绝对不会被罗技驱动拦截
echo 0x0047 > idProduct    # IntelliMouse Explorer 3.0 PID
echo 0x0300 > bcdDevice
echo 0x0200 > bcdUSB       # 纯USB 2.0，不声明支持SuperSpeed（关键！）

# 标准单一HID设备
echo 0x00 > bDeviceClass
echo 0x00 > bDeviceSubClass
echo 0x00 > bDeviceProtocol

mkdir -p strings/0x409
echo "000000000002" > strings/0x409/serialnumber
echo "Microsoft" > strings/0x409/manufacturer
echo "IntelliMouse Explorer 3.0" > strings/0x409/product

mkdir -p configs/c.1/strings/0x409
echo "HID Mouse" > configs/c.1/strings/0x409/configuration
echo 100 > configs/c.1/MaxPower

# 标准引导鼠标接口
mkdir -p functions/hid.usb0
echo 1 > functions/hid.usb0/subclass  # Boot Interface Subclass
echo 2 > functions/hid.usb0/protocol  # Mouse Protocol
echo 9 > functions/hid.usb0/report_length

# 与你之前完全相同的16位高精度报告描述符
python3 -c "open('functions/hid.usb0/report_desc', 'wb').write(bytes.fromhex('05010902a10185020901a1000509190129101500250175019510810205010930093116008026ff7f75109502810609381581257f750895018106050c0a38021581257f750895018106c0c0'))"

ln -s functions/hid.usb0 configs/c.1/

echo "绑定 UDC..."
UDC_NAME=$(ls /sys/class/udc | head -n 1)
if [ -z "$UDC_NAME" ]; then
    echo "❌ 找不到 UDC 控制器"
    exit 1
fi
echo "$UDC_NAME" > UDC

sleep 1
chmod 666 /dev/hidg0 2>/dev/null || true

echo "================================================="
echo "✅ 微软IE3.0虚拟鼠标已就绪: /dev/hidg0"
echo "✅ 预期驱动: mouhid.sys (无任何第三方拦截)"
echo "================================================="