#!/bin/bash

# setup_mouse.sh
# USB Gadget HID 鼠标配置 - 创建 /dev/hidg0

modprobe libcomposite
modprobe usb_f_hid

CONFIGFS="/sys/kernel/config/usb_gadget"
GADGET="$CONFIGFS/g_mouse"

# 清理旧配置
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

# 通用单一 HID 鼠标，无任何额外功能
echo 0x1d6b > idVendor     # Linux Foundation VID
echo 0x0104 > idProduct    # 通用 gadget PID
echo 0x0300 > bcdDevice
echo 0x0200 > bcdUSB       # USB 2.0

# 标准单一HID设备
echo 0x00 > bDeviceClass
echo 0x00 > bDeviceSubClass
echo 0x00 > bDeviceProtocol

mkdir -p strings/0x409
echo "000000000001" > strings/0x409/serialnumber
echo "Generic" > strings/0x409/manufacturer
echo "USB Mouse" > strings/0x409/product

mkdir -p configs/c.1/strings/0x409
echo "HID Mouse" > configs/c.1/strings/0x409/configuration
echo 100 > configs/c.1/MaxPower

# 标准引导鼠标接口
mkdir -p functions/hid.usb0
echo 1 > functions/hid.usb0/subclass  # Boot Interface Subclass
echo 2 > functions/hid.usb0/protocol  # Mouse Protocol
echo 9 > functions/hid.usb0/report_length

# 16 键高精度鼠标报告描述符
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
echo "✅ USB Gadget 虚拟鼠标已就绪: /dev/hidg0"
echo "✅ 标准 HID 引导协议接口"
echo "================================================="
