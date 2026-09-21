#!/usr/bin/env python3
# ==============================================================================
#  pad_key_probe.py — 手柄按键探针: 按一下, 打印内核给的是什么
#
#  用途: 物理手柄上还没映射的键 (典型: GameSir G7 Pro 的"分享/上传"键) 要先知道
#        内核报的是哪个 code, 才能决定映射到哪个逻辑位 (io/pad_input.cu 的
#        pad_key_bit)。本脚本只读不抢: 逐事件打印 EV_KEY 的 code 名与原始值,
#        并标注"固件当前是否映射"(对照 io/pad_input.cu 的表)。
#
#  运行前提: **aimbot 必须已停止** — 固件用 EVIOCGRAB 独占手柄节点, 独占期间其他
#        读取者收不到事件 (拿不到事件不是脚本的问题)。
#      sudo systemctl stop ... / webui 里停止 / sudo pkill -x aimbot
#      sudo python3 scripts/test/pad_key_probe.py            # 自动选手柄节点
#      sudo python3 scripts/test/pad_key_probe.py <eventN|by-id 子串>
#      sudo python3 scripts/test/pad_key_probe.py --raw      # 连 EV_ABS 一起打 (摇杆/扳机)
#  按 Ctrl+C 结束。纯标准库; 无 root 时若节点可读也能跑。
#
#  输出解读: 例如按下分享键得到
#      [EV_KEY] code=0x13c (BTN_TRIGGER_HAPPY1) value=1 down   ← 固件未映射
#  那就是要在 pad_key_bit() 里加一条 `case BTN_TRIGGER_HAPPY1: return PADBTN_XXX;`
#  (以及 PadBtn 与 P5G 报告的对应位), 并补单测。
# ==============================================================================
import ctypes
import glob
import os
import re
import struct
import sys

EV_KEY, EV_ABS = 0x01, 0x03
INPUT_EVENT = struct.Struct("llHHi")          # time, time, type, code, value (64-bit)

# 内核 EV_KEY 名表 (只列手柄会用到的; 由 linux/input-event-codes.h 逐条抄录)
KEY_NAMES = {
    0x120: "BTN_JOYSTICK", 0x121: "BTN_TRIGGER", 0x122: "BTN_THUMB",
    0x123: "BTN_THUMB2", 0x124: "BTN_TOP", 0x125: "BTN_TOP2", 0x126: "BTN_PINKIE",
    0x127: "BTN_BASE", 0x128: "BTN_BASE2", 0x129: "BTN_BASE3", 0x12a: "BTN_BASE4",
    0x12b: "BTN_BASE5", 0x12c: "BTN_BASE6", 0x12d: "BTN_DEAD",
    0x130: "BTN_SOUTH/A", 0x131: "BTN_EAST/B", 0x132: "BTN_C", 0x133: "BTN_NORTH/X",
    0x134: "BTN_WEST/Y", 0x135: "BTN_Z", 0x136: "BTN_TL/L1", 0x137: "BTN_TR/R1",
    0x138: "BTN_TL2/L2", 0x139: "BTN_TR2/R2", 0x13a: "BTN_SELECT/SHARE",
    0x13b: "BTN_START/OPTIONS", 0x13c: "BTN_MODE/PS", 0x13d: "BTN_THUMBL/L3",
    0x13e: "BTN_THUMBR/R3", 0x13f: "BTN_DIGI",
    0x140: "BTN_TOOL_PEN", 0x141: "BTN_TOOL_RUBBER", 0x142: "BTN_TOOL_BRUSH",
    0x143: "BTN_TOOL_PENCIL", 0x144: "BTN_TOOL_AIRBRUSH", 0x145: "BTN_TOOL_FINGER",
    0x14a: "BTN_TOOL_MOUSE", 0x14c: "BTN_TOOL_DOUBLETAP", 0x14d: "BTN_TOOL_TRIPLETAP",
    0x145+8: "BTN_TOOL_QUADTAP",
    0x220: "BTN_DPAD_UP", 0x221: "BTN_DPAD_DOWN", 0x222: "BTN_DPAD_LEFT",
    0x223: "BTN_DPAD_RIGHT",
    0x2c0: "BTN_TRIGGER_HAPPY1", 0x2c1: "BTN_TRIGGER_HAPPY2",
    0x2c2: "BTN_TRIGGER_HAPPY3", 0x2c3: "BTN_TRIGGER_HAPPY4",
    0x2c4: "BTN_TRIGGER_HAPPY5", 0x2c5: "BTN_TRIGGER_HAPPY6",
    0x2c6: "BTN_TRIGGER_HAPPY7", 0x2c7: "BTN_TRIGGER_HAPPY8",
    0x2c8: "BTN_TRIGGER_HAPPY9", 0x2c9: "BTN_TRIGGER_HAPPY10",
    0x2ca: "BTN_TRIGGER_HAPPY11", 0x2cb: "BTN_TRIGGER_HAPPY12",
    0x2cc: "BTN_TRIGGER_HAPPY13", 0x2cd: "BTN_TRIGGER_HAPPY14",
    0x2ce: "BTN_TRIGGER_HAPPY15", 0x2cf: "BTN_TRIGGER_HAPPY16",
    0x2d0: "BTN_TRIGGER_HAPPY17", 0x2d1: "BTN_TRIGGER_HAPPY18",
    0x2d2: "BTN_TRIGGER_HAPPY19", 0x2d3: "BTN_TRIGGER_HAPPY20",
}
ABS_NAMES = {0: "ABS_X", 1: "ABS_Y", 2: "ABS_Z", 3: "ABS_RX", 4: "ABS_RY",
             5: "ABS_RZ", 6: "ABS_THROTTLE", 7: "ABS_RUDDER", 8: "ABS_WHEEL",
             9: "ABS_GAS", 10: "ABS_BRAKE", 16: "ABS_HAT0X", 17: "ABS_HAT0Y"}

# io/pad_input.cu 的键位表 — 探针据此标注"已映射/未映射"; 0x63 走的是**兄弟节点**
#   表 (pad_extra_key_bit: 手柄的分享键落在键盘节点上, 由同一读取线程从兄弟节点收)
MAPPED = {0x130: "PADBTN_A", 0x131: "PADBTN_B", 0x133: "PADBTN_X",
          0x134: "PADBTN_Y", 0x136: "PADBTN_LB", 0x137: "PADBTN_RB",
          0x13a: "PADBTN_BACK", 0x13b: "PADBTN_START", 0x13c: "PADBTN_GUIDE",
          0x13d: "PADBTN_L3", 0x13e: "PADBTN_R3", 0x220: "PADBTN_DPAD_UP",
          0x221: "PADBTN_DPAD_DOWN", 0x222: "PADBTN_DPAD_LEFT",
          0x223: "PADBTN_DPAD_RIGHT",
          0x063: "PADBTN_TOUCH (兄弟节点 = 键盘节点上的分享键)"}


def name_of(code, table, fallback):
    return table.get(code, "%s(%d)" % (fallback, code))


def evdev_name(path):
    """节点名 (EVIOCGNAME), 用于人读; 失败返回空串"""
    try:
        with open(path, "rb") as f:
            buf = ctypes.create_string_buffer(256)
            import fcntl
            fcntl.ioctl(f.fileno(), 0x81014506, buf)      # EVIOCGNAME(256)
            return buf.value.decode("utf-8", "replace")
    except Exception:
        return ""


def main(argv):
    args = [a for a in argv[1:] if not a.startswith("--")]
    show_abs = "--raw" in argv
    spec = args[0] if args else ""

    def open_candidates():
        out = []
        if spec.startswith("/dev/"):
            return [spec]
        if spec:
            out += sorted(glob.glob("/dev/input/by-id/*%s*event-joystick" % spec))
            out += sorted(glob.glob("/dev/input/by-id/*%s*event*" % spec))
        out += sorted(glob.glob("/dev/input/by-id/*-event-joystick"))
        if not out:
            out = ["/dev/input/event%d" % i for i in range(32)
                   if os.path.exists("/dev/input/event%d" % i)]
        return out

    dev = None
    for c in open_candidates():
        if os.path.exists(c) and os.access(c, os.R_OK):
            dev = c
            break
    if dev is None:
        print("❌ 没有可读的手柄节点 — 用 sudo 运行, 或确认手柄已插。")
        print("   (aimbot 正在运行时会独占节点: 先停止它)")
        return 1

    nm = evdev_name(dev)
    print("监听 %s  %s" % (dev, ("(%s)" % nm) if nm else ""))
    print("按手柄上的键 (分享/上传键等); Ctrl+C 结束。%s"
          % ("EV_ABS 一并打印。" if show_abs else "--raw 可连摇杆/扳机一起打。"))
    try:
        fd = open(dev, "rb", buffering=0)
    except PermissionError:
        print("❌ %s 无读权限 — 请 sudo 运行" % dev)
        return 1
    seen = set()
    try:
        while True:
            raw = fd.read(INPUT_EVENT.size)
            if len(raw) < INPUT_EVENT.size:
                continue
            _s, _us, etype, code, value = INPUT_EVENT.unpack(raw)
            if etype == EV_KEY:
                if value == 1:
                    seen.add(code)
                kind = {0: "up  ", 1: "down", 2: "auto"}.get(value, str(value))
                mark = ("→ 固件已映射 %s" % MAPPED[code]) if code in MAPPED \
                       else "→ **固件未映射** (按 0 丢掉)"
                print("[EV_KEY] code=0x%03x (%s) value=%d %s  %s"
                      % (code, name_of(code, KEY_NAMES, "KEY"), value, kind, mark))
            elif etype == EV_ABS and show_abs:
                print("[EV_ABS] code=0x%02x (%s) value=%d"
                      % (code, name_of(code, ABS_NAMES, "ABS"), value))
    except KeyboardInterrupt:
        print("\n按键小结 (本次按到过, 未在固件映射表里的单列):")
        for c in sorted(seen):
            print("  0x%03x  %-24s %s" % (c, name_of(c, KEY_NAMES, "KEY"),
                  MAPPED.get(c, "**未映射**")))
    finally:
        fd.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
