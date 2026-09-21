#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================================
#  uinput_pad_test.py — 手柄模式输入/合并层的端到端验证 (Jetson 本机)。
#
#  用 /dev/uinput 合成一只虚拟 Xbox 布局手柄 (xpad 风格: RX/RY=右摇杆 ±32767,
#  Z/RZ=扳机 0..1023 — 刻意与实体 GameSir-G7 Pro 的 HID 风格布局不同, 以覆盖
#  轴分配的两族分支), 驱动 aimbot 的 -M <模式> --pad-dump 干跑, 解析 stdout 的
#  [PAD] 行断言: 按键/摇杆/扳机透传、fire 门控、dpad、拔出清键与重连恢复。
#  纯 python3 标准库 (fcntl/struct/os)。零注入时右摇杆 = 人类通道 (合成数学的
#  v=0 特例) 在用例 2 断言; 非零注入的合并/钳制数学与发布点契约由 build/pad_test
#  覆盖 (律指令在无目标的干跑里恒 0, e2e 无法确定性驱动)。
#
#  --mode: pad (XInput 后端) 与 p5g (P5 General 后端) 共用整条输入/合并/发布链,
#  而 --pad-dump 读的是发布点、与后端无关, 故同一套断言对两者都成立 ——
#  --mode p5g 即验证 P5G 后端下的同一条链路 (P5G 的签名流水线需真加密狗才上
#  PS5 线, 但本测试断言的是合并层输出, 加密狗在场与否都不改断言面)。
#
#  跑法 (仓库根, 手柄模式需要 evdev/uinput 权限, sudo 运行):
#    sudo python3 scripts/test/uinput_pad_test.py [--mode pad|p5g] \
#        [--aimbot bin/aimbot] [--model engine/apex.engine] [--cam /dev/video0]
#  全过输出 ALL PASS 返回 0。
# ============================================================================
import argparse
import fcntl
import os
import re
import signal
import struct
import subprocess
import sys
import time

# ---- linux/uinput.h 与 linux/input-event-codes.h 的内核 ABI 常量 ----
UI_SET_EVBIT   = 0x40045564          # _IOW('U', 100, int)
UI_SET_KEYBIT  = 0x40045565          # _IOW('U', 101, int)
UI_SET_ABSBIT  = 0x40045567          # _IOW('U', 103, int)
UI_ABS_SETUP   = 0x401C5504          # _IOW('U', 4, struct uinput_abs_setup)
UI_DEV_SETUP   = 0x405C5503          # _IOW('U', 3, struct uinput_setup)
UI_DEV_CREATE  = 0x5501              # _IO('U', 1)
UI_DEV_DESTROY = 0x5502              # _IO('U', 2)
EV_SYN, EV_KEY, EV_ABS = 0x00, 0x01, 0x03
SYN_REPORT = 0
BUS_USB = 0x03

ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ = 0x00, 0x01, 0x02, 0x03, 0x04, 0x05
ABS_HAT0X, ABS_HAT0Y = 0x10, 0x11
(BTN_A, BTN_B, BTN_C, BTN_X, BTN_Y, BTN_Z,
 BTN_TL, BTN_TR, BTN_TL2, BTN_TR2,
 BTN_SELECT, BTN_START, BTN_MODE, BTN_THUMBL, BTN_THUMBR) = range(0x130, 0x13F)

PAD_NAME = "padtest-virt-pad"
PAD_RE = re.compile(r"\[PAD\] lx=(-?\d+) ly=(-?\d+) rx=(-?\d+) ry=(-?\d+) "
                    r"lt=(\d+) rt=(\d+) btns=0x([0-9a-f]+) "
                    r"fire=(\d) ads=(\d) aim_gate=(\d)")

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
g_fail = 0


def ok(cond, msg):
    global g_fail
    print(("  ok  " if cond else "  FAIL ") + msg)
    if not cond:
        g_fail += 1


class VPad:
    """uinput 虚拟手柄 (xpad 风格轴分配)"""

    # 按钮 → 逻辑位 (与 src/io/pad_input.h 的 PadBtn 位表一致)
    BITS = {BTN_A: 0x0001, BTN_B: 0x0002, BTN_X: 0x0004, BTN_Y: 0x0008,
            BTN_TL: 0x0010, BTN_TR: 0x0020, BTN_SELECT: 0x0040,
            BTN_START: 0x0080, BTN_MODE: 0x0100,
            BTN_THUMBL: 0x0200, BTN_THUMBR: 0x0400}

    def __init__(self):
        self.fd = os.open("/dev/uinput", os.O_WRONLY | os.O_NONBLOCK)
        for b in self.BITS:
            fcntl.ioctl(self.fd, UI_SET_KEYBIT, b)
        self.axes = [(ABS_X, -32767, 32767), (ABS_Y, -32767, 32767),
                     (ABS_RX, -32767, 32767), (ABS_RY, -32767, 32767),
                     (ABS_Z, 0, 1023), (ABS_RZ, 0, 1023),
                     (ABS_HAT0X, -1, 1), (ABS_HAT0Y, -1, 1)]
        for code, mn, mx in self.axes:
            fcntl.ioctl(self.fd, UI_SET_ABSBIT, code)
            # struct uinput_abs_setup { __u16 code; (filler); input_absinfo(6×s32) }
            fcntl.ioctl(self.fd, UI_ABS_SETUP,
                        struct.pack("Hxx6i", code, 0, mn, mx, 0, 0, 0))
        fcntl.ioctl(self.fd, UI_SET_EVBIT, EV_KEY)
        fcntl.ioctl(self.fd, UI_SET_EVBIT, EV_ABS)
        fcntl.ioctl(self.fd, UI_SET_EVBIT, EV_SYN)
        # struct uinput_setup { input_id(4×u16); char name[80]; __u32 ff_max }
        setup = struct.pack("HHHH", BUS_USB, 0x1234, 0x5678, 0)
        setup += PAD_NAME.encode()[:79].ljust(80, b"\0")
        setup += struct.pack("I", 0)
        fcntl.ioctl(self.fd, UI_DEV_SETUP, setup)
        fcntl.ioctl(self.fd, UI_DEV_CREATE)
        self.node = self._find_node()

    def _find_node(self, timeout=3.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                with open("/proc/bus/input/devices") as f:
                    for blk in f.read().split("\n\n"):
                        if ('N: Name="%s"' % PAD_NAME) in blk:
                            m = re.search(r"H: Handlers=.*\b(event\d+)\b", blk)
                            if m:
                                return "/dev/input/" + m.group(1)
            except OSError:
                pass
            time.sleep(0.05)
        sys.exit("✗ 未找到虚拟手柄节点")

    def destroy(self):
        try:
            fcntl.ioctl(self.fd, UI_DEV_DESTROY)
        except OSError:
            pass
        os.close(self.fd)

    def _emit(self, typ, code, val):
        tv = time.time()
        os.write(self.fd, struct.pack("llHHi", int(tv), int(tv % 1 * 1e6),
                                      typ, code, val))

    def syn(self):
        self._emit(EV_SYN, SYN_REPORT, 0)

    def key(self, code, down):
        self._emit(EV_KEY, code, 1 if down else 0)
        self.syn()

    def axis(self, code, val):
        self._emit(EV_ABS, code, val)
        self.syn()

    def reset(self):
        """回中: 摇杆 0 (±32767 域中心), 扳机/dpad 0"""
        for code in (ABS_X, ABS_Y, ABS_RX, ABS_RY, ABS_Z, ABS_RZ,
                     ABS_HAT0X, ABS_HAT0Y):
            self.axis(code, 0)


class Dump:
    """跟随 aimbot 日志; 断言一律针对最新快照 ([PAD] 为状态行而非事件队列)"""

    def __init__(self, path):
        self.path = path
        self.offset = 0
        self.pads = []

    def drain(self):
        with open(self.path, errors="replace") as f:
            f.seek(self.offset)
            chunk = f.read()
            self.offset = f.tell()
        for line in chunk.splitlines():
            m = PAD_RE.search(line)
            if m:
                g = m.groups()
                self.pads.append({"lx": int(g[0]), "ly": int(g[1]),
                                  "rx": int(g[2]), "ry": int(g[3]),
                                  "lt": int(g[4]), "rt": int(g[5]),
                                  "btns": int(g[6], 16), "fire": g[7],
                                  "ads": g[8], "gate": g[9]})

    def snap(self):
        self.drain()
        return self.pads[-1] if self.pads else None

    def expect(self, field, value, timeout=2.0):
        """轮询至最新 [PAD] 行的 field == value (50ms 节流; 超时返回 None 记失败)"""
        deadline = time.time() + timeout
        p = None
        while time.time() < deadline:
            p = self.snap()
            if p is not None and p[field] == value:
                return p
            time.sleep(0.05)
        ok(False, "2s 内未等到 %s=%s (最新: %s)" % (field, value, p))
        return p

    def lines(self):
        with open(self.path, errors="replace") as f:
            return f.read()


def settle(d, secs):
    time.sleep(secs)
    return d.snap()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="pad", choices=("pad", "p5g"),
                    help="输出后端 (两者共用输入/合并/发布链; 断言面相同)")
    ap.add_argument("--aimbot", default=os.path.join(ROOT, "bin", "aimbot"))
    ap.add_argument("--model", default=os.path.join(ROOT, "engine", "apex.engine"))
    ap.add_argument("--cam", default="/dev/video0")
    ap.add_argument("--log", default="/tmp/pad_e2e.log")
    args = ap.parse_args()

    print("== 准备: 虚拟手柄 + 启动 aimbot (-M %s --pad-dump -P padtest)" % args.mode)
    pad = VPad()
    print("  虚拟手柄节点:", pad.node,
          "(uinput 无 by-id 节点, 走名字回落匹配 — 覆盖 reader 的回退路径)")
    if os.path.exists(args.log):
        os.remove(args.log)
    logf = open(args.log, "w")
    proc = subprocess.Popen(
        [args.aimbot, "-M", args.mode, "--pad-dump", "-P", "padtest",
         "-m", args.model, "-d", args.cam, "-f", "120",
         "-t", "0.4", "-y", "65", "-x", "1500", "-k", "fire", "-v", "n"],
        stdout=logf, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
    d = None
    pad2 = None
    try:
        deadline = time.time() + 15
        while time.time() < deadline:
            if "✅ 手柄: " in open(args.log, errors="replace").read():
                break
            if proc.poll() is not None:
                sys.exit("✗ aimbot 提前退出:\n" + open(args.log).read())
            time.sleep(0.1)
        else:
            sys.exit("✗ 15s 内 reader 未连接虚拟手柄:\n" + open(args.log).read())
        print("  reader 已连接虚拟手柄")
        d = Dump(args.log)
        pad.reset()
        settle(d, 0.3)

        print("== case1 按键透传 1:1")
        pad.key(BTN_A, True); p = d.expect("btns", 0x0001)
        ok(p is not None and p["btns"] == 0x0001, "A 按下 → btns=0x0001")
        pad.key(BTN_A, False); p = d.expect("btns", 0x0000)
        ok(p is not None and p["btns"] == 0x0000, "A 松开 → btns=0x0000")
        for b in (BTN_TL, BTN_TR, BTN_SELECT, BTN_MODE):
            pad.key(b, True)
        want = 0x0010 | 0x0020 | 0x0040 | 0x0100
        p = d.expect("btns", want)
        ok(p is not None and p["btns"] == want, "LB+RB+back+guide → btns=0x%04x" % want)
        for b in (BTN_TL, BTN_TR, BTN_SELECT, BTN_MODE):
            pad.key(b, False)
        settle(d, 0.15)

        print("== case2 摇杆透传 1:1 (±32767 全分辨率, 零注入时右摇杆=人类通道)")
        pad.axis(ABS_X, -32767); pad.axis(ABS_Y, 12345)
        pad.axis(ABS_RX, -1000); pad.axis(ABS_RY, 8000)
        p = d.expect("lx", -32767)
        ok(p is not None and p["ly"] == 12345, "左摇杆 ±32767 域原值直通")
        ok(p is not None and p["rx"] == -1000 and p["ry"] == 8000,
           "右摇杆原值直通 (v=0 注入, 合成=人类通道)")
        pad.reset(); settle(d, 0.15)

        print("== case3 扳机模拟量直映 (0..1023 → 0..255, 实测量程标定)")
        pad.axis(ABS_Z, 1023); pad.axis(ABS_RZ, 512)
        p = d.expect("rt", 128)
        ok(p is not None and p["lt"] == 255, "LT=1023 → 255 (满量程)")
        ok(p is not None and p["rt"] == 128, "RT=512 → round(512·255/1023)=128")
        pad.axis(ABS_RZ, 256); p = d.expect("rt", 64)
        ok(p is not None and p["rt"] == 64, "RT=256 → 64")
        pad.axis(ABS_Z, 0); pad.axis(ABS_RZ, 0); settle(d, 0.15)

        print("== case4 fire 门控 (RT≠0=fire; -k fire 下 LT 不开门)")
        pad.axis(ABS_RZ, 1023); p = d.expect("fire", "1")
        ok(p is not None and p["gate"] == "1", "RT 按下 → fire=1, aim_gate=1")
        pad.axis(ABS_RZ, 0); p = settle(d, 0.1)
        ok(p is not None and p["fire"] == "0" and p["gate"] == "1",
           "RT 松开 100ms → fire=0, KEEP_ALIVE 窗内 gate 仍 1")
        p = settle(d, 0.3)
        ok(p is not None and p["gate"] == "0", "KEEP_ALIVE (200ms) 过后 gate=0")
        pad.axis(ABS_Z, 1023); p = d.expect("ads", "1")
        ok(p is not None and p["fire"] == "0" and p["gate"] == "0",
           "LT 按下 → ads=1, -k fire 下不开门")
        pad.axis(ABS_Z, 0); settle(d, 0.15)

        print("== case5 dpad (HAT0 → 逻辑位)")
        pad.axis(ABS_HAT0X, -1); p = d.expect("btns", 0x2000)
        ok(p is not None and p["btns"] == 0x2000, "HAT0X=-1 → dpad_left (0x2000)")
        pad.axis(ABS_HAT0Y, 1); p = d.expect("btns", 0x3000)
        ok(p is not None and p["btns"] == 0x3000, "HAT0Y=1 叠加 → left+down (0x3000)")
        pad.axis(ABS_HAT0X, 0); pad.axis(ABS_HAT0Y, 0)
        p = d.expect("btns", 0x0000)
        ok(p is not None, "回中 → dpad 清零")

        print("== case6 拔出 (uinput 销毁): 清键位 + 自动重连")
        pad.axis(ABS_RZ, 1023); p = d.expect("fire", "1")
        n_pads = len(d.pads)
        pad.destroy()                     # 持 RT 状态下拔出
        p = settle(d, 2.5)
        ok("手柄断开" in d.lines(), "日志记录断开一次")
        ok(p is not None and p["rt"] == 0 and p["btns"] == 0 and p["fire"] == "0",
           "断开后键位清零 (持 RT 拔出 → rt=0, fire=0, 防卡键)")
        ok(len(d.pads) > n_pads + 10 and proc.poll() is None,
           "拔出后 dump 持续输出, 进程不崩")
        pad2 = VPad()                     # 同名重插 (event 节点可能变号)
        deadline = time.time() + 10
        while time.time() < deadline:
            if d.lines().count("✅ 手柄: ") >= 2:
                break
            d.drain(); time.sleep(0.1)
        else:
            ok(False, "重插后 10s 内未重连")
        pad2.key(BTN_A, True); p = d.expect("btns", 0x0001, timeout=4.0)
        ok(p is not None, "重连后 A → btns=0x0001 (恢复)")
        pad2.key(BTN_A, False)

        print("== case7 长稳: 干跑全程 (含 case6 前后) 覆盖 ~30s, 持续采样, 无崩溃")
        time.sleep(18.0); d.drain()
        ok(len(d.pads) > 400, "累计 [PAD] 行 >400 (≥50ms 节流持续输出, 得 %d)"
           % len(d.pads))
        ok(proc.poll() is None, "aimbot 全程存活")
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5)
            ok(d is not None and "已停止" in d.lines(), "SIGINT 体面退出 (已停止)")
        except subprocess.TimeoutExpired:
            ok(False, "SIGINT 后 5s 未退出")
            proc.kill()
        logf.close()
        try:
            pad.destroy()
        except Exception:
            pass
        if pad2 is not None:
            try:
                pad2.destroy()
            except Exception:
                pass

    print("ALL PASS" if g_fail == 0 else "FAILED (%d)" % g_fail)
    return 1 if g_fail else 0


if __name__ == "__main__":
    sys.exit(main())
