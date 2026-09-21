#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================================
#  uinput_calib_test.py — 标定链路的真机验收 (Jetson 本机, 与 AI 检测无关)。
#
#  用 /dev/uinput 合成输入设备按模式驱动一次完整标定轮, 并断言四件事:
#    [1] 触发: hid = 鼠标双侧键 (BTN_SIDE+BTN_EXTRA = HID 侧键位 0x10|0x08) 长按 5s;
#        pad  = L3+R3 长按 5s。触发后日志必须出现 [标定] 触发行。
#    [2] 激励波形: 状态机按计划播放 (每条激励段一行日志); pad 侧额外从 --pad-dump 的
#        [PAD] 行断言 "整只手柄由激励独占" —— 右摇杆按计划偏转 (±满偏×设计偏转比例),
#        左摇杆/两扳机归中, 按键照旧透传 (L3/R3 到得了游戏)。
#    [3] 标定期采样率: 日志给出实测采样间隔/等效 fps/采集率/无样本帧占比/每帧采样耗时
#        (几何 = 640 裁切 → 320 相关域, 9 块×2 轴一维投影) —— "不假设 120fps" 的落点。
#    [4] 判定与回写: 结论只有两种 —— 成功 (点头 + 只回写本输出模式的延迟 VAR:
#        hid → HID_L_EST, pad → PAD_L_EST, p5g → P5G_L_EST) 或失败 (摇头 + 原因, 什么都不写)。本脚本不
#        制造屏幕运动, 故**期望失败**且脚本里的 VAR 逐字未动 (屏幕静止/无响应是最常见
#        的现场), 这本身就是一条验收: 绝不写编造的值。
#
#  跑法 (仓库根, 需要 uinput/evdev 权限, sudo 运行; 采集卡与模型按需给出):
#    sudo python3 scripts/test/uinput_calib_test.py --mode hid|pad|p5g \
#        [--aimbot bin/aimbot] [--model engine/apex.engine] [--cam /dev/video0] [--fps 120]
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
import tempfile
import time

UI_SET_EVBIT   = 0x40045564
UI_SET_KEYBIT  = 0x40045565
UI_SET_RELBIT  = 0x40045566
UI_SET_ABSBIT  = 0x40045567
UI_ABS_SETUP   = 0x401C5504
UI_DEV_SETUP   = 0x405C5503
UI_DEV_CREATE  = 0x5501
UI_DEV_DESTROY = 0x5502
EV_SYN, EV_KEY, EV_REL, EV_ABS = 0x00, 0x01, 0x02, 0x03
SYN_REPORT = 0
BUS_USB = 0x03

# 鼠标 (hid 模式的触发通道): 侧键 = BTN_SIDE/BTN_EXTRA → HID 位 0x08/0x10 (core/control.h)
BTN_LEFT, BTN_RIGHT, BTN_MIDDLE, BTN_SIDE, BTN_EXTRA = 0x110, 0x111, 0x112, 0x113, 0x114
REL_X, REL_Y, REL_WHEEL = 0x00, 0x01, 0x08
# 手柄 (pad 模式): L3/R3 = 触发, 扳机/摇杆用来验证"标定期归中"
ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ = 0x00, 0x01, 0x02, 0x03, 0x04, 0x05
BTN_A, BTN_THUMBL, BTN_THUMBR = 0x130, 0x13D, 0x13E

MOUSE_NAME = "calibtest-virt-mouse"
PAD_NAME   = "calibtest-virt-pad"
PAD_RE = re.compile(r"\[PAD\] lx=(-?\d+) ly=(-?\d+) rx=(-?\d+) ry=(-?\d+) "
                    r"lt=(\d+) rt=(\d+) btns=0x([0-9a-f]+)")
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

g_fail = 0


def ok(cond, msg):
    global g_fail
    print(("  ok  " if cond else "  FAIL ") + msg)
    if not cond:
        g_fail += 1


class UInput:
    """极简 uinput 设备: 键位/轴按 profiles 声明, 事件按 struct input_event 写。"""

    def __init__(self, name, keys=(), rels=(), abses=()):
        self.name = name
        self.fd = os.open("/dev/uinput", os.O_WRONLY | os.O_NONBLOCK)
        for k in keys:
            fcntl.ioctl(self.fd, UI_SET_KEYBIT, k)
        for r in rels:
            fcntl.ioctl(self.fd, UI_SET_RELBIT, r)
        for code, mn, mx in abses:
            fcntl.ioctl(self.fd, UI_SET_ABSBIT, code)
            fcntl.ioctl(self.fd, UI_ABS_SETUP, struct.pack("Hxx6i", code, 0, mn, mx, 0, 0, 0))
        fcntl.ioctl(self.fd, UI_SET_EVBIT, EV_KEY)
        if rels:
            fcntl.ioctl(self.fd, UI_SET_EVBIT, EV_REL)
        if abses:
            fcntl.ioctl(self.fd, UI_SET_EVBIT, EV_ABS)
        fcntl.ioctl(self.fd, UI_SET_EVBIT, EV_SYN)
        setup = struct.pack("HHHH", BUS_USB, 0x1234, 0x9abc, 0)
        setup += name.encode()[:79].ljust(80, b"\0") + struct.pack("I", 0)
        fcntl.ioctl(self.fd, UI_DEV_SETUP, setup)
        fcntl.ioctl(self.fd, UI_DEV_CREATE)
        self.node = self._find_node()

    def _find_node(self, timeout=6.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            for blk in open("/proc/bus/input/devices").read().split("\n\n"):
                if ('N: Name="%s"' % self.name) in blk:
                    m = re.search(r"H: Handlers=.*\b(event\d+)\b", blk)
                    if m:
                        return "/dev/input/" + m.group(1)
            time.sleep(0.05)
        sys.exit("✗ 未找到虚拟设备节点: " + self.name)

    def emit(self, typ, code, val):
        tv = time.time()
        os.write(self.fd, struct.pack("llHHi", int(tv), int(tv % 1 * 1e6), typ, code, val))

    def key(self, code, down):
        self.emit(EV_KEY, code, 1 if down else 0)
        self.emit(EV_SYN, SYN_REPORT, 0)

    def axis(self, code, val):
        self.emit(EV_ABS, code, val)
        self.emit(EV_SYN, SYN_REPORT, 0)

    def destroy(self):
        try:
            fcntl.ioctl(self.fd, UI_DEV_DESTROY)
        except OSError:
            pass
        os.close(self.fd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="hid", choices=("hid", "pad", "p5g"))
    ap.add_argument("--aimbot", default=os.path.join(ROOT, "bin", "aimbot"))
    ap.add_argument("--model", default=os.path.join(ROOT, "engine", "apex.engine"))
    ap.add_argument("--cam", default="/dev/video0")
    ap.add_argument("--fps", default="120", choices=("120", "60"))
    args = ap.parse_args()

    work = tempfile.mkdtemp(prefix="calibtest-")
    script = os.path.join(work, "profile.sh")
    logp = os.path.join(work, "aimbot.log")
    l_var = {"hid": "HID_L_EST", "pad": "PAD_L_EST", "p5g": "P5G_L_EST"}[args.mode]
    with open(script, "w") as f:          # 三套输出各一格延迟, 都写成模板的守卫形式
        f.write("#!/bin/bash\nHID_SPDX=\"${HID_SPDX:-100}\"\n"
                'HID_L_EST="${HID_L_EST:-60.0}"\n'
                'PAD_L_EST="${PAD_L_EST:-60.0}"\n'
                'P5G_L_EST="${P5G_L_EST:-60.0}"\n')
    os.chmod(script, 0o755)
    before = open(script).read()

    if args.mode == "hid":
        dev = UInput(MOUSE_NAME, keys=(BTN_LEFT, BTN_RIGHT, BTN_MIDDLE, BTN_SIDE, BTN_EXTRA),
                     rels=(REL_X, REL_Y, REL_WHEEL))
        dev_args = ["-D", dev.node]
        trig = (BTN_SIDE, BTN_EXTRA)
    else:
        dev = UInput(PAD_NAME, keys=(BTN_A, BTN_THUMBL, BTN_THUMBR),
                     abses=((ABS_X, -32767, 32767), (ABS_Y, -32767, 32767),
                            (ABS_RX, -32767, 32767), (ABS_RY, -32767, 32767),
                            (ABS_Z, 0, 1023), (ABS_RZ, 0, 1023)))
        dev_args = ["-M", args.mode, "-P", PAD_NAME, "-T", "6", "--pad-dump"]
        trig = (BTN_THUMBL, BTN_THUMBR)
    print("虚拟设备: %s" % dev.node)
    print("回写 VAR: %s (模式 %s)" % (l_var, args.mode))

    cmd = ["sudo", "-n", args.aimbot, "-m", args.model, "-c", "0", "-t", "0.5", "-y", "65",
           "-d", args.cam, "-f", args.fps, "-x", "2000", "-l", "60",
           "-S", script, "-k", "fire", "-a", "y"] + dev_args
    log = open(logp, "w")
    p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=ROOT)
    try:
        time.sleep(6)                                  # 等 USB 会话/采集/AI 线程起来
        if args.mode != "hid":                         # 人手通道给非零值: 标定期必须被归中
            dev.axis(ABS_X, 30000); dev.axis(ABS_Y, -20000); dev.axis(ABS_Z, 512)
            dev.key(BTN_A, True)
            time.sleep(0.5)
        for k in trig:
            dev.key(k, True)
        time.sleep(6)                                  # 长按 6s > 5s 触发判据
        for k in trig:
            dev.key(k, False)
        time.sleep(24)                                 # 整轮 (触发 5s + 播放 + 回执)
    finally:
        p.send_signal(signal.SIGINT)
        time.sleep(2)
        try:
            p.kill()
        except OSError:
            pass
        log.close()
        dev.destroy()

    text = open(logp, errors="replace").read()
    lines = text.splitlines()

    print("[1] 触发")
    ok("[标定] 触发" in text, "长按触发键 5s → 进入标定 (日志出现 [标定] 触发)")
    dev_line = "已独占" if args.mode == "hid" else "手柄节点"
    ok(dev_line in text and "raw_gadget 会话" in text,
       "输入通道在位 (%s) + raw_gadget 会话已建立 (标定的注入通道)" % dev_line)

    print("[2] 激励波形")
    segs = [l for l in lines if re.search(r"\[标定\] 段 \d+ [XY]", l)]
    ok(len(segs) >= 1, "状态机按计划播放激励段 (逐段日志 %d 行)" % len(segs))
    ok(any("超时" in l or "屏速" in l for l in segs),
       "每段给出实测行程/屏速或不可测原因 (现场证据在日志里)")
    if args.mode != "hid":        # pad 与 p5g 共用同一条激励/合并链, 波形断言一致
        pads = [m.groups() for m in (PAD_RE.search(l) for l in lines) if m]
        rx = [int(g[2]) for g in pads]
        exc = [v for v in rx if v != 0]
        ok(any(v > 0 for v in exc) and any(v < 0 for v in exc),
           "右摇杆按计划双向偏转 (激励幅度 %s)" % sorted(set(exc))[:4])
        ok(all(int(g[0]) == 0 and int(g[1]) == 0 and int(g[4]) == 0 and int(g[5]) == 0
               for g in pads if int(g[2]) != 0),
           "标定期整只手柄由激励独占: 左摇杆与两扳机归中")
        ok(any(int(g[6], 16) & 0x600 == 0x600 for g in pads),
           "按键照旧透传 (L3+R3 位在标定期仍然到达游戏)")

    print("[3] 标定期采样率")
    m = re.search(r"\[标定\] 采样: 有效样本 (\d+) / 采集帧 (\d+), 均值 ([\d.]+)ms ≈ (\d+)fps "
                  r"\(采集率 (\d+) fps, 无样本帧 ([\d.]+)%\) \| 采样耗时 ([\d.]+)ms/帧", text)
    ok(m is not None, "日志给出该轮采样率/采集率/无样本帧占比/每帧采样耗时")
    if m:
        smp, frames, ms, fps, camfps, miss, cost = m.groups()
        print("      样本 %s / 帧 %s, 均值 %sms ≈ %sfps (采集率 %s), 无样本 %s%%, "
              "采样耗时 %sms/帧" % (smp, frames, ms, fps, camfps, miss, cost))
        ok(int(smp) >= int(frames) * 0.9, "有效样本覆盖绝大多数采集帧 (采样跟得上采集)")
        ok(float(cost) < 1000.0 / int(camfps) * 0.5,
           "每帧采样耗时 < 采集周期的 50%% (640→320 保帧率的验收点)")

    print("[4] 判定与回写 (本脚本不制造屏幕运动 → 期望如实失败)")
    after = open(script).read()
    if "[标定] L=" in text:
        # 成功: 只写本模式那一格, 且守卫写法原样保住 (值落在 ${...:-默认} 的默认位上)
        ok((l_var + '="${' + l_var + ":-") in after,
           "成功路径: 只回写本输出模式的延迟 VAR, 守卫写法保留 (%s)" % l_var)
        others = [v for v in ("HID_L_EST", "PAD_L_EST", "P5G_L_EST") if v != l_var]
        ok(all((v + '="${' + v + ":-60.0}\"") in after for v in others),
           "另两套输出的延迟槽逐字未动")
    else:
        ok("无法测量" in text, "失败路径: 给出原因 (画面完全静止/无响应/尾迹被截断 …)")
        ok(after == before, "失败路径不写回: 脚本逐字未动 (绝不写编造的值)")
    killed = os.path.join(work, "aimbot.log")
    print("  日志: %s" % killed)

    print("ALL PASS" if not g_fail else "FAILED")
    return 1 if g_fail else 0


if __name__ == "__main__":
    sys.exit(main())
