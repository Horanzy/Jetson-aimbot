#!/usr/bin/env python3
# ==============================================================================
#  p5g_dongle_probe.py — P5 General 加密狗 bring-up 探针 (Jetson, sudo)
#
#  在加密狗第一次插上本机时运行, 逐项核对固件 (io/pad_p5g) 依赖的每一条主机侧
#  契约, 并把协议文档 (docs/p5general/) 第 10 节"未能从代码确定的事项"里能由
#  实测回答的部分当场回答:
#    ① 枚举: VID/PID 2B81:0101 在 /sys/bus/hid 的形态、绑定的内核驱动、hidraw 节点
#    ② 描述符: 加密狗自身的 HID 报告描述符 (sysfs report_descriptor, 逐字节 dump)
#    ③ 控制传输: GET_REPORT(Feature) 三条 — 0x03 (48B 定义) / 0xF1 (64B 签名) /
#       0xF2 (16B 签名状态)
#    ④ 签名流水线: 中断 OUT 写入 64B 输入报告 → 中断 IN 读回 (hash 由加密狗填写),
#       往返时延与回包对请求的 1:1 关系
#    ⑤ 附带: evdev 节点 (pad_input 需要排除它) / raw_gadget + UDC 就绪度
#
#  纯 stdlib, 只读 + 一条 OUT 写入 (不进认证、不改设备状态)。run: sudo python3 ...
# ==============================================================================
import fcntl
import os
import select
import sys
import time

VID, PID = 0x2B81, 0x0101
# asm-generic _IOC: dir(2b)@30 | size(14b)@16 | type(8b)@8 | nr(8b)@0; 'H'=0x48,
#   dir = WRITE|READ = 1|2 = 3 (hidraw 的 FEATURE ioctl 声明双向缓冲)
def _IOC(nr, size):
    return (3 << 30) | (0x48 << 8) | nr | (size << 16)
HIDIOCSFEATURE = lambda n: _IOC(0x06, n)     # SET_REPORT(feature) 控制传输
HIDIOCGFEATURE = lambda n: _IOC(0x07, n)     # GET_REPORT(feature) 控制传输

def fail(msg):
    print("  ❌ " + msg)

def ok(msg):
    print("  ✅ " + msg)

def info(msg):
    print("  ℹ " + msg)

def find_hid_device():
    base = "/sys/bus/hid/devices"
    if not os.path.isdir(base):
        return None, "无 /sys/bus/hid"
    for name in sorted(os.listdir(base)):
        # hid 设备目录名: bbbb:VVVV:PPPP.inst (4 位十六进制大写, 冒号三段)
        try:
            bus, vid, pid = name.split(".")[0].split(":")
        except ValueError:
            continue
        if bus == "0003" and int(vid, 16) == VID and int(pid, 16) == PID:
            return os.path.join(base, name), name
    return None, None

def hidraw_node(devdir):
    hp = os.path.join(devdir, "hidraw")
    if not os.path.isdir(hp):
        return None
    entries = [e for e in os.listdir(hp) if e.startswith("hidraw")]
    return "/dev/" + entries[0] if entries else None

def xread(path, binary=False):
    try:
        with open(path, "rb" if binary else "r") as f:
            return f.read()
    except OSError:
        return None

def main():
    print("== P5 General 加密狗探针 (VID %04X PID %04X) ==" % (VID, PID))
    checks = 0

    # ① 枚举形态
    devdir, name = find_hid_device()
    if not devdir:
        fail("加密狗未枚举 (无 0003:2B81:0101.* 于 /sys/bus/hid) — 确认插在 Jetson "
             "主机口, lsusb 应见 2b81:0101")
        return 1
    ok("已枚举: %s" % name)
    checks += 1
    drv = os.path.basename(os.readlink(os.path.join(devdir, "driver"))) \
        if os.path.exists(os.path.join(devdir, "driver")) else "(无)"
    info("绑定驱动: %s" % drv)
    uev = xread(os.path.join(devdir, "uevent")) or ""
    for line in uev.splitlines():
        if line.startswith(("HID_ID", "HID_NAME")):
            info(line)

    # ② 加密狗自身报告描述符 (文档 §10.6 的实测回答)
    rd = xread(os.path.join(devdir, "report_descriptor"), binary=True)
    if rd:
        ok("报告描述符 %d 字节 (已 dump 到 /tmp/p5g_dongle_report_desc.bin)" % len(rd))
        with open("/tmp/p5g_dongle_report_desc.bin", "wb") as f:
            f.write(rd)
        print("     前 64 字节: " + rd[:64].hex(" "))
    else:
        info("sysfs 无 report_descriptor (内核较旧) — 跳过")

    # ③ hidraw 与权限
    node = hidraw_node(devdir)
    if not node:
        fail("无 hidraw 子节点 — 无法驱动")
        return 1
    ok("hidraw 节点: %s" % node)
    try:
        fd = os.open(node, os.O_RDWR)
    except OSError as e:
        fail("%s 无法读写 (%s) — 用 sudo 运行本探针/启动脚本" % (node, e))
        return 1
    ok("hidraw 打开成功 (O_RDWR)")
    checks += 1

    # evdev 附属节点 (pad_input 排除项的存在性确认)
    evs = [e for e in os.listdir("/dev/input") if e.startswith("event")]
    info("/dev/input 下 event* 共 %d 个 (加密狗的摇杆节点已被 pad_input 按 VID/PID 排除)" % len(evs))

    def get_feature(rid, n, label):
        buf = bytearray(n)
        buf[0] = rid
        try:
            r = fcntl.ioctl(fd, HIDIOCGFEATURE(n), buf, True)
        except OSError as e:
            fail("%s: GET_REPORT(0x%02X) 失败 (%s)" % (label, rid, e))
            return None
        if r != n:
            fail("%s: GET_REPORT(0x%02X) 返回 %dB (期望 %dB)" % (label, rid, r, n))
            return None
        ok("%s: GET_REPORT(0x%02X) → %dB: %s" % (label, rid, r, bytes(buf[:min(16, n)]).hex(" ")))
        return bytes(buf)

    # ④ 三条 Feature 控制传输 (认证未开始: 0xF1/0xF2 应答内容无意义, 只验通路)
    f03 = get_feature(0x03, 48, "设备定义")
    if f03 and f03[1:3] == b"\x21\x28":
        info("  0x03 载荷开头 21 28 与 Usage 0x2821 回显一致 (与参考固件 output_0x03 同形)")
    get_feature(0xF1, 64, "签名/nonce")
    f2 = get_feature(0xF2, 16, "签名状态")
    if f2:
        info("  0xF2 净荷 (认证前): %s" % f2[1:].hex(" "))
    checks += 1

    # ⑤ 签名流水线: OUT 写 → IN 读, 测往返
    rpt = bytearray(64)
    rpt[0] = 0x01                       # 输入报告 ID
    for i in range(1, 5):
        rpt[i] = 0x80                   # 四摇杆轴中位
    rpt[30] = 0x1A                      # 特征字 0x001A (小端)
    n_ok, t_min, t_sum = 0, 1e9, 0.0
    hashes = set()
    try:
        for k in range(8):
            t0 = time.monotonic()
            w = os.write(fd, bytes(rpt))
            if w != 64:
                fail("中断 OUT 写入返回 %d (期望 64)" % w)
                break
            # 等 IN 回包 (500ms 足够分辨 1:1 应答与无应答)
            t1 = time.monotonic()
            r, _, _ = select.select([fd], [], [], 0.5)
            if not r:
                print("     (第 %d 次写入后 500ms 内无 IN 回包)" % (k + 1))
                continue
            data = os.read(fd, 64)
            dt = (time.monotonic() - t0) * 1000
            n_ok += 1
            t_min, t_sum = min(t_min, dt), t_sum + dt
            hashes.add(bytes(data[56:64]))
            if k == 0:
                ok("OUT→IN 往返 %.1fms, 回包 %dB, 末 8B (hash): %s"
                   % (dt, len(data), data[56:64].hex(" ")))
                diff = [i for i in range(56) if data[i] != rpt[i]]
                info("  回包与请求前 56 字节%s (差异字节: %s)"
                     % ("一致" if not diff else "存在差异", diff or "无"))
    except OSError as e:
        fail("中断 OUT 写入失败 (%s) — 设备可能无 OUT 端点, 固件需退控制管线" % e)
    if n_ok:
        ok("签名往返: %d/8 次, 均值 %.1fms (最快 %.1fms) → 有效报告率 ≈ %dHz"
           % (n_ok, t_sum / n_ok, t_min, int(1000 / (t_sum / n_ok))))
        if len(hashes) == 1 and bytes(data[56:64]) == b"\0" * 8:
            info("  注意: 8 次回包 hash 全零 — 若 PS5 要求逐报告签名, 需先走认证握手再判定")
        checks += 1
    else:
        info("  无 IN 回包: 对照 GP2040-CE 行为 (挂载即持续轮询 IN), 请确认加密狗已"
             "完成自身上电; 固件的 500ms pending 看门狗可容忍此类静默")

    os.close(fd)

    # ⑥ 设备侧就绪度
    if os.path.exists("/dev/raw-gadget") and os.listdir("/sys/class/udc"):
        ok("设备侧: /dev/raw-gadget 与 UDC 就绪 (插 PS5 的口)")
        checks += 1
    else:
        fail("设备侧: /dev/raw-gadget 或 UDC 缺失 — sudo bash scripts/setup_mouse.sh")

    print("== %s (%d/%d 项通过) ==" %
          ("加密狗侧就绪, 可启动 -M p5g 对 PS5 联调" if n_ok else "流水线未验证 — 见上行说明",
           checks, 5))
    return 0

if __name__ == "__main__":
    sys.exit(main())
