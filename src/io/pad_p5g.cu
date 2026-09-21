// ============================================================================
//  pad_p5g.cu — pad_p5g.h 的实现: P5 General 加密狗形态的设备字节 (对 PS5),
//    逻辑态 → 64B 输入报告的组装, 真加密狗的 hidraw 主机驱动 (本机 USB 口,
//    中断 IN/OUT + 三条 Feature 控制传输), 认证状态机与签名流水线的后端循环。
//
//  设备字节逐字节锁在 GP2040-CE 固件的描述符数组上 (协议文档 §2/§3/§4, 见
//    docs/p5general/): 全速, 单配置单接口 (HID, 无子类/协议), 中断 IN 0x82
//    64B bInterval=1 (1ms 轮询), 中断 OUT 0x01 64B bInterval=6, 报告描述符
//    165B 双 Collection (Game Pad + 认证专用 vendor 0xFFF0)。无序列号
//    (iSerialNumber=0), 字符串保持参考固件取值 (Activtor / P5General / 0.1)
//    — PS5 的认证不看字符串, 与加密狗身份的一致性由 VID/PID + 认证往返承担。
//
//  位宽: 线上的摇杆字段是 8 位 (描述符声明 6×8 位绝对轴 + 参考固件把 16 位内部
//    状态右移 8 位放入该字段 — 两方一致), 而本库的逻辑域是 pad_input 的
//    step-258 展开域, 故逆映射是 floor(v/258)+128 而不是 >>8 (落点规则、往返恒等
//    的逐个码值证据、以及 >>8 在负半程与 256 的 127 个码值不整除的算术, 都写在
//    pad_p5g.h 的 p5g_stick8 上)。
//
//  对参考固件的偏离 (都是本机多线程主机栈上的鲁棒性改进, 协议语义不越界):
//    ① 传输失败一次即放弃本回合回 idle (或清 pending) — 参考固件的失败路径把
//      状态永久挂在 wait 上 (其 resetHostData 为空、无超时恢复, 协议文档 §9),
//      PS5 的超时重发会重启回合, 该死点不复制;
//    ② 加密狗失联 = 关 hidraw + 1s 周期重扫 (P5G_DONGLE_RESCAN_MS, 与手柄读取侧
//      同一纪律), 参考固件拔出后状态机不复位;
//    ③ 签名往返与认证 I/O 的看门狗 500ms (P5G_AUTH_IO_BUDGET_US) — 取值同参考
//      固件给加密狗的唯一显式时间预算 (F2 自动取回延迟 500ms): 参考固件愿意等
//      加密狗的最长时间, 就是"健康往返"的现成上界口径。
//
//  两条已知且接受的结论 (写下来是为了不被当成疏漏):
//    · 后端循环里的 hidraw write 与控制传输是阻塞调用, 没有 usbraw 那样的
//      SIGUSR1 唤醒 (内核 hidraw 的写最终走 usbhid_output_report → 同步
//      usb_interrupt_msg, 受内核控制传输超时 5s 约束)。因此最坏情况是 Ctrl+C
//      的停机被推迟到该次调用返回 (≤5s), 而不是永久挂住; 换非阻塞打开也无济
//      于事 (O_NONBLOCK 只影响 hidraw 的 read 路径, 写与 feature ioctl 不受它
//      管), 故不引入无实效的复杂性。
//    · 总线挂起时的远程唤醒不实现: 参考固件调 tud_remote_wakeup() 而其描述符
//      bmAttributes 并未声明远程唤醒 (协议文档 §10.12 自陈效果未验证), 本实现在
//      无一手依据前不做这件事 — 描述符与行为保持一致。
// ============================================================================

#include "io/pad_p5g.h"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <dirent.h>
#include <fcntl.h>
#include <linux/hidraw.h>                // HIDIOCSFEATURE / HIDIOCGFEATURE (hidraw ioctl)
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "core/state.h"                  // global_running, DEFAULT_FREQ
#include "io/pad_output.h"               // pad_publish_snapshot (发布点契约)

// ---- 常量 (全部有出处) -------------------------------------------------------
// 认证时序: F2 自动取回延迟 500ms + 加密狗 I/O 预算 500ms, 均取参考固件对加密狗
//   的唯一显式时间预算 (auth_recv_f2_us = +500ms, 协议文档 §5.5/§9) — 参考固件
//   愿意等加密狗的最长时间即健康往返的上界口径; 签名 pending 看门狗同一值。
constexpr uint64_t P5G_AUTH_F2_DELAY_US  = 500 * 1000;
constexpr uint64_t P5G_AUTH_IO_BUDGET_US = 500 * 1000;
constexpr uint64_t P5G_FLOW_PENDING_TIMEOUT_US = 500 * 1000;
// 变化后的重复发送配额: 参考固件 diff_report_repeat = 4 (协议文档 §6)
constexpr int P5G_FLOW_REPEATS = 4;
// 扳机线上数字位门 (0–255): 高于 G7 Pro 扳机轴实测抗抖带 (flat 15/255, padthr
//   缺省 6% 的同一出处) 的第一个整值 — 带内是行程起始的抖动而非有意的按压;
//   模拟量本身无阈值 1:1 直映 (pad 模式既有约束)。
constexpr uint8_t P5G_DIGIT_TRIG_MIN = 16;
// 十字键 hat 值 (GP2040-CE P5GENERAL_HAT_*): 0=上 顺时针至 7=左上, 中性 0x0F
constexpr uint8_t P5G_HAT_UP = 0, P5G_HAT_UPRIGHT = 1, P5G_HAT_RIGHT = 2,
                  P5G_HAT_DOWNRIGHT = 3, P5G_HAT_DOWN = 4, P5G_HAT_DOWNLEFT = 5,
                  P5G_HAT_LEFT = 6, P5G_HAT_UPLEFT = 7, P5G_HAT_NEUTRAL = 0x0F;
// 触摸板分辨率 1920×943 (DualSense 面板, 参考固件 P5GENERAL_TP_X/Y_MAX);
//   触点预置位置 = 中心 (参考固件 initialize 的预置)
constexpr int P5G_TP_X_MAX = 1920, P5G_TP_Y_MAX = 943;
// 加密狗缺席时的重扫周期 (ms): 与手柄读取侧同一纪律 (1s 周期重扫), 不是每拍 —
//   扫描是两条 sysfs opendir/readdir, 1kHz 下按拍扫纯属白烧 CPU (本进程已有
//   1kHz 循环 + TRT 推理), 而加密狗插拔是人工事件, 1s 的发现延迟无感。
constexpr uint64_t P5G_DONGLE_RESCAN_MS = 1000;

// ---- 设备字节 ----------------------------------------------------------------
// 设备描述符 (18B): GP2040-CE p5general_device_descriptor 逐字节
static constexpr uint8_t P5G_DEVICE_DESC[18] = {
    0x12, 0x01,              // bLength=18, bDescriptorType=DEVICE
    0x00, 0x02,              // bcdUSB = 0x0200
    0x00, 0x00, 0x00,        // bDeviceClass/SubClass/Protocol = 类在接口
    0x40,                    // bMaxPacketSize0 = 64
    0x81, 0x2B,              // idVendor  = 0x2B81 (小端)
    0x01, 0x01,              // idProduct = 0x0101
    0x01, 0x00,              // bcdDevice = 0x0001
    0x01, 0x02, 0x00,        // iManufacturer/iProduct/iSerialNumber = 1/2/无
    0x01,                    // bNumConfigurations = 1
};
static_assert(sizeof(P5G_DEVICE_DESC) == USB_DT_DEVICE_SIZE, "设备描述符长度必须是 18B");

// 配置描述符节 (41B = 9 配置 + 9 接口 + 9 HID + 7 IN + 7 OUT)
static constexpr uint8_t P5G_CONFIG[41] = {
    0x09, 0x02, 0x29, 0x00,  // 配置: wTotalLength=41
    0x01,                    // bNumInterfaces = 1
    0x01,                    // bConfigurationValue = 1
    0x00,                    // iConfiguration = 0
    0x80,                    // bmAttributes: 总线供电, 无远程唤醒
    0xFA,                    // bMaxPower = 500mA (uapi 2mA 单位由 usbraw 换算)

    0x09, 0x04,              // 接口: bInterfaceNumber=0, bAlternateSetting=0
    0x00, 0x00, 0x02,        // bNumEndpoints = 2
    0x03, 0x00, 0x00,        // 类 HID, 无子类/协议 (非 boot 接口)
    0x00,                    // iInterface = 0

    0x09, 0x21, 0x11, 0x01,  // HID: bcdHID 1.11, 国家 0
    0x00, 0x01, 0x22, 0xA5, 0x00,   // 1 个类描述符 (报告), 长度 165

    0x07, 0x05, 0x82, 0x03, 0x40, 0x00, 0x01,   // 中断 IN 0x82 / 64B / 1ms
    0x07, 0x05, 0x01, 0x03, 0x40, 0x00, 0x06,   // 中断 OUT 0x01 / 64B / 6ms
};
static_assert(sizeof(P5G_CONFIG) == 41 && P5G_CONFIG[2] == 41 && P5G_CONFIG[3] == 0,
              "配置节 wTotalLength 必须等于数组全长");

// 端点描述符 (EP_ENABLE 用) 与配置节内端点字节逐字节一致 (pad_xinput 同一校验)
static constexpr usb_endpoint_descriptor P5G_EP_IN = {
    0x07, USB_DT_ENDPOINT, 0x82, USB_ENDPOINT_XFER_INT, 64, 1,
};
static constexpr usb_endpoint_descriptor P5G_EP_OUT = {
    0x07, USB_DT_ENDPOINT, 0x01, USB_ENDPOINT_XFER_INT, 64, 6,
};
static constexpr bool p5g_ep_blob_matches(size_t off, const usb_endpoint_descriptor& e) {
    return P5G_CONFIG[off]     == e.bLength
        && P5G_CONFIG[off + 1] == e.bDescriptorType
        && P5G_CONFIG[off + 2] == e.bEndpointAddress
        && P5G_CONFIG[off + 3] == e.bmAttributes
        && P5G_CONFIG[off + 4] == (uint8_t)(e.wMaxPacketSize & 0xFF)
        && P5G_CONFIG[off + 5] == (uint8_t)(e.wMaxPacketSize >> 8)
        && P5G_CONFIG[off + 6] == e.bInterval;
}
static_assert(p5g_ep_blob_matches(27, P5G_EP_IN), "配置节 IN 端点字节与 EP_ENABLE 描述符不一致");
static_assert(p5g_ep_blob_matches(34, P5G_EP_OUT), "配置节 OUT 端点字节与 EP_ENABLE 描述符不一致");

// 报告描述符 (165B = 配置节 wDescriptorLength): GP2040-CE
//   p5general_report_descriptor 逐字节。Collection 1 = Game Pad (输入 0x01 64B /
//   输出 0x02 48B / 特征 0x03 48B / 特征 0xE0 3B), Collection 2 = vendor 0xFFF0
//   认证专用 (特征 0xF0/0xF1 64B, 0xF2 16B)。
static constexpr uint8_t P5G_REPORT_DESC[165] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x09, 0x30,        //   Usage (X)
    0x09, 0x31,        //   Usage (Y)
    0x09, 0x32,        //   Usage (Z)
    0x09, 0x35,        //   Usage (Rz)
    0x09, 0x33,        //   Usage (Rx)
    0x09, 0x34,        //   Usage (Ry)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x06,        //   Report Count (6)        ← 字节 1..6 摇杆×4 + 扳机×2
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x06, 0x00, 0xFF,  //   Usage Page (Vendor 0xFF00)
    0x09, 0x20,        //   Usage (0x20)
    0x95, 0x01,        //   Report Count (1)        ← 字节 7 reportCounter
    0x81, 0x02,        //   Input
    0x05, 0x01,        //   Usage Page (Generic Desktop)
    0x09, 0x39,        //   Usage (Hat switch)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x07,        //   Logical Maximum (7)
    0x35, 0x00,        //   Physical Minimum (0)
    0x46, 0x3B, 0x01,  //   Physical Maximum (315)
    0x65, 0x14,        //   Unit (Degrees)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)        ← 字节 8 低 4 位 hat
    0x81, 0x42,        //   Input (Data,Var,Abs,Null State)
    0x65, 0x00,        //   Unit (None)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (1)
    0x29, 0x0E,        //   Usage Maximum (14)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x0E,        //   Report Count (14)       ← 字节 8 高 4 位 + 字节 9
    0x81, 0x02,        //   Input
    0x06, 0x00, 0xFF,  //   Usage Page (Vendor 0xFF00)
    0x09, 0x21,        //   Usage (0x21)
    0x95, 0x0E,        //   Report Count (14)       ← 字节 10 高 6 位 + 字节 11 (填充)
    0x81, 0x02,        //   Input
    0x06, 0x00, 0xFF,  //   Usage Page (Vendor 0xFF00)
    0x09, 0x22,        //   Usage (0x22)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x34,        //   Report Count (52)       ← 字节 12..63
    0x81, 0x02,        //   Input
    0x85, 0x02,        //   Report ID (2)
    0x09, 0x23,        //   Usage (0x23)
    0x95, 0x2F,        //   Report Count (47)
    0x91, 0x02,        //   Output                  ← PS5 的震动/LED 走中断 OUT, 收下即丢
    0x85, 0x03,        //   Report ID (3)
    0x0A, 0x21, 0x28,  //   Usage (0x2821)
    0x95, 0x2F,        //   Report Count (47)
    0xB1, 0x02,        //   Feature                 ← GET_REPORT(0x03) 47B 载荷
    0x06, 0x80, 0xFF,  //   Usage Page (Vendor 0xFF80)
    0x85, 0xE0,        //   Report ID (0xE0)
    0x09, 0x57,        //   Usage (0x57)
    0x95, 0x02,        //   Report Count (2)
    0xB1, 0x02,        //   Feature                 ← 参考固件未实现 → STALL
    0xC0,              // End Collection
    0x06, 0xF0, 0xFF,  // Usage Page (Vendor 0xFFF0)
    0x09, 0x40,        // Usage (0x40)
    0xA1, 0x01,        // Collection (Application)  ← 认证专用
    0x85, 0xF0,        //   Report ID (0xF0)
    0x09, 0x47,        //   Usage (0x47)
    0x95, 0x3F,        //   Report Count (63)
    0xB1, 0x02,        //   Feature                 ← SET_REPORT 质询载入
    0x85, 0xF1,        //   Report ID (0xF1)
    0x09, 0x48,        //   Usage (0x48)
    0x95, 0x3F,        //   Report Count (63)
    0xB1, 0x02,        //   Feature                 ← GET_REPORT 签名/nonce
    0x85, 0xF2,        //   Report ID (0xF2)
    0x09, 0x49,        //   Usage (0x49)
    0x95, 0x0F,        //   Report Count (15)
    0xB1, 0x02,        //   Feature                 ← GET_REPORT 签名状态
    0xC0,              // End Collection
};
static_assert(sizeof(P5G_REPORT_DESC) == 165, "报告描述符必须为 165B (与配置节 wDescriptorLength 一致)");

// GET_REPORT(Feature, 0x03) 的 48B 应答 = [线上报告 ID 0x03][47B 静态载荷] (协议
//   §5.1 的线上布局: byte0 = 报告 ID, 其后为净荷), 载荷是参考固件 output_0x03 的
//   逐字节转录 (设备能力/配置位图, 语义不可考, 协议文档 §5.2; 其 47 字节已含尾部
//   的零, 本数组无额外补零 — 实测加密狗的应答逐字节相同)
static constexpr uint8_t P5G_FEATURE_03_REPLY[48] = {
    0x03,
    0x21, 0x28, 0x03, 0xC3, 0x00, 0x2C, 0x56,
    0x01, 0x00, 0xD0, 0x07, 0x00, 0x80, 0x04, 0x00,
    0x00, 0x80, 0x0D, 0x0D, 0x84, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// 字符串表: 保持参考固件取值 (改它们需要一手依据, 与 pad_xinput 同一纪律)
static const UsbRawStringDef P5G_STRINGS[] = {
    { 1, "Activtor" }, { 2, "P5General" }, { 3, "0.1" },
};

// ---- 认证状态机实例 (ep0 钩子与后端循环共享) ----------------------------------
P5gAuth& p5g_auth() {
    static P5gAuth inst;
    return inst;
}

// ---- ep0 类请求钩子 (HID 类段整体归本设备, 路由约定见 usbraw.h) ----------------
// 类请求的语义是设备特定的: 0xF0 的数据阶段必须真收下来转发加密狗, 0xF1/0xF2 的
//   应答取自认证缓冲 — 内建的 boot HID 类表只会零长度收尾, 不足以承担这些语义,
//   故本设备定义钩子后整个类段 (含无钩子设备会走内建表的 SET_IDLE/SET_PROTOCOL)
//   都在此应答; 表未接的一律 STALL (拒绝即拒得正式)。
static bool p5g_class_request(UsbRawSession& s, const usb_ctrlrequest& c) {
    const bool is_get = (c.bRequestType & 0x80) != 0;      // 0xA1 = 类|接口|IN
    if (c.bRequest == 0x09) {                              // SET_REPORT
        uint8_t buf[USBRAW_PKT_MAX];
        // 数据/状态阶段一律收尾 (wLength=0 也走一次零长度 EP0_READ): 内核 raw_gadget
        //   对未结算的 ep0 阶段持在途标记, 该会话后续全部控制传输 (含重枚举) 都会
        //   -EBUSY (见 usbraw.h 的路由约定)。读入长度 = min(wLength, 单包) — 本设备
        //   声明的 feature 报告最大 64B (0xF0/0xF1), 数据阶段因此恰为一个 ep0 包。
        const int len = c.wLength < (int)USBRAW_PKT_MAX ? c.wLength : (int)USBRAW_PKT_MAX;
        if (!usbraw_ep0_read(s, buf, len)) return true;    // 收尾失败: 链路已断
        if (c.wLength == 64 && (c.wValue >> 8) == 0x03 && (c.wValue & 0xFF) == 0xF0) {
            std::printf("ℹ [P5G] 收到 PS5 质询 (净荷类型 0x%02X)\n", buf[1]);
            std::fflush(stdout);
            p5g_auth().accept_f0(buf);                     // 质询: 仅 idle 接受, 其余静默丢弃
        }
        return true;                                       // 数据已消费 (接受或丢弃), 正常收尾
    }
    if (is_get && c.bRequest == 0x01) {                    // GET_REPORT (仅 Feature 应答)
        if ((c.wValue >> 8) != 0x03) return false;         // 非特征报告 → STALL (参考固件同)
        uint8_t buf[USBRAW_PKT_MAX];
        int len = 0;
        switch (c.wValue & 0xFF) {
        case 0x03:
            if (c.wLength < (int)sizeof(P5G_FEATURE_03_REPLY)) return false;
            std::printf("ℹ [P5G] PS5 读取设备定义 (0x03, wLength %d)\n", c.wLength);
            std::fflush(stdout);
            memcpy(buf, P5G_FEATURE_03_REPLY, sizeof(P5G_FEATURE_03_REPLY));
            len = (int)sizeof(P5G_FEATURE_03_REPLY);
            break;
        case 0xF1:
            if (c.wLength != 64) return false;
            p5g_auth().reply_f1(buf);
            len = 64;
            break;
        case 0xF2:
            if (c.wLength != 16) return false;
            p5g_auth().reply_f2(buf);
            len = 16;
            break;
        default:
            return false;                                  // 0xE0 与未知特征 ID → STALL
        }
        return usbraw_ep0_write(s, buf, len, c.wLength);
    }
    if (c.bRequest == 0x0A || c.bRequest == 0x0B) {        // SET_IDLE / SET_PROTOCOL:
        uint8_t z = 0;                                     //   无数据阶段, 零长度收尾
        return usbraw_ep0_read(s, &z, 0);
    }
    if (is_get && c.bRequest == 0x02) {                    // GET_IDLE: idle rate 0
        uint8_t zero = 0;
        return usbraw_ep0_write(s, &zero, 1, c.wLength);
    }
    if (is_get && c.bRequest == 0x03) {                    // GET_PROTOCOL: report 协议
        uint8_t one = 1;
        return usbraw_ep0_write(s, &one, 1, c.wLength);
    }
    return false;                                          // 其余 → STALL (拒绝即拒得正式)
}

const UsbRawDeviceDef& pad_p5g_usb_def() {
    static const UsbRawDeviceDef def = [] {
        UsbRawDeviceDef d{};
        memcpy(&d.device, P5G_DEVICE_DESC, sizeof(P5G_DEVICE_DESC));
        d.qualifier         = nullptr;            // 全速设备无限定符 → 该请求 STALL
        d.config            = P5G_CONFIG;
        d.config_len        = sizeof(P5G_CONFIG);
        d.report_desc       = P5G_REPORT_DESC;
        d.report_desc_len   = sizeof(P5G_REPORT_DESC);
        d.speed             = USB_SPEED_FULL;     // 端点 bInterval 按 1ms 帧解释
        d.ep_in             = P5G_EP_IN;
        d.has_ep_out        = true;               // PS5 的输出报告 (震动/LED) 收下即丢
        d.ep_out            = P5G_EP_OUT;
        d.strings           = P5G_STRINGS;
        d.string_count      = (uint8_t)(sizeof(P5G_STRINGS) / sizeof(P5G_STRINGS[0]));
        d.vendor_request    = p5g_class_request;  // 类请求段整体归钩子 (见 usbraw.h 的路由约定)
        d.rate_trace        = true;               // 签名流水线的有效报告率是运行时观测项
        d.rate_tag          = "PAD-USB";          // 与 XInput 后端同一行格式 (报告率行)
        return d;
    }();
    return def;
}

// ---- 逻辑态 → 64B 输入报告 ---------------------------------------------------
// 触摸板触点 (4B): bit7 未按下 / 7bit 计数器 / 12bit X / 12bit Y。无触摸源 =
//   双触点常态未按下于中心 (参考固件 initialize 的预置)
static void p5g_touch_unpressed(uint8_t* p) {
    p[0] = 0x80;                                  // unpressed=1, counter=0
    const int cx = P5G_TP_X_MAX / 2, cy = P5G_TP_Y_MAX / 2;
    p[1] = (uint8_t)(cx & 0xFF);
    p[2] = (uint8_t)(((cx >> 8) & 0x0F) | ((cy & 0x0F) << 4));
    p[3] = (uint8_t)((cy >> 4) & 0xFF);
}

void pad_p5g_report(const PadLogical& st, uint8_t out[PAD_P5G_REPORT_LEN]) {
    memset(out, 0, PAD_P5G_REPORT_LEN);
    out[0] = 0x01;                                // 报告 ID (输入)
    out[1] = p5g_stick8(st.lx);
    out[2] = p5g_stick8(st.ly);
    out[3] = p5g_stick8(st.rx);                   // 右摇杆 = 合并态 (注入已在合并层钳满偏)
    out[4] = p5g_stick8(st.ry);
    out[5] = st.lt;                               // 扳机模拟量 1:1 (无阈值)
    out[6] = st.rt;
    // out[7] reportCounter: 恒 0 (参考固件从不维护, 协议文档 §10.9)
    uint8_t hat = P5G_HAT_NEUTRAL;
    if (st.btns & PADBTN_DPAD_UP) {
        if (st.btns & PADBTN_DPAD_RIGHT) hat = P5G_HAT_UPRIGHT;
        else if (st.btns & PADBTN_DPAD_LEFT) hat = P5G_HAT_UPLEFT;
        else hat = P5G_HAT_UP;
    } else if (st.btns & PADBTN_DPAD_DOWN) {
        if (st.btns & PADBTN_DPAD_RIGHT) hat = P5G_HAT_DOWNRIGHT;
        else if (st.btns & PADBTN_DPAD_LEFT) hat = P5G_HAT_DOWNLEFT;
        else hat = P5G_HAT_DOWN;
    } else if (st.btns & PADBTN_DPAD_RIGHT) hat = P5G_HAT_RIGHT;
    else if (st.btns & PADBTN_DPAD_LEFT) hat = P5G_HAT_LEFT;
    out[8] = (uint8_t)(hat & 0x0F);               // bit0..3 = hat (中性 0x0F)
    // bit4..7 = west(X/方), south(A/叉), east(B/圆), north(Y/三角) — 线上顺序如此
    if (st.btns & PADBTN_X)    out[8] |= 1u << 4;
    if (st.btns & PADBTN_A)    out[8] |= 1u << 5;
    if (st.btns & PADBTN_B)    out[8] |= 1u << 6;
    if (st.btns & PADBTN_Y)    out[8] |= 1u << 7;
    if (st.btns & PADBTN_LB)    out[9] |= 1u << 0;        // l1
    if (st.btns & PADBTN_RB)    out[9] |= 1u << 1;        // r1
    if (st.lt >= P5G_DIGIT_TRIG_MIN) out[9] |= 1u << 2;   // l2/r2 数字位: 模拟量出抗抖带
    if (st.rt >= P5G_DIGIT_TRIG_MIN) out[9] |= 1u << 3;
    if (st.btns & PADBTN_BACK)  out[9] |= 1u << 4;        // select/share
    if (st.btns & PADBTN_START) out[9] |= 1u << 5;        // start/options
    if (st.btns & PADBTN_L3)    out[9] |= 1u << 6;
    if (st.btns & PADBTN_R3)    out[9] |= 1u << 7;
    if (st.btns & PADBTN_GUIDE) out[10] |= 1u << 0;       // home (PS 键)
    // 触摸板按下: 物理手柄没有这个键 — 由"分享/上传"一类键代位 (io/pad_input 的
    //   兄弟节点读取: 实测 G7 Pro 的分享键报在键盘接口的 KEY_SYSRQ)。pad 模式
    //   (XInput) 无触摸板位, 该逻辑位在那边无落点。
    if (st.btns & PADBTN_TOUCH) out[10] |= 1u << 1;
    // 字节 11 / auth_seq_number / 陀螺 / 加速度计恒 0
    out[30] = 0x1A; out[31] = 0x00;               // 特征字 0x001A (小端, 参考固件初始化即定)
    p5g_touch_unpressed(out + 32);                // 触点 1
    p5g_touch_unpressed(out + 36);                // 触点 2
    // out[56..63] hash: 恒 0 — 由加密狗回填 (PS5 校验的逐报告认证数据)
}

// ---- 认证状态机 --------------------------------------------------------------
// 逐规则移植自 GP2040-CE 的 P5GeneralAuthUSBListener (协议文档 §5 的状态机转录):
//   F0 (SET_REPORT Feature) 载入缓冲并进 SendF0 → tick 把它原样写加密狗 →
//   ACK 后按质询类型分派 F1 配额并决定是否安排 +500ms 的 F2 自动取回 → 每完成
//   一次取回回 idle。GET_REPORT(F1/F2) 的应答永远是"上一次取回的缓冲", 即
//   **滞后一次取数**: PS5 先读走旧内容, 该次读取才去武装加密狗取下一份。
void P5gAuth::accept_f0(const uint8_t* raw64) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (st_ != St::Idle) return;                  // 非 idle 静默忽略 (参考固件同规则)
    memcpy(auth_buffer_, raw64, 64);              // 线上 [0xF0][63B 载荷] 原样, 含 ID 字节
    st_ = St::SendF0;                             // 看门狗在下一拍 tick 入态武装 (≤1ms 滞后)
}

void P5gAuth::reply_f1(uint8_t* out64) {
    std::lock_guard<std::mutex> lk(mtx_);
    out64[0] = 0xF1;                              // 线上 byte0 = 本次报告 ID (协议 §5.1)
    // 净荷取缓冲跳过其 ID 字节的 63 字节 (参考固件即 memcpy(buf, auth_buffer+1, 63));
    //   "滞后一次取数"的语义落在缓冲内容上, 与 ID 字节无关 — 因此 byte0 恒为请求
    //   自身的报告 ID, 而非缓冲里残留的上一次 ID (0xF0/0xF2)。
    memcpy(out64 + 1, auth_buffer_ + 1, 63);
    if (st_ == St::Idle) { st_ = St::RecvF1; io_deadline_us_ = 0; }
}

void P5gAuth::reply_f2(uint8_t* out16) {
    std::lock_guard<std::mutex> lk(mtx_);
    out16[0] = 0xF2;
    memcpy(out16 + 1, auth_buffer_ + 1, 15);
    if (st_ == St::Idle) { st_ = St::RecvF1; io_deadline_us_ = 0; }   // 与 F1 同一武装路径
}

void P5gAuth::reset() {
    std::lock_guard<std::mutex> lk(mtx_);
    st_ = St::Idle;                               // 缓冲不清: 内容只会被整份覆盖
    f1_num_ = 0;
    io_deadline_us_ = 0;
}

void P5gAuth::set_dongle_ready(bool on) {
    std::lock_guard<std::mutex> lk(mtx_);
    dongle_ready_ = on;
}

void P5gAuth::dispatch_locked(uint64_t now_us) {  // 锁内; F0 ACK 后按类型字节分派
    switch (auth_buffer_[1]) {                    // 净荷第 1 字节 = 质询类型 (nonce 阶段)
    case 0x01: f1_num_ = 4; break;
    case 0x03: f1_num_ = 1; break;
    default:   f1_num_ = 0; break;                // 0x02 与其它合流 (参考固件同表)
    }
    const bool auto_f2 = (auth_buffer_[1] == 0x01 && auth_buffer_[3] == 3)
                      || auth_buffer_[1] == 0x02
                      || auth_buffer_[1] == 0x03;
    if (auto_f2) {
        f2_at_us_ = now_us + P5G_AUTH_F2_DELAY_US;
        io_deadline_us_ = f2_at_us_ + P5G_AUTH_IO_BUDGET_US;
        st_ = St::RecvF2Delay;
    } else {
        st_ = St::Idle;
        io_deadline_us_ = 0;
    }
}

void P5gAuth::tick(uint64_t now_us, const P5gAuthIo& io) {
    // 快照决策 (锁内) → 阻塞 I/O (锁外) → 落地 (锁内, 状态已变则结果作废)
    St st; uint8_t buf[64];
    bool act = false, ready = false;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        ready = dongle_ready_;
        if (ready && st_ != St::Idle) {           // 看门狗只在加密狗在位时计时:
            if (io_deadline_us_ == 0)             //   缺席是"等它插上", 不是"它死了"
                io_deadline_us_ = now_us + P5G_AUTH_IO_BUDGET_US;
            else if (now_us >= io_deadline_us_) {
                st_ = St::Idle;                   // 超时弃回合: 配额一并清 (与 reset 同形,
                f1_num_ = 0;                      //   否则重插后带着旧配额继续)
                io_deadline_us_ = 0;
            }
        }
        st = st_;
        if (st == St::SendF0) { memcpy(buf, auth_buffer_, 64); act = true; }
        else if (st == St::RecvF1) act = f1_num_ > 0;
    }
    if (st == St::Idle || !ready) return;         // 不在位: 武装态保持, 等 1s 重扫

    if (st == St::SendF0) {
        const bool ok = act && io.set_f0(io.ctx, buf);
        std::lock_guard<std::mutex> lk(mtx_);
        if (st_ != St::SendF0) return;            // 期间被重置/重新武装 — 结果作废
        if (ok) dispatch_locked(now_us);          // ACK 即分派 (参考固件 set_report_complete)
        else   { st_ = St::Idle; io_deadline_us_ = 0; }   // 一次失败即弃回合 (见文件头①)
        return;
    }
    if (st == St::RecvF1) {
        uint8_t r[64];
        const bool ok = act && io.get_f1(io.ctx, r);
        std::lock_guard<std::mutex> lk(mtx_);
        if (st_ != St::RecvF1) return;
        if (ok) {
            memcpy(auth_buffer_, r, 64);          // 取回的下一份 F1 落缓冲 → PS5 下次 GET 读到
            f1_num_ -= 1;
            st_ = St::Idle;
            io_deadline_us_ = 0;
        } else { st_ = St::Idle; io_deadline_us_ = 0; }
        return;
    }
    // RecvF2Delay: 到时才取 (取前确认仍属本回合; 失败/重置都落 idle)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (st_ != St::RecvF2Delay || now_us < f2_at_us_) return;
    }
    uint8_t r[16];
    const bool ok = io.get_f2(io.ctx, r);
    std::lock_guard<std::mutex> lk(mtx_);
    if (st_ != St::RecvF2Delay) return;
    if (ok) {
        memcpy(auth_buffer_, r, 16);
        st_ = St::Idle;
        io_deadline_us_ = 0;
    } else { st_ = St::Idle; io_deadline_us_ = 0; }
}

// ---- 签名流水线 --------------------------------------------------------------
bool p5g_flow_submit(const uint8_t* rpt64, bool dongle_ready, P5gFlow& f,
                     uint64_t now_us, uint8_t* out) {
    if (!dongle_ready || f.pending) return false; // 加密狗未就绪/单份在途: 不产生新报告
    if (!f.have_last || memcmp(rpt64, f.last, PAD_P5G_REPORT_LEN) != 0) {
        memcpy(f.last, rpt64, PAD_P5G_REPORT_LEN);
        f.have_last = true;
        f.repeat = P5G_FLOW_REPEATS;
        f.pending = true;
        f.pending_us = now_us;
        memcpy(out, rpt64, PAD_P5G_REPORT_LEN);
        return true;
    }
    if (f.repeat > 0) {                           // 内容不变: 重发同一份直至配额耗尽
        f.repeat -= 1;
        f.pending = true;
        f.pending_us = now_us;
        memcpy(out, f.last, PAD_P5G_REPORT_LEN);
        return true;
    }
    return false;                                 // 空闲静默 (无变化且配额耗尽)
}

bool p5g_flow_signed(const uint8_t* in64, P5gFlow& f) {
    // 单槽背压 (参考固件 rule: 上一份未发走则丢弃新到签名)。生产循环里本分支
    //   实际不可达 — 后端每拍至多读一份签名并当拍就交给会话的"最新报告槽", 槽
    //   本身即覆盖语义 (不排队), 故没有需要丢弃的积压; 保留该判据是为契约完整
    //   (helper 可被单独驱动, 见 io/pad_test.cu 的 P5G 流水线段)。
    if (f.ready) return false;
    memcpy(f.finish, in64, PAD_P5G_REPORT_LEN);
    f.ready = true;
    return true;
}

void p5g_flow_sent(P5gFlow& f) {
    f.ready = false;
    f.pending = false;                            // 一份签名报告的完整回合结束
}

void p5g_flow_tick(P5gFlow& f, uint64_t now_us) {
    if (f.pending && now_us - f.pending_us >= P5G_FLOW_PENDING_TIMEOUT_US)
        // 签名丢失: 放弃该份, 报告流不卡死。放弃后不强制重发 — 内容未变且重复
        //   配额已尽时线上继续静默, 直到下一次输入变化 (内容驱动的既有语义;
        //   目标静止且律空闲时无影响)。
        f.pending = false;
}

// ---- 加密狗 hidraw 主机驱动 ---------------------------------------------------
// 识别: /sys/bus/hid/devices/0003:2B81:0101.*/hidraw/hidrawN (内核 hid 侧任意
//   驱动绑定均保留 hidraw 节点)。中断 IN = read, 中断 OUT = write (内核按设备
//   是否声明 OUT 端点自动选择中断管线或控制管线), Feature 控制传输 = HIDIOCS/
//   GFEATURE。其 evdev 摇杆节点由 pad_input 显式排除 (不是人手通道)。
struct P5gDongle {
    int fd = -1;
    bool dead = false;                            // I/O 失败标记: 主循环关闭重扫
};

static uint64_t p5g_now_us() {
    return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

static bool p5g_dongle_scan(std::string& node) {
    DIR* dp = opendir("/sys/bus/hid/devices");
    if (!dp) return false;
    char prefix[32];
    snprintf(prefix, sizeof(prefix), "0003:%04X:%04X.", P5G_DONGLE_VID, P5G_DONGLE_PID);
    const size_t plen = strlen(prefix);
    dirent* de;
    bool found = false;
    while (!found && (de = readdir(dp)) != nullptr) {
        if (strncmp(de->d_name, prefix, plen) != 0) continue;
        const std::string hp = std::string("/sys/bus/hid/devices/") + de->d_name + "/hidraw";
        if (DIR* hd = opendir(hp.c_str())) {
            dirent* he;
            while ((he = readdir(hd)) != nullptr) {
                if (strncmp(he->d_name, "hidraw", 6) == 0) {
                    node = "/dev/" + std::string(he->d_name);
                    found = true;
                    break;
                }
            }
            closedir(hd);
        }
    }
    closedir(dp);
    return found;
}

static bool p5g_dongle_open(P5gDongle& d) {
    std::string node;
    if (!p5g_dongle_scan(node)) return false;
    d.fd = open(node.c_str(), O_RDWR);
    if (d.fd < 0) {
        static bool perm_logged = false;          // 权限问题只报一次 (可行动原因)
        if (errno == EACCES && !perm_logged) {
            perm_logged = true;
            std::cerr << "❌ [P5G] " << node << " 无权限 — 启动脚本经 sudo 运行, "
                         "或补 udev 规则 (SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"2b81\")\n";
        }
        return false;
    }
    d.dead = false;
    p5g_auth().set_dongle_ready(true);
    std::cout << "✅ [P5G] 加密狗: " << node << " (VID 2B81 PID 0101)\n";
    return true;
}

static void p5g_dongle_close(P5gDongle& d, const char* why) {
    if (d.fd >= 0) close(d.fd);
    d.fd = -1;
    d.dead = false;
    p5g_auth().set_dongle_ready(false);
    std::cerr << "⚠ [P5G] 加密狗失联 (" << why << "), 1s 重扫\n";
}

// 三条 Feature 控制传输 (阻塞; 失败 = 标记失联 + 返回 false, 状态机放弃本回合)。
//   成功各打一行 — 一个 PS5 握手产出几行, 运行期状态机 idle 不打 (bring-up 观测点)
static bool p5g_io_set_f0(void* ctx, const uint8_t* buf64) {
    P5gDongle& d = *static_cast<P5gDongle*>(ctx);
    uint8_t tmp[64];
    memcpy(tmp, buf64, 64);
    if (d.fd < 0 || ioctl(d.fd, HIDIOCSFEATURE(64), tmp) != 64) {
        d.dead = true;
        return false;
    }
    std::printf("ℹ [P5G] 质询已转发加密狗 (类型 0x%02X)\n", tmp[1]);
    std::fflush(stdout);
    return true;
}
static bool p5g_io_get_f1(void* ctx, uint8_t* buf64) {
    P5gDongle& d = *static_cast<P5gDongle*>(ctx);
    if (d.fd < 0) { d.dead = true; return false; }
    buf64[0] = 0xF1;                              // hidraw 约定: 首字节 = 报告 ID
    if (ioctl(d.fd, HIDIOCGFEATURE(64), buf64) != 64) { d.dead = true; return false; }
    return true;
}
static bool p5g_io_get_f2(void* ctx, uint8_t* buf16) {
    P5gDongle& d = *static_cast<P5gDongle*>(ctx);
    if (d.fd < 0) { d.dead = true; return false; }
    buf16[0] = 0xF2;
    if (ioctl(d.fd, HIDIOCGFEATURE(16), buf16) != 16) { d.dead = true; return false; }
    std::printf("ℹ [P5G] 签名状态: %02X %02X (状态字节 = 净荷[0])\n", buf16[1], buf16[2]);
    std::fflush(stdout);
    return true;
}

// ---- 后端循环 ----------------------------------------------------------------
namespace {

UsbRawSession g_usb;
std::thread g_loop_th;

// 后端循环 (拍率 = DEFAULT_FREQ): ① PS5 配置下降沿 → 认证回合与流水线复位 →
//   ② 加密狗在位管理 (1s 周期重扫) → ③ 认证状态机一拍 → ④ 读加密狗签名回包 →
//   ⑤ 合并态 → 变化驱动提交加密狗。发往 PS5 的字节只来自 ④。
void p5g_loop(UsbRawSession* s) {
    P5gDongle dg;
    P5gFlow flow;
    P5gAuth& auth = p5g_auth();
    const P5gAuthIo io{ &dg, p5g_io_set_f0, p5g_io_get_f1, p5g_io_get_f2 };
    bool ps5_configured = false;
    bool logged_absent = false;
    uint64_t next_scan_us = 0;                    // 0 = 立即允许首次扫描
    const auto period = std::chrono::microseconds(1000000 / DEFAULT_FREQ);
    auto next = std::chrono::steady_clock::now();

    while (!s->stopping) {
        next += period;
        std::this_thread::sleep_until(next);
        const uint64_t now = p5g_now_us();

        // ① PS5 配置下降沿 (重枚举/挂起): 认证回合与流水线复位 — 重连后的握手
        //    从干净状态重新开始
        bool cfg;
        { std::lock_guard<std::mutex> lk(s->mtx); cfg = s->configured; }
        if (ps5_configured && !cfg) {
            auth.reset();
            flow = P5gFlow{};
        }
        ps5_configured = cfg;

        // ② 加密狗在位管理: 失联关闭后 1s 周期重扫, 缺席静默重试 (缺席只记一次,
        //    与手柄读取侧同一纪律); 在位前 PS5 侧不会有任何输入报告 (参考固件同规则)
        if (dg.fd < 0) {
            if (now >= next_scan_us) {
                next_scan_us = now + P5G_DONGLE_RESCAN_MS * 1000;
                if (p5g_dongle_open(dg)) logged_absent = false;
            }
            if (dg.fd < 0 && !logged_absent && global_running) {
                std::cout << "⚠ [P5G] 加密狗未在位 (VID 2B81 PID 0101), 每秒重扫\n";
                logged_absent = true;
            }
        } else if (dg.dead) {
            p5g_dongle_close(dg, "传输失败");
            continue;
        }

        // ③ 认证状态机 (转发质询 / 取回签名与状态; idle 时无操作)
        auth.tick(now, io);
        if (dg.dead) continue;                    // 本拍 I/O 打死加密狗: 下拍走重扫

        // ④ 加密狗签名回读 (中断 IN; 单槽: 上一份未发走则丢弃新到的) —
        //    发往 PS5 的字节永远是加密狗返回的字节
        if (dg.fd >= 0) {
            struct pollfd pfd{}; pfd.fd = dg.fd; pfd.events = POLLIN;
            if (poll(&pfd, 1, 0) > 0) {
                uint8_t inr[64];
                const ssize_t n = read(dg.fd, inr, sizeof(inr));
                if (n == (ssize_t)sizeof(inr)) {
                    if (p5g_flow_signed(inr, flow)) {
                        usbraw_submit(*s, flow.finish, PAD_P5G_REPORT_LEN);   // → PS5 IN
                        p5g_flow_sent(flow);
                    }
                } else if (n <= 0 && errno != EINTR && errno != EAGAIN) {
                    p5g_dongle_close(dg, errno == 0 ? "连接关闭" : strerror(errno));
                    continue;
                }
            }
        }

        // ⑤ 组装合并态 → 提交加密狗签名 (变化驱动 + 4 次重复 + 单份在途)
        p5g_flow_tick(flow, now);
        if (dg.fd >= 0) {
            uint8_t rpt[PAD_P5G_REPORT_LEN], out[PAD_P5G_REPORT_LEN];
            pad_p5g_report(pad_publish_snapshot(), rpt);
            if (p5g_flow_submit(rpt, true, flow, now, out)) {
                if (write(dg.fd, out, PAD_P5G_REPORT_LEN) != (ssize_t)PAD_P5G_REPORT_LEN) {
                    p5g_dongle_close(dg, "签名提交失败");
                    continue;
                }
            }
        }
    }
}

} // namespace

bool pad_p5g_start() {
    if (!usbraw_start(g_usb, pad_p5g_usb_def())) return false;
    g_loop_th = std::thread(p5g_loop, &g_usb);
    return true;
}

void pad_p5g_stop() {
    usbraw_stop(g_usb);                           // 置 stopping → 后端循环自行退出 (≤ 一拍)
    if (g_loop_th.joinable()) g_loop_th.join();
}
