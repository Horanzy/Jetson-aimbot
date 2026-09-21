// ============================================================================
//  pad_xinput.cu — pad_xinput.h 的实现: 微软有线 Xbox 360 手柄 (0x045E/0x028E)
//    的设备字节, 20 字节输入报告的组装, 以及"发布点 → 报告槽"的报告循环。
//
//  身份与形态 (类/子类/协议 = FF/5D/01 的厂商接口, 中断 IN 0x81 + 中断
//    OUT 0x02, 无 HID 报告描述符): Windows 的 xusb 按 VID/PID + 该接口三元组
//    绑定 XInput, 绑定不看字符串也不涉签名, 而**接口描述符之后紧跟的厂商描述符
//    (0x21) 是它的必要形态** — functionfs/gadgetfs 那条路正是卡在这里, 故设备栈
//    走 raw_gadget。字节表取自 tinyusb-xinput 的参考实现 (与正版有线手柄的
//    lsusb dump 一致: 48B 配置节 / 16B 的 0x21 blob / 双 32B 中断端点),
//    字符串取该参考的中性取值 (GENERIC / XINPUT CONTROLLER)。
//
//  终端刷新率由 枚举速度 × 端点 bInterval 两件事合成 (USB 2.0 §9.6.6): 全速按
//    1ms 帧计、bInterval 即帧数, 高速按 2^(bInterval−1) 个 125µs 微帧计。参考
//    固件用高速 + bInterval=4 (2^3 微帧 = 1ms = 1000Hz), 与实测一致; 全速 +
//    bInterval=1 也声明 1000Hz 轮询且设备侧的写完成率同样是 1000Hz (mean 1.00ms),
//    但**宿主侧的 XInput 状态不跟随**: 报告的包号在涨 (≈940–1000Hz) 而
//    XInputGetState 的字段停在初始值 (实测: 同一恒定合并态 lx=−32767/rt=128/
//    btns=0x0001 下, 全速+bInterval=1 报全 0, 高速+bInterval=4 逐字段相符)。
//    故终端刷新率的实现取高速 + bInterval=4: 宿主按 1ms 取走报告, 状态如实跟随。
//    全速设备无限定符 (该 GET_DESCRIPTOR 走 STALL); 高速设备必须答限定符 —
//    本定义给出同一设备的全速运行形态 (USB 2.0 §9.6.2)。
// ============================================================================

#include "io/pad_xinput.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include <linux/usb/ch9.h>
#include <unistd.h>                      // gethostname (序列号身份源的最末回落)

#include "core/state.h"
#include "io/pad_output.h"               // pad_publish_snapshot (发布点契约)

// ---- 设备字节 --------------------------------------------------------------
// 设备描述符 (18B): bcdUSB 2.0 + 厂商类三元组 + 64B ep0 + 微软有线手柄身份。
static constexpr uint8_t XINPUT_DEVICE_DESC[18] = {
    0x12, 0x01,              // bLength=18, bDescriptorType=DEVICE
    0x00, 0x02,              // bcdUSB = 0x0200
    0xFF, 0xFF, 0xFF,        // bDeviceClass/SubClass/Protocol = 厂商自定义
    0x40,                    // bMaxPacketSize0 = 64 (全速 ep0 上限)
    0x5E, 0x04,              // idVendor  = 0x045E (Microsoft)
    0x8E, 0x02,              // idProduct = 0x028E (有线 360 手柄)
    0x72, 0x05,              // bcdDevice = 0x0572 (参考实现的取值)
    0x01, 0x02, 0x03,        // iManufacturer/iProduct/iSerialNumber = 1/2/3
    0x01,                    // bNumConfigurations = 1
};
static_assert(sizeof(XINPUT_DEVICE_DESC) == USB_DT_DEVICE_SIZE, "设备描述符长度必须是 18B");

// 枚举速度与端点间隔: 终端刷新率的两个旋钮 (推导见文件头)。高速设备的规范行为
//   是提供设备限定符 (该 GET_DESCRIPTOR 必须答), 本定义据此给出全速运行形态。
static constexpr enum usb_device_speed XINPUT_SPEED = USB_SPEED_HIGH;
static constexpr uint8_t XINPUT_EP_IN_INTERVAL  = 4;    // 高速: 2^3×125µs = 1ms → 1000Hz
static constexpr uint8_t XINPUT_EP_OUT_INTERVAL = 8;    // 主机→设备 (LED/力反馈) 低频即可

// 配置描述符节 (48B): 单接口双端点 + 一个 16B 的类特定 (0x21) 描述符 —— XInput
//   的厂商接口没有 HID 报告描述符, 主机的 XInput 栈不看该描述符的内容, 但它的
//   长度/类型字节与端点地址必须自洽, 且它是 host 侧枚举能通过的必要形态 (见文件头)。
static constexpr uint8_t XINPUT_CONFIG[48] = {
    0x09, 0x02, 0x30, 0x00,  // bLength=9, CONFIG, wTotalLength=0x0030=48
    0x01,                    // bNumInterfaces = 1
    0x01,                    // bConfigurationValue = 1
    0x00,                    // iConfiguration = 0
    0x80,                    // bmAttributes: 总线供电, 无远程唤醒
    0xFA,                    // bMaxPower = 250 (单位 2mA) = 500mA (VBUS 请求由本字节导出)

    0x09, 0x04,              // INTERFACE
    0x00,                    // bInterfaceNumber = 0
    0x00,                    // bAlternateSetting = 0
    0x02,                    // bNumEndpoints = 2
    0xFF, 0x5D, 0x01,        // bInterfaceClass/SubClass/Protocol = FF/5D/01 (XInput 绑定键)
    0x00,                    // iInterface = 0

    0x10, 0x21,              // 类特定描述符: bLength=16, bDescriptorType=0x21
    0x10, 0x01, 0x01, 0x24,  //   厂商字节 (含义未公开; 主机侧不解析)
    0x81,                    //   对应 IN 端点地址
    0x14,                    //   厂商字节
    0x03, 0x00, 0x03, 0x13, 0x02, 0x00, 0x03, 0x00,   //   厂商字节

    0x07, 0x05, 0x81, 0x03, 0x20, 0x00, XINPUT_EP_IN_INTERVAL,   // 中断 IN 0x81 / 32B
    0x07, 0x05, 0x02, 0x03, 0x20, 0x00, XINPUT_EP_OUT_INTERVAL,  // 中断 OUT 0x02 / 32B
};
static_assert(sizeof(XINPUT_CONFIG) == 48 && XINPUT_CONFIG[2] == 48 && XINPUT_CONFIG[3] == 0,
              "配置节 wTotalLength 必须等于数组全长");

// 端点描述符 (EP_ENABLE 用): 与配置节内的端点字节必须逐字节一致 — 主机看到的
//   端点描述符与内核使能的端点描述符不符, 就是枚举与端点行为的脱节。
static constexpr usb_endpoint_descriptor XINPUT_EP_IN = {
    0x07, USB_DT_ENDPOINT, 0x81, USB_ENDPOINT_XFER_INT, 32, XINPUT_EP_IN_INTERVAL,
};
static constexpr usb_endpoint_descriptor XINPUT_EP_OUT = {
    0x07, USB_DT_ENDPOINT, 0x02, USB_ENDPOINT_XFER_INT, 32, XINPUT_EP_OUT_INTERVAL,
};
constexpr bool xinput_ep_blob_matches(size_t off, const usb_endpoint_descriptor& e) {
    return XINPUT_CONFIG[off]     == e.bLength
        && XINPUT_CONFIG[off + 1] == e.bDescriptorType
        && XINPUT_CONFIG[off + 2] == e.bEndpointAddress
        && XINPUT_CONFIG[off + 3] == e.bmAttributes
        && XINPUT_CONFIG[off + 4] == (uint8_t)(e.wMaxPacketSize & 0xFF)
        && XINPUT_CONFIG[off + 5] == (uint8_t)(e.wMaxPacketSize >> 8)
        && XINPUT_CONFIG[off + 6] == e.bInterval;
}
static_assert(xinput_ep_blob_matches(34, XINPUT_EP_IN), "配置节 IN 端点字节与 EP_ENABLE 描述符不一致");
static_assert(xinput_ep_blob_matches(41, XINPUT_EP_OUT), "配置节 OUT 端点字节与 EP_ENABLE 描述符不一致");

// ---- 身份字符串 -------------------------------------------------------------
// 序列号 (iSerialNumber) 由 **本机身份** 派生, 不用任何公开常量:
//   主机侧的设备实例 ID 直接取自它 (实测: 本机残留的 XInput 实例就是 ...\1.0
//   (参考固件的固定串) 与 ...\<12 位派生串> 两种形态), 于是它承担唯一性 —
//   固定串会让两个单元撞同一个实例, 且把设备指纹写成公开常量。
//   派生规则 (可复现): 源 = /etc/machine-id, 缺失退到 /var/lib/dbus/machine-id,
//   再退到 hostname; 取 FNV-1a 64 位 → 12 位大写十六进制。派生而非随机: 重启/
//   重插保持同一身份 (宿主不会每次都当新设备)。三个源都读不到时退回固定串并告警。
//   设备定义的改动**不需要**换实例, 这一点是实测的: 同一派生串在旧定义 (全速
//   bInterval=1) 与新定义 (高速 bInterval=4) 之间复用同一实例, 宿主每次都重读
//   描述符并重新绑定, 定义变更不会被实例缓存挡住 (实例缓存不是设备失效的原因,
//   见 AGENTS.md 的 Pad output)。故派生输入不含描述符字节 — 否则每改一个描述符
//   常量都会在宿主侧留下一个新的陈旧实例, 而换来的保护是零。
constexpr uint64_t FNV_OFFSET = 1469598103934665603ULL;
constexpr uint64_t FNV_PRIME  = 1099511628211ULL;
static uint64_t fnv1a(const void* p, size_t n, uint64_t h) {
    const unsigned char* b = (const unsigned char*)p;
    for (size_t i = 0; i < n; ++i) { h ^= b[i]; h *= FNV_PRIME; }
    return h;
}
static std::string read_first_line(const char* path) {
    std::ifstream f(path);
    std::string s;
    if (f.good() && std::getline(f, s) && !s.empty()) return s;
    return {};
}
static const std::string& xinput_serial() {
    static const std::string s = [] {
        std::string id;
        for (const char* p : { "/etc/machine-id", "/var/lib/dbus/machine-id" }) {
            id = read_first_line(p);
            if (!id.empty()) break;
        }
        if (id.empty()) {
            char host[256] = {0};
            if (gethostname(host, sizeof(host) - 1) == 0 && host[0]) id = host;
        }
        if (id.empty()) {
            std::cerr << "⚠ 无可用本机身份 (machine-id/hostname 皆不可读): 序列号退回固定串\n";
            return std::string("000000000001");
        }
        const uint64_t h = fnv1a(id.data(), id.size(), FNV_OFFSET);
        char buf[16];
        snprintf(buf, sizeof(buf), "%012llX", (unsigned long long)(h & 0xFFFFFFFFFFFFULL));
        return std::string(buf);
    }();
    return s;
}

// 设备限定符 (高速设备必答): 描述同一设备的全速运行形态 (USB 2.0 §9.6.2 排除
//   与设备描述符相同的字段, 其余必须与设备描述符一致)。
static const usb_qualifier_descriptor XINPUT_QUALIFIER = {
    (uint8_t)sizeof(usb_qualifier_descriptor), USB_DT_DEVICE_QUALIFIER,
    0x0200, 0xFF, 0xFF, 0xFF, 0x40, 1, 0,
};

const UsbRawDeviceDef& pad_xinput_usb_def() {
    static const UsbRawStringDef XINPUT_STRINGS[] = {
        { 1, "GENERIC" }, { 2, "XINPUT CONTROLLER" }, { 3, xinput_serial().c_str() },
    };
    static const UsbRawDeviceDef def = [] {
        UsbRawDeviceDef d{};
        memcpy(&d.device, XINPUT_DEVICE_DESC, sizeof(XINPUT_DEVICE_DESC));
        d.qualifier         = &XINPUT_QUALIFIER;
        d.config            = XINPUT_CONFIG;
        d.config_len        = sizeof(XINPUT_CONFIG);
        d.report_desc       = nullptr;            // 厂商接口无报告描述符
        d.report_desc_len   = 0;
        d.speed             = XINPUT_SPEED;
        d.ep_in             = XINPUT_EP_IN;
        d.has_ep_out        = true;               // 主机的 LED/力反馈命令落在 OUT 0x02
        d.ep_out            = XINPUT_EP_OUT;
        d.strings           = XINPUT_STRINGS;
        d.string_count      = (uint8_t)(sizeof(XINPUT_STRINGS) / sizeof(XINPUT_STRINGS[0]));
        d.vendor_request    = nullptr;            // 厂商请求段 STALL (xusb 的初始化探测据此通过)
        d.rate_trace        = true;               // 主机轮询率是本后端的实测项 (见 usbraw)
        d.rate_tag          = "PAD-USB";
        return d;
    }();
    return def;
}

// ---- 20 字节输入报告 -------------------------------------------------------
// 位表 (线格式的事实源, 与 Linux xpad 的 xpad360_process_packet 对 data[2]/[3]
//   的取位逐位对应; 报头 00 14 = 控制数据 / 长度 20, 也是它认包的判据):
//   byte2: bit0-3 = 十字上/下/左/右, bit4 START, bit5 BACK, bit6 L3, bit7 R3
//   byte3: bit0 LB, bit1 RB, bit2 Guide, bit3 保留=0, bit4 A, bit5 B, bit6 X, bit7 Y
//   byte4/5 = LT/RT 模拟量 0–255
//   byte6..13 = LX/LY/RX/RY, int16 小端, 中心 0, 满偏 ±32767
//   byte14..19 = 保留零
static void put_i16le(uint8_t* p, int v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

// Y 轴口径转换: 逻辑态 "上/左为负" ↔ 线上 "上为正"。满偏取反仍须是满偏 —
//   -(-32768) 超出 int16 域, 以 int 运算后饱和到 PAD_AXIS_MAX。
static int y_axis_wire(int16_t logical) {
    return std::clamp(-(int)logical, -PAD_AXIS_MAX, PAD_AXIS_MAX);
}

void pad_xinput_report(const PadLogical& st, uint8_t out[PAD_XINPUT_REPORT_LEN]) {
    memset(out, 0, PAD_XINPUT_REPORT_LEN);
    out[0] = 0x00;                        // 报头: 控制数据
    out[1] = (uint8_t)PAD_XINPUT_REPORT_LEN;
    if (st.btns & PADBTN_DPAD_UP)    out[2] |= 1u << 0;
    if (st.btns & PADBTN_DPAD_DOWN)  out[2] |= 1u << 1;
    if (st.btns & PADBTN_DPAD_LEFT)  out[2] |= 1u << 2;
    if (st.btns & PADBTN_DPAD_RIGHT) out[2] |= 1u << 3;
    if (st.btns & PADBTN_START)      out[2] |= 1u << 4;
    if (st.btns & PADBTN_BACK)       out[2] |= 1u << 5;
    if (st.btns & PADBTN_L3)         out[2] |= 1u << 6;
    if (st.btns & PADBTN_R3)         out[2] |= 1u << 7;
    if (st.btns & PADBTN_LB)         out[3] |= 1u << 0;
    if (st.btns & PADBTN_RB)         out[3] |= 1u << 1;
    if (st.btns & PADBTN_GUIDE)      out[3] |= 1u << 2;
    if (st.btns & PADBTN_A)          out[3] |= 1u << 4;
    if (st.btns & PADBTN_B)          out[3] |= 1u << 5;
    if (st.btns & PADBTN_X)          out[3] |= 1u << 6;
    if (st.btns & PADBTN_Y)          out[3] |= 1u << 7;
    out[4] = st.lt;
    out[5] = st.rt;
    put_i16le(out + 6,  st.lx);
    put_i16le(out + 8,  y_axis_wire(st.ly));
    put_i16le(out + 10, st.rx);
    put_i16le(out + 12, y_axis_wire(st.ry));
}

// ---- 报告循环 --------------------------------------------------------------
namespace {

UsbRawSession g_usb;
std::thread g_report_th;

// 报告循环: 与发布点同拍 (DEFAULT_FREQ) 取最新合并态 → 20B 报告 → 会话报告槽。
//   槽位是"最新覆盖"语义, 重复提交与跳拍等价 (后者只在 seq 未变时省一次组装),
//   恒提使主机侧报告流与实体手柄一样连续 — 主机每次轮询都有报告可取, 报告率即
//   端点轮询率 (发送节拍由 usbraw 发送线程阻塞在 EP_WRITE 决定; 提交率与写完成
//   率都由它打印, 两个数字相等即无积压)。
void report_loop(UsbRawSession* s) {
    const auto period = std::chrono::microseconds(1000000 / DEFAULT_FREQ);
    auto next = std::chrono::steady_clock::now();
    while (!s->stopping) {
        next += period;
        std::this_thread::sleep_until(next);
        uint8_t rpt[PAD_XINPUT_REPORT_LEN];
        pad_xinput_report(pad_publish_snapshot(), rpt);
        usbraw_submit(*s, rpt, PAD_XINPUT_REPORT_LEN);
    }
}

} // namespace

bool pad_xinput_start() {
    if (!usbraw_start(g_usb, pad_xinput_usb_def())) return false;
    g_report_th = std::thread(report_loop, &g_usb);
    return true;
}

void pad_xinput_stop() {
    usbraw_stop(g_usb);                   // 置 stopping → 报告循环自行退出 (≤ 一拍)
    if (g_report_th.joinable()) g_report_th.join();
}
