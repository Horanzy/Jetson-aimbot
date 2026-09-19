// ============================================================================
//  usbraw.h — USB raw_gadget 会话承载层: 一个 USB 设备的会话生命周期 (sysfs
//    UDC 两级名字发现 → INIT → RUN) 与 ep0 标准请求应答 (描述符按 wLength
//    截断 / 状态 / 配置 / 接口 / feature), 外加中断 IN 端点的"最新报告槽"
//    发送线程。ep0 标准部分与具体 HID 报告无关 — 设备各自提供全部描述符,
//    设备特有 (类/vendor) 请求仅经可选钩子应答, 不设即 STALL (拒绝即拒得
//    正式, 不静默吞掉)。
// ============================================================================

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

#include <linux/usb/ch9.h>       // usb_ctrlrequest / 描述符结构 / USB_DT_*, USB_REQ_*

// 中断 IN 端点单包上限 (USB 2.0; 本库报告 = HID_REPORT_LEN)。ep0 应答与单包
//   无关 — GET_DESCRIPTOR 可跨包送全段, 缓冲上限单独由 USBRAW_DESC_MAX 界定。
constexpr size_t USBRAW_PKT_MAX = 64;
// ep0 描述符应答缓冲上限 (字节): HID 报告描述符 wDescriptorLength 虽为 u16,
//   现实设备均在数百字节内 (本库鼠标 75B), 取 512 为充足上限; usbraw_start
//   校验设备定义 (config/report) 不超限, 超限拒绝启动而非静默截断。
constexpr size_t USBRAW_DESC_MAX = 512;

// 设备特有请求钩子: 返回 true = 已正式应答 (数据/状态阶段的 ep0 ioctl 已由钩子
//   自行完成, 可用下方 usbraw_ep0_* 手段); false = 未处理 (usbraw STALL)。
struct UsbRawSession;
using UsbRawVendorHook = bool (*)(UsbRawSession&, const usb_ctrlrequest&);

struct UsbRawStringDef { uint8_t index; const char* utf8; };   // 窄串 UTF-8 (ASCII 1:1 映射)

// 设备定义: 描述符全部由设备侧提供, usbraw 不持有任何具体设备知识。
struct UsbRawDeviceDef {
    usb_device_descriptor device;          // 设备描述符 (18B)
    usb_qualifier_descriptor qualifier;    // 设备限定符 (高速枚举必答, USB 2.0 §9.6.2)
    const uint8_t* config;                 // 配置节原始字节串 (config/interface/类/endpoint 全段)
    uint16_t config_len;                   // = 配置节 wTotalLength
    const uint8_t* report_desc;            // 报告描述符 (经标准 GET_DESCRIPTOR 的类类型取)
    uint16_t report_desc_len;
    usb_endpoint_descriptor ep_in;         // 唯一中断 IN 端点 (SET_CONFIGURATION 时 EP_ENABLE)
    const UsbRawStringDef* strings;        // 字符串表; index 0 固定 LANGID 0x0409 包
    uint8_t string_count;
    UsbRawVendorHook vendor_request;       // 设备特有请求 (鼠标不设 → 一切非标准请求 STALL)
};

struct UsbRawSession {
    // — 不变区 (start 注入) —
    int fd = -1;                           // /dev/raw-gadget; close 即解绑 UDC
    const UsbRawDeviceDef* dev = nullptr;

    std::atomic<bool> stopping{false};     // stop() 置位; 阻塞 ioctl 返回后线程由此退出

    // — 运行态 (mtx 保护; cv 供发送线程等 configured+新数据) —
    std::mutex mtx;
    std::condition_variable cv;
    bool configured = false;               // 主机已完成 SET_CONFIGURATION(cfg_value)
    uint8_t cfg_value = 0;                 // 当前配置值 (0 = 未配置)
    bool ep_enabled = false;               // 中断 IN 端点句柄有效
    int ep_handle = -1;                    // EP_ENABLE 返回的端点句柄 (RESET/重枚举时刷新)
    uint8_t slot[USBRAW_PKT_MAX];          // 最新报告槽 (有效长度 = ep_in.wMaxPacketSize)
    bool slot_fresh = false;

    // — 线程 (start 起, stop join; 两者的阻塞 ioctl 均以 fd close 收尾) —
    std::thread ctrl_th;                   // EVENT_FETCH 循环 (唯一取事件方)
    std::thread send_th;                   // 最新报告槽 → 中断 IN 端点发送循环
};
// 槽位模型即语义: 相对移动报告由生产者保证 "槽里是未发送的累计量" — 每拍 extract
//   出的就是自上拍累计, submit 覆盖写入并置 fresh; 报告率 = min(拍率, 主机服务率)。

bool usbraw_start(UsbRawSession& s, const UsbRawDeviceDef& dev);
void usbraw_stop(UsbRawSession& s);
void usbraw_submit(UsbRawSession& s, const uint8_t* rpt, uint16_t len);

// 设备特有请求钩子的正式应答手段 (阻塞至对应 ep0 阶段完成; 失败返回 false)
bool usbraw_ep0_write(UsbRawSession& s, const void* data, int len, int wLength);
bool usbraw_ep0_read(UsbRawSession& s, void* data, int max_len);
void usbraw_ep0_stall(UsbRawSession& s);
