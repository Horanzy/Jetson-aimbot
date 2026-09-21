// ============================================================================
//  usbraw.h — USB raw_gadget 会话承载层: 一个 USB 设备的会话生命周期
//    (sysfs UDC 两级名字发现 → INIT → RUN → VBUS_DRAW) 与 ep0 标准请求应答
//    (描述符按 wLength 截断 / 状态 / 配置 / 接口 / feature), 外加中断 IN
//    端点的 "最新报告槽" 发送线程 (EP_WRITE 阻塞至主机取走)。
//
//  会话层与报告协议无关: 枚举速度、设备限定符、配置节、端点集、字符串表与
//    类请求语义全部由设备定义给定 (见 UsbRawDeviceDef), 本层不持有任何具体
//    设备的字节知识 — 报告长度与字段含义属于设备侧。
// ============================================================================

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

#include <linux/usb/ch9.h>       // usb_ctrlrequest / 描述符结构 / USB_DT_*, USB_REQ_*

// 中断端点单包上限 (USB 2.0 全速/高速中断端点 wMaxPacketSize ≤ 64)
constexpr size_t USBRAW_PKT_MAX = 64;
// ep0 应答缓冲上限 (字节): 报告描述符 wDescriptorLength 是 u16, 而现实设备的
//   该类描述符都在数百字节内; usbraw_start 校验设备定义不超此限, 超限拒绝启动
//   而非静默截断。
constexpr size_t USBRAW_DESC_MAX = 512;

// 设备特有请求钩子: 返回 true = 已正式应答 (数据/状态阶段的 ep0 ioctl 已由钩子
//   自行完成, 手段见本文件末尾的 usbraw_ep0_*); false = 未处理, 由本层 STALL。
// 路由约定: 定义了钩子的设备, **整个类请求段 (bmRequestType=CLASS) 也归钩子** —
//   类请求的语义是设备特定的 (feature report 的转发/应答、厂商页面的读写),
//   内建 HID 类请求表只服务无钩子的 boot HID 鼠标; 未定义钩子的设备保持
//   类段 → 内建表, vendor 段 → STALL。
struct UsbRawSession;
using UsbRawVendorHook = bool (*)(UsbRawSession&, const usb_ctrlrequest&);

struct UsbRawStringDef { uint8_t index; const char* utf8; };   // 窄串 UTF-8 (ASCII 1:1 映射成 UTF-16LE)

// 设备定义: 描述符与端点集全部由设备侧提供。
struct UsbRawDeviceDef {
    usb_device_descriptor device;          // 设备描述符 (18B)
    // 设备限定符: 只有高速设备才答 (USB 2.0 §9.6.2); nullptr = 无限定符, 该
    //   GET_DESCRIPTOR 走 STALL — 仅全速设备的规范行为。
    const usb_qualifier_descriptor* qualifier;
    const uint8_t* config;                 // 配置节原始字节串 (config/interface/类/endpoint 全段)
    uint16_t config_len;                   // = 配置节 wTotalLength
    // 报告描述符 (经标准 GET_DESCRIPTOR 的类类型取); nullptr = 无该描述符 → STALL。
    //   仅 HID 设备有; 非 HID 的厂商接口报 nullptr。
    const uint8_t* report_desc;
    uint16_t report_desc_len;
    // 枚举速度: 决定端点 bInterval 的时间单位 (USB 2.0 §9.6.6) — 全速按 1ms 帧
    //   计 (bInterval=4 → 4ms 轮询), 高速按 2^(bInterval-1) 个 125µs 微帧计。
    enum usb_device_speed speed;
    usb_endpoint_descriptor ep_in;         // 中断 IN 端点 (SET_CONFIGURATION 时 EP_ENABLE)
    const UsbRawStringDef* strings;        // 字符串表; index 0 固定 LANGID 0x0409 包
    uint8_t string_count;
    UsbRawVendorHook vendor_request;       // 设备特有请求 (不设 → 一切非标准请求 STALL)
};

struct UsbRawSession {
    // — 不变区 (start 注入) —
    int fd = -1;                           // /dev/raw-gadget; close 即解绑 UDC
    const UsbRawDeviceDef* dev = nullptr;

    std::atomic<bool> stopping{false};     // stop() 置位; 阻塞 ioctl 返回后线程由此退出

    // — 运行态 (mtx 保护; cv 供发送线程等配置完成 + 新数据) —
    std::mutex mtx;
    std::condition_variable cv;
    bool configured = false;               // 主机已完成 SET_CONFIGURATION(cfg_value)
    uint8_t cfg_value = 0;                 // 当前配置值 (0 = 未配置)
    bool ep_enabled = false;               // 中断 IN 端点句柄有效
    int ep_handle = -1;                    // EP_ENABLE 返回的端点句柄 (RESET/重枚举时刷新)
    uint8_t slot[USBRAW_PKT_MAX];          // 最新报告槽
    uint16_t slot_len = 0;                 // 槽内报告长度 = EP_WRITE 长度 (提交长度即包边界)
    bool slot_fresh = false;

    // — 线程 (start 起, stop join; 阻塞 ioctl 由唤醒信号 + fd close 收尾) —
    std::thread ctrl_th;                   // EVENT_FETCH 循环 (唯一取事件方)
    std::thread send_th;                   // 最新报告槽 → 中断 IN 端点发送循环
};

// 槽位模型即语义: 提交者只保证 "槽里是尚未发出的最新状态" — 每拍覆盖写入并置
//   fresh, 发送线程取走即发。**报告率 = min(拍率, 主机服务率)**: 提交长度即包
//   边界 (主机一次 IN 取走一次 EP_WRITE 的全部字节), 主机服务快于拍率时每次提交
//   恰好产生一次主机可见报告, 慢于拍率时多拍的提交被覆盖合并成一次 — 一拍产生
//   两次报告的通路在本模型里不存在 (槽是单份状态, 不是队列)。

bool usbraw_start(UsbRawSession& s, const UsbRawDeviceDef& dev);
void usbraw_stop(UsbRawSession& s);
void usbraw_submit(UsbRawSession& s, const uint8_t* rpt, uint16_t len);

// 设备特有请求钩子的正式应答手段 (阻塞至对应 ep0 阶段完成; 失败返回 false)
bool usbraw_ep0_write(UsbRawSession& s, const void* data, int len, int wLength);
bool usbraw_ep0_read(UsbRawSession& s, void* data, int max_len);
void usbraw_ep0_stall(UsbRawSession& s);
