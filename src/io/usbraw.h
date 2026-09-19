// ============================================================================
//  usbraw.h — USB raw_gadget 会话承载层: 一个 USB 设备的会话生命周期 (sysfs
//    UDC 两级名字发现 → INIT → RUN) 与 ep0 标准请求应答 (描述符按 wLength
//    截断 / 状态 / 配置 / 接口 / feature), 外加中断端点的报告收发:
//    IN = "最新报告槽" 发送线程 (EP_WRITE 阻塞至主机取走), OUT = 收取线程
//    (EP_READ 阻塞; 包内容无消费方, 收到即丢 — 主机侧 LED/力反馈命令的落点)。
//    ep0 标准部分与具体设备无关 — 设备各自提供全部描述符 (含速度与端点集),
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

// 中断端点单包上限 (USB 2.0 全速中断端点 wMaxPacketSize ≤ 64)。报告长度是本库
//   各设备的报告长 (鼠标 9B / 手柄报告 20B), 均在该上限内。
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
    // 设备限定符: 只有高速设备才答 (USB 2.0 §9.6.2); nullptr = 无限定符, 该
    //   GET_DESCRIPTOR 走 STALL — 仅全速设备的规范行为。
    const usb_qualifier_descriptor* qualifier;
    const uint8_t* config;                 // 配置节原始字节串 (config/interface/类/endpoint 全段)
    uint16_t config_len;                   // = 配置节 wTotalLength
    // 报告描述符 (经标准 GET_DESCRIPTOR 的类类型取); nullptr = 无该描述符 → STALL。
    //   仅 HID 设备有; 类/协议非 HID 的设备 (厂商接口) 报 nullptr。
    const uint8_t* report_desc;
    uint16_t report_desc_len;
    // 枚举速度: 决定端点 bInterval 的时间单位 (USB 2.0 §9.6.6) — 全速按 1ms 帧
    //   计 (bInterval=4 → 4ms 轮询), 高速按 2^(bInterval-1) 个 125µs 微帧计。
    enum usb_device_speed speed;
    usb_endpoint_descriptor ep_in;         // 中断 IN 端点 (SET_CONFIGURATION 时 EP_ENABLE)
    // 中断 OUT 端点: 声明则一并 EP_ENABLE 并起收取线程 — 主机侧 LED/力反馈命令
    //   必须有人收 (只使能不收, 主机的 OUT 传输永远 NAK, 命令超时)。
    bool has_ep_out;
    usb_endpoint_descriptor ep_out;
    const UsbRawStringDef* strings;        // 字符串表; index 0 固定 LANGID 0x0409 包
    uint8_t string_count;
    UsbRawVendorHook vendor_request;       // 设备特有请求 (不设 → 一切非标准请求 STALL)
    // 发送完成间隔统计 (每 2s 打一行: 均值周期与由此得的报告率)。EP_WRITE 阻塞至
    //   主机取走, 相邻完成的间隔均值即主机服务周期 — 端点轮询率的设备侧度量。
    bool rate_trace;
};

struct UsbRawSession {
    // — 不变区 (start 注入) —
    int fd = -1;                           // /dev/raw-gadget; close 即解绑 UDC
    const UsbRawDeviceDef* dev = nullptr;

    std::atomic<bool> stopping{false};     // stop() 置位; 阻塞 ioctl 返回后线程由此退出

    // — 运行态 (mtx 保护; cv 供发送线程等 configured+新数据, out_cv 供 OUT 收取) —
    std::mutex mtx;
    std::condition_variable cv;
    std::condition_variable out_cv;
    bool configured = false;               // 主机已完成 SET_CONFIGURATION(cfg_value)
    uint8_t cfg_value = 0;                 // 当前配置值 (0 = 未配置)
    bool ep_enabled = false;               // 中断 IN 端点句柄有效
    int ep_handle = -1;                    // EP_ENABLE 返回的端点句柄 (RESET/重枚举时刷新)
    uint8_t slot[USBRAW_PKT_MAX];          // 最新报告槽
    uint16_t slot_len = 0;                 // 槽内报告长度 = EP_WRITE 长度 (提交长度即包边界)
    bool slot_fresh = false;

    // OUT 端点 (dev->has_ep_out 时): 使能/失效由控制线程, 在途读的收尾由收取线程
    //   观察 (见 usbraw.cu 的 out_ep_enable/out_ep_retire)
    bool out_enabled = false;              // 端点已使能 (控制线程维护)
    int out_handle = -1;                   // EP_ENABLE 返回的句柄
    bool out_reading = false;              // 收取线程正在 EP_READ 中 (收尾据此决定是否需唤醒)
    uint64_t out_gen = 0;                  // 配置纪元: 每次 SET_CONFIGURATION/RESET +1, 读循环据此续读

    // — 线程 (start 起, stop join; 阻塞 ioctl 由唤醒信号 + fd close 收尾) —
    std::thread ctrl_th;                   // EVENT_FETCH 循环 (唯一取事件方)
    std::thread send_th;                   // 最新报告槽 → 中断 IN 端点发送循环
    std::thread out_th;                    // 中断 OUT 端点收取循环 (仅 has_ep_out)
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
