// ============================================================================
//  usbraw.cu — usbraw.h 的实现: UDC 两级名字发现, raw_gadget ioctl 会话
//    (INIT → RUN → VBUS_DRAW), ep0 标准请求表 (控制线程) 与中断 IN 端点的
//    最新报告槽发送线程。
//
//  阻塞收尾: stop() 置 stopping、唤醒两个线程、close fd — close 即解绑 UDC
//    (内核 raw_gadget 驱动 raw_release → usb_gadget_unregister_driver →
//    driver.disconnect)。唤醒信号见下文 "端点 ioctl 的唤醒信号": 在途 ioctl
//    持有文件引用, 只 close 无法驱动 release, 必须先把阻塞线程叫回来。
//  SETUP 应答路由 (内核 raw_gadget 驱动 gadget_setup 的记账语义): (IN 且
//    wLength>0) 经 EP0_WRITE 送数据; OUT 方向或 wLength=0 的一律经 EP0_READ
//    收尾数据/状态阶段 (对纯状态阶段即零长度收尾) — 用 EP0_WRITE 会报
//    wrong direction。
// ============================================================================

#include "io/usbraw.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include <dirent.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/hid.h>                 // HID_DT_REPORT (类 GET_DESCRIPTOR 的类型值)
#include <linux/usb/raw_gadget.h>

#include "core/state.h"

// ioctl 数据缓冲: uapi 的 data[] 是柔性数组成员, 不能作为 struct 的中间成员
//   (C++ 非法) — 就地对齐缓冲 + 头指针重解释, 内核按 ptr + sizeof(头) 取数据;
//   头字段逐个显式赋值, 头后缓冲由内核回填 (事件) 或由 memcpy 填充 (发送)。
//   CONTROL 事件只携带 8B usb_ctrlrequest, 故事件缓冲尾部取其大小。
struct alignas(8) RawEventBuf {
    uint8_t buf[sizeof(usb_raw_event) + sizeof(usb_ctrlrequest)];
    usb_raw_event* ev() { return reinterpret_cast<usb_raw_event*>(buf); }
};
struct alignas(8) RawIoBuf {
    uint8_t buf[sizeof(usb_raw_ep_io) + USBRAW_DESC_MAX];
    usb_raw_ep_io* io() { return reinterpret_cast<usb_raw_ep_io*>(buf); }
    // data[] 的实体 — 经 cast 指针访问柔性成员会让 fortify 的对象大小推断
    //   失真 (__memcpy_chk 误判 abort), 写入一律走 buf 内偏移
    uint8_t* payload() { return buf + sizeof(usb_raw_ep_io); }
};

// ---- UDC 名字发现 ----------------------------------------------------------
// 两级名字: device_name = /sys/class/udc 首个实例目录名; driver_name = 该实例
//   父设备 driver 链接基名 (对 tegra-xudc 即 "tegra-xudc")。raw_gadget 无
//   "空名 = 任意 UDC" 通配, gadget_bind 以 strcmp 择 UDC — 名字必须精确,
//   截断即拒绝。
static bool discover_udc(struct usb_raw_init* init) {
    DIR* d = opendir("/sys/class/udc");
    if (!d) return false;
    struct dirent* de;
    while ((de = readdir(d)) && de->d_name[0] == '.')
        ;
    std::string inst = de ? de->d_name : "";
    closedir(d);
    if (inst.empty() || inst.size() >= UDC_NAME_LENGTH_MAX) return false;

    char resolved[PATH_MAX];
    if (!realpath(("/sys/class/udc/" + inst).c_str(), resolved)) return false;
    std::string p = resolved;                         // .../<pdev>/udc/<inst>
    size_t slash = p.rfind('/');
    if (slash == std::string::npos) return false;
    p.resize(slash);                                  // .../<pdev>/udc
    slash = p.rfind('/');
    if (slash == std::string::npos) return false;
    p.resize(slash);                                  // .../<pdev>
    p += "/driver";
    char link[PATH_MAX];
    ssize_t n = readlink(p.c_str(), link, sizeof(link) - 1);
    if (n < 0) return false;
    link[n] = '\0';
    const char* drv = strrchr(link, '/');
    drv = drv ? drv + 1 : link;
    if (strlen(drv) >= UDC_NAME_LENGTH_MAX) return false;

    memset(init->driver_name, 0, UDC_NAME_LENGTH_MAX);
    memset(init->device_name, 0, UDC_NAME_LENGTH_MAX);
    memcpy(init->driver_name, drv, strlen(drv));
    memcpy(init->device_name, inst.c_str(), inst.size());
    return true;
}

// ---- 端点 ioctl 的唤醒信号 -------------------------------------------------
// 内核侧端点与 ep0 的阻塞等待都是 interruptible 的 (raw_gadget 的
//   wait_for_completion_interruptible / down_interruptible), 被中断时驱动返回
//   -EINTR — 于是 "空操作处理器 + pthread_kill" 就是把阻塞在 ioctl 里的线程叫
//   回来的手段: 在途 ioctl 持有文件引用, 只 close 无法驱动 raw_release
//   (release 被推迟到在途 ioctl 返回之后), 停机必须把线程一一叫回来。
static void wake_noop(int) {}
static void install_wake_handler() {
    static std::once_flag once;
    std::call_once(once, [] {
        struct sigaction sa{};
        sa.sa_handler = wake_noop;
        sigemptyset(&sa.sa_mask);
        sigaction(SIGUSR1, &sa, nullptr);
    });
}
static void wake_blocked_io(std::thread& t) {
    if (t.joinable()) pthread_kill(t.native_handle(), SIGUSR1);
}

// ---- ep0 应答手段 ----------------------------------------------------------
// IN 数据阶段: 按 wLength 截断发送 (主机先读 9B 配置头再读全长, 截断必须生效 —
//   USB 2.0 §9.4.3: 实发 = min(wLength, 描述符长度))。阻塞至状态阶段完成。
static bool ep0_send_data(int fd, const uint8_t* data, int len, int wLength) {
    RawIoBuf b;
    b.io()->ep = 0;
    b.io()->flags = 0;
    if (len > wLength) len = wLength;
    b.io()->length = len;
    if (len > 0) memcpy(b.payload(), data, len);
    return ioctl(fd, USB_RAW_IOCTL_EP0_WRITE, b.io()) >= 0;
}

// SETUP 应答路由 (方向记账见文件头): 截断与 OUT 收尾都收拢在这一处。
static bool ep0_reply(int fd, const usb_ctrlrequest& c, const uint8_t* data, int len) {
    if ((c.bRequestType & USB_DIR_IN) && c.wLength)
        return ep0_send_data(fd, data, len, c.wLength);
    RawIoBuf b;                                       // OUT / 纯状态阶段: 收尾, 数据无消费方
    b.io()->ep = 0;
    b.io()->flags = 0;
    b.io()->length = c.wLength < (int)USBRAW_DESC_MAX ? c.wLength : (int)USBRAW_DESC_MAX;
    return ioctl(fd, USB_RAW_IOCTL_EP0_READ, b.io()) >= 0;
}

// 字符串描述符: bLength/bDescriptorType 头 + UTF-16LE 正文 — 表内串均为 ASCII
//   (码点 <0x80 与 UTF-16LE 一一对应); index 0 = LANGID 包 (0x0409 美式英语,
//   设备唯一语言); 表外 index → STALL (规范允许)。
static bool ep0_send_string(UsbRawSession& s, int index, int wLength) {
    if (index == 0) {
        static const uint8_t langid[4] = { 4, USB_DT_STRING, 0x09, 0x04 };
        return ep0_send_data(s.fd, langid, 4, wLength);
    }
    for (int i = 0; i < s.dev->string_count; ++i) {
        if (s.dev->strings[i].index != index) continue;
        const char* u = s.dev->strings[i].utf8;
        int n = (int)strlen(u);
        if (2 + 2 * n > (int)USBRAW_PKT_MAX) return false;   // 超 ep0 单包: 拒绝而非截断正文
        uint8_t buf[USBRAW_PKT_MAX];
        buf[0] = (uint8_t)(2 + 2 * n);
        buf[1] = USB_DT_STRING;
        for (int k = 0; k < n; ++k) { buf[2 + 2 * k] = (uint8_t)u[k]; buf[3 + 2 * k] = 0; }
        return ep0_send_data(s.fd, buf, 2 + 2 * n, wLength);
    }
    return false;
}

// GET_DESCRIPTOR 描述符字节源: 设备 / 限定符 / 配置 (含 other-speed 的类型字节
//   翻转, USB 2.0 §9.6.4) / 报告描述符 (类描述符经标准 GET_DESCRIPTOR 取,
//   HID 1.11 §7.1)。返回字节数; 不认识的类型与设备未定义的描述符返回 -1
//   (交 STALL)。
static int desc_bytes(const UsbRawDeviceDef& d, const usb_ctrlrequest& c, uint8_t* out) {
    switch (c.wValue >> 8) {
    case USB_DT_DEVICE:
        memcpy(out, &d.device, USB_DT_DEVICE_SIZE);
        return USB_DT_DEVICE_SIZE;
    case USB_DT_DEVICE_QUALIFIER:
        if (!d.qualifier) return -1;                  // 仅全速设备: 无限定符, 规范答复是 STALL
        memcpy(out, d.qualifier, sizeof(usb_qualifier_descriptor));
        return (int)sizeof(usb_qualifier_descriptor);
    case USB_DT_CONFIG:
    case USB_DT_OTHER_SPEED_CONFIG:
        memcpy(out, d.config, d.config_len);
        if ((c.wValue >> 8) == USB_DT_OTHER_SPEED_CONFIG)
            out[1] = USB_DT_OTHER_SPEED_CONFIG;           // 同一配置节, 仅类型字节按规范翻转
        return d.config_len;
    case HID_DT_REPORT:
        if (!d.report_desc) return -1;                // 非 HID 设备无报告描述符
        memcpy(out, d.report_desc, d.report_desc_len);
        return d.report_desc_len;
    default:
        return -1;
    }
}

// ---- ep0 标准请求表 --------------------------------------------------------
// 返回 false = 表未接 (交设备特有请求钩子 / STALL)。SET_CONFIGURATION 在控制
//   循环特判 (状态阶段之后才 CONFIGURE, 见 set_configuration)。
static bool standard_request(UsbRawSession& s, const usb_ctrlrequest& c) {
    uint8_t buf[USBRAW_DESC_MAX];
    int len = -1;
    switch (c.bRequest) {
    case USB_REQ_GET_DESCRIPTOR:
        if ((c.wValue >> 8) == USB_DT_STRING)
            return ep0_send_string(s, c.wValue & 0xff, c.wLength);
        len = desc_bytes(*s.dev, c, buf);
        break;
    case USB_REQ_GET_STATUS:
        buf[0] = 0; buf[1] = 0;                           // 设备=总线供电无远程唤醒 / 接口=0 / 端点=无 HALT
        len = 2;
        break;
    case USB_REQ_GET_CONFIGURATION:
        { std::lock_guard<std::mutex> lk(s.mtx); buf[0] = s.cfg_value; }
        len = 1;
        break;
    case USB_REQ_GET_INTERFACE:
        buf[0] = 0;                                       // bAlternateSetting = 0 (唯一接口设置)
        len = 1;
        break;
    case USB_REQ_SET_ADDRESS:                             // tegra-xudc 硬件自答; 再收到也零长度收尾
        len = 0;
        break;
    case USB_REQ_SET_INTERFACE:                           // 唯一接口设置, 零长度收尾
        len = 0;
        break;
    case USB_REQ_CLEAR_FEATURE:
    case USB_REQ_SET_FEATURE:
        if ((c.wValue & 0xff) == USB_ENDPOINT_HALT
            && (c.wIndex & 0xff) == s.dev->ep_in.bEndpointAddress) {
            int h;
            { std::lock_guard<std::mutex> lk(s.mtx); h = s.ep_enabled ? s.ep_handle : -1; }
            if (h >= 0)
                ioctl(s.fd, c.bRequest == USB_REQ_CLEAR_FEATURE
                              ? USB_RAW_IOCTL_EP_CLEAR_HALT : USB_RAW_IOCTL_EP_SET_HALT, h);
        }
        len = 0;
        break;
    default:
        return false;
    }
    return ep0_reply(s.fd, c, buf, len);
}

// HID 类请求表 (bmRequestType=0x21/0xA1, 由 boot 接口声明引发): SET_IDLE 被
//   STALL 会令 Windows 的 HID 类驱动初始化失败 (5s 后 RESET 重枚举, 循环至
//   主机放弃轮询) — SET_IDLE/SET_REPORT 因此以零长度成功收尾。
static bool hid_class_request(UsbRawSession& s, const usb_ctrlrequest& c) {
    uint8_t buf[2];
    int len;
    switch (c.bRequest) {
    case 0x01: buf[0] = 0; len = 1; break;               // GET_IDLE: idle rate 0
    case 0x03: buf[0] = 1; len = 1; break;               // GET_PROTOCOL: report 协议
    case 0x09: case 0x0A: case 0x0B:                     // SET_REPORT / SET_IDLE / SET_PROTOCOL
        len = 0;                                         //   参数与数据不消费, 正常收尾
        break;
    default:
        return false;
    }
    return ep0_reply(s.fd, c, buf, len);
}

// SET_CONFIGURATION: 值 = bConfigurationValue (=1) 时: 使能中断端点 (未使能时)
//   → 状态阶段 (阻塞至完成) → CONFIGURE (gadget 进入 configured 态) → 置
//   configured 唤醒发送线程 — 状态阶段完成后才宣告配置的次序。值 = 0: 去配置。
//   其余值: 设备只定义唯一配置, STALL。
static void set_configuration(UsbRawSession& s, const usb_ctrlrequest& c) {
    const uint8_t val = c.wValue & 0xff;
    const uint8_t cfg_val = s.dev->config[5];             // 配置节 bConfigurationValue
    if (val != 0 && val != cfg_val) { usbraw_ep0_stall(s); return; }
    if (val == 0) {
        { std::lock_guard<std::mutex> lk(s.mtx); s.cfg_value = 0; s.configured = false; }
        ep0_reply(s.fd, c, nullptr, 0);
        return;
    }
    int enabled = 0;
    {
        std::lock_guard<std::mutex> lk(s.mtx);
        if (!s.ep_enabled) {
            enabled = ioctl(s.fd, USB_RAW_IOCTL_EP_ENABLE, &s.dev->ep_in);
            if (enabled >= 0) { s.ep_handle = enabled; s.ep_enabled = true; }
        } else enabled = s.ep_handle;
    }
    if (enabled < 0) {
        if (s.stopping) return;                           // 停机唤醒 (EINTR) 不是端点故障
        usbraw_ep0_stall(s);                              // 配置无法成立: 正式拒绝该请求
        std::cerr << "❌ 中断 IN 端点使能失败: " << strerror(errno) << "\n";
        global_running = false;
        return;
    }
    ep0_reply(s.fd, c, nullptr, 0);                       // 状态阶段 (阻塞至完成)
    if (ioctl(s.fd, USB_RAW_IOCTL_CONFIGURE, 0) < 0) {
        std::cerr << "⚠ CONFIGURE 失败: " << strerror(errno) << "\n";
        return;
    }
    { std::lock_guard<std::mutex> lk(s.mtx); s.cfg_value = val; s.configured = true; }
    s.cv.notify_all();
}

// RESET: 中断端点随重枚举失效 → 使能的一端 DISABLE; SET_CONFIGURATION 到来时
//   再 ENABLE (见 set_configuration) — 端点生命周期与官方 raw-gadget 示例一致:
//   在未配置状态 enable 的端点, tegra-xudc 不会为其服务 IN token (主机 IN 轮询
//   永不完成, 数据请求排队无消费)。
static void on_reset(UsbRawSession& s) {
    std::lock_guard<std::mutex> lk(s.mtx);
    if (s.ep_enabled && ioctl(s.fd, USB_RAW_IOCTL_EP_DISABLE, s.ep_handle) == 0)
        s.ep_enabled = false;
    s.cfg_value = 0;
    s.configured = false;
}

// ---- 线程 ------------------------------------------------------------------
// 控制线程: EVENT_FETCH 阻塞取事件 → 分派; 唯一从 /dev/raw-gadget 取事件的线程。
static void ctrl_loop(UsbRawSession* s) {
    RawEventBuf b;
    while (true) {
        b.ev()->type = 0;
        b.ev()->length = sizeof(usb_ctrlrequest);     // 缓冲容量; 内核按 min(容量, 事件长) 回拷
        int rv = ioctl(s->fd, USB_RAW_IOCTL_EVENT_FETCH, b.ev());
        if (s->stopping) return;                      // stop() 已置位: 不再触碰会话 fd
        if (rv < 0) {
            if (errno == EINTR) continue;             // 信号打断: 无事件, 重取
            std::cerr << "❌ raw_gadget 事件线程退出: " << strerror(errno) << "\n";
            global_running = false;
            return;
        }
        switch (b.ev()->type) {
        case USB_RAW_EVENT_RESET:                            // 重枚举: 端点失效, 待 SET_CONFIGURATION 重建
            on_reset(*s);
            break;
        case USB_RAW_EVENT_CONTROL: {
            const usb_ctrlrequest& c = *reinterpret_cast<const usb_ctrlrequest*>(b.ev()->data);
            const uint8_t type = c.bRequestType & USB_TYPE_MASK;
            bool answered = false;
            if (type == USB_TYPE_STANDARD) {
                if (c.bRequest == USB_REQ_SET_CONFIGURATION) {
                    set_configuration(*s, c);         // 特判: 状态阶段后才 CONFIGURE
                    answered = true;
                } else answered = standard_request(*s, c);
            } else if (type == USB_TYPE_CLASS) {
                // 类请求路由 (约定见 usbraw.h): 定义了钩子的设备整个类段归钩子,
                //   未定义钩子才走内建 HID 类请求表。
                answered = s->dev->vendor_request ? s->dev->vendor_request(*s, c)
                                                  : hid_class_request(*s, c);
            } else if (s->dev->vendor_request) {      // vendor 段: 仅设了钩子才问
                answered = s->dev->vendor_request(*s, c);
            }
            // 未应答一律 STALL — 拒绝即拒得正式, **不可静默吞掉**: 内核 raw_gadget
            //   在 ep0 仍有在途阶段时对下一个 SETUP 报 -EBUSY, 吞掉一次就让该会话
            //   后续全部控制传输 (含重枚举) 失败。
            if (!answered) usbraw_ep0_stall(*s);
            break;
        }
        case USB_RAW_EVENT_DISCONNECT:
        case USB_RAW_EVENT_SUSPEND:
            { std::lock_guard<std::mutex> lk(s->mtx); s->configured = false; }
            break;
        case USB_RAW_EVENT_RESUME:                        // 挂起期端点与配置有效, 恢复即恢复发送
            { std::lock_guard<std::mutex> lk(s->mtx);
              if (s->cfg_value == s->dev->config[5]) s->configured = true; }
            s->cv.notify_all();
            break;
        default:
            break;
        }
    }
}

// 发送线程: 等 configured+新数据 → 取最新报告槽 → EP_WRITE (阻塞至主机取走) →
//   成功后立刻取最新再发。EP_WRITE 长度 = 提交长度: 短包即包边界 (提交长度由
//   生产者给定, 不取端点 wMaxPacketSize — 报告短于单包时按 wMaxPacketSize 发
//   会把报告补齐成整包)。一次提交 = 一次 EP_WRITE = 一次主机 IN 传输, 因此
//   报告率 = min(拍率, 主机服务率), 与提交率之间没有倍频通路。
//   失败 (链路断/重枚举) → 清 configured 回等待; 重配置后仍连续失败满 10 次
//   = 链路不可用, 停机 (与 evdev 读取侧同一持续硬错判据)。
static void send_loop(UsbRawSession* s) {
    RawIoBuf b;
    b.io()->flags = 0;
    int errs = 0;
    while (true) {
        {
            std::unique_lock<std::mutex> lk(s->mtx);
            s->cv.wait(lk, [&] { return s->stopping.load() || (s->configured && s->slot_fresh); });
            if (s->stopping) return;
            b.io()->ep = s->ep_handle;
            b.io()->length = s->slot_len;
            memcpy(b.payload(), s->slot, s->slot_len);
            s->slot_fresh = false;
        }
        if (ioctl(s->fd, USB_RAW_IOCTL_EP_WRITE, b.io()) >= 0) {
            errs = 0;
            continue;
        }
        const int e = errno;
        if (s->stopping) return;
        if (e == EINTR) continue;                         // 仅停机/重枚举唤醒路径会到这
        { std::lock_guard<std::mutex> lk(s->mtx); s->configured = false; }
        if (++errs > 10) {
            std::cerr << "❌ 中断 IN 发送持续失败 (" << strerror(e) << "), 停机\n";
            global_running = false;
            return;
        }
    }
}

// ---- 生命周期 --------------------------------------------------------------
bool usbraw_start(UsbRawSession& s, const UsbRawDeviceDef& dev) {
    if (dev.config_len > USBRAW_DESC_MAX || dev.report_desc_len > USBRAW_DESC_MAX) {
        std::cerr << "❌ 设备定义描述符超应答缓冲上限 (" << USBRAW_DESC_MAX << "B)\n";
        return false;
    }
    if (dev.ep_in.wMaxPacketSize == 0 || dev.ep_in.wMaxPacketSize > USBRAW_PKT_MAX) {
        std::cerr << "❌ 设备定义端点包长非法 (须 1.." << USBRAW_PKT_MAX << ")\n";
        return false;
    }
    install_wake_handler();
    s.dev = &dev;
    struct usb_raw_init init{};
    if (!discover_udc(&init)) {
        std::cerr << "❌ UDC 名字发现失败 (/sys/class/udc 无实例)\n";
        return false;
    }
    init.speed = (uint8_t)dev.speed;      // 端点 bInterval 的时间单位随之确定 (见 UsbRawDeviceDef)
    s.fd = open("/dev/raw-gadget", O_RDWR);
    if (s.fd < 0) {
        std::cerr << "❌ 无法打开 /dev/raw-gadget (" << strerror(errno) << ")\n"
                  << "   模块缺失/未载 → 本机内核须先具备 raw_gadget (modprobe;"
                  << " 缺失时按内核文档自行构建)\n"
                  << "   权限不足       → sudo bash scripts/setup_mouse.sh\n";
        return false;
    }
    if (ioctl(s.fd, USB_RAW_IOCTL_INIT, &init) < 0) {
        std::cerr << "❌ raw_gadget INIT 失败: " << strerror(errno) << "\n";
        close(s.fd); s.fd = -1;
        return false;
    }
    if (ioctl(s.fd, USB_RAW_IOCTL_RUN, 0) < 0) {
        const int e = errno;
        std::cerr << "❌ raw_gadget RUN 失败: " << strerror(e) << "\n";
        if (e == EBUSY)
            std::cerr << "   UDC 被占: 先 sudo bash scripts/setup_mouse.sh 腾空 (解绑遗留 gadget, 停占用进程)\n";
        close(s.fd); s.fd = -1;
        return false;
    }
    // 请求电流 = 配置节 bMaxPower 声明值 (uapi 单位 2mA, 与 bMaxPower 同单位;
    //   内核侧 usb_gadget_vbus_draw(2 × value) mA。传 mA 会请求 2 倍), 须在
    //   RUN 之后 (驱动要求 RUNNING 态)
    const uint32_t vbus_2ma = (uint32_t)dev.config[8];
    if (ioctl(s.fd, USB_RAW_IOCTL_VBUS_DRAW, vbus_2ma) < 0)
        std::cerr << "⚠ VBUS_DRAW 失败: " << strerror(errno) << "\n";

    s.ctrl_th = std::thread(ctrl_loop, &s);
    s.send_th = std::thread(send_loop, &s);
    std::cout << "✅ raw_gadget 会话: UDC " << init.device_name
              << " (driver " << init.driver_name << ", "
              << (dev.speed == USB_SPEED_FULL ? "full" : dev.speed == USB_SPEED_HIGH ? "high" : "?")
              << "-speed, " << (int)vbus_2ma * 2 << "mA)\n";
    return true;
}

void usbraw_stop(UsbRawSession& s) {
    {
        std::lock_guard<std::mutex> lk(s.mtx);
        if (s.stopping.exchange(true)) return;
    }
    s.cv.notify_all();
    // 在途 ioctl 持有文件引用, close 无法驱动 release (见 "端点 ioctl 的唤醒信号"):
    //   两个线程各自阻塞在自己的 ioctl 上, 一一叫回来再 close。
    wake_blocked_io(s.ctrl_th);
    wake_blocked_io(s.send_th);
    if (s.fd >= 0) close(s.fd);                           // close 即解绑 UDC (见文件头)
    s.fd = -1;
    if (s.ctrl_th.joinable()) s.ctrl_th.join();
    if (s.send_th.joinable()) s.send_th.join();
}

void usbraw_submit(UsbRawSession& s, const uint8_t* rpt, uint16_t len) {
    if (!s.dev || len == 0 || len > USBRAW_PKT_MAX
        || len > s.dev->ep_in.wMaxPacketSize) return;     // 超单包的报告不是一次传输
    {
        std::lock_guard<std::mutex> lk(s.mtx);
        memcpy(s.slot, rpt, len);
        s.slot_len = len;
        s.slot_fresh = true;
    }
    s.cv.notify_one();
}

// ---- 设备特有请求钩子的应答手段 --------------------------------------------
bool usbraw_ep0_write(UsbRawSession& s, const void* data, int len, int wLength) {
    if (len > wLength || len > (int)USBRAW_PKT_MAX) return false;
    return ep0_send_data(s.fd, (const uint8_t*)data, len, wLength);
}
bool usbraw_ep0_read(UsbRawSession& s, void* data, int max_len) {
    if (max_len > (int)USBRAW_PKT_MAX) return false;
    RawIoBuf b;
    b.io()->ep = 0;
    b.io()->flags = 0;
    b.io()->length = max_len;
    int rv = ioctl(s.fd, USB_RAW_IOCTL_EP0_READ, b.io());
    if (rv < 0) return false;
    if (rv > 0) memcpy(data, b.payload(), rv);
    return true;
}
void usbraw_ep0_stall(UsbRawSession& s) {
    ioctl(s.fd, USB_RAW_IOCTL_EP0_STALL, 0);
}
