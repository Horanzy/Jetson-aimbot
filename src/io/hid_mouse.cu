// ============================================================================
//  hid_mouse.cu — hid_mouse.h 的实现: evdev 事件解析 (REL_*/BTN_* 累积),
//    /dev/input/by-id 设备名匹配, 独占读取循环 (读取侧持续硬错误停机);
//    USB 鼠标身份 (设备/配置/报告描述符) 与 9 字节 HID 报文组包 — 组包后经
//    overlay 回调把控制律 counts 合成进位移字节, 提交 raw_gadget 会话报告槽。
// ============================================================================

#include "io/hid_mouse.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <iostream>
#include <thread>

#include <fcntl.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

void extract_and_clear(MouseState& s, int16_t& x, int16_t& y,
                       int8_t& w, int8_t& hw, uint16_t& btns) {
    std::lock_guard<std::mutex> lk(s.mtx);
    x=(int16_t)std::clamp(s.rel_x,-32768,32767);
    y=(int16_t)std::clamp(s.rel_y,-32768,32767);
    w=(int8_t)std::clamp(s.rel_wheel,-128,127);
    hw=(int8_t)std::clamp(s.rel_hwheel,-128,127);
    btns=s.buttons;
    s.rel_x=s.rel_y=s.rel_wheel=s.rel_hwheel=0;
}
std::string find_mouse_device(const std::string& kw) {
    if (!kw.empty() && kw.front()=='/') return access(kw.c_str(),R_OK)==0?kw:"";
    std::string cmd="find "+std::string(DEV_SEARCH_PATH)
                   +" -name '*"+kw+"*-event-mouse' -print -quit 2>/dev/null";
    FILE* fp=popen(cmd.c_str(),"r"); if (!fp) return {};
    char buf[512]; std::string r;
    if (fgets(buf,sizeof(buf),fp)) { r=buf; if(!r.empty()&&r.back()=='\n') r.pop_back(); }
    pclose(fp); return r;
}
void reader_thread(const std::string& dev, MouseState& st) {
    int fd=open(dev.c_str(),O_RDONLY);
    if (fd<0) { std::cerr<<"无法打开鼠标\n"; global_running=false; return; }
    if (ioctl(fd,EVIOCGRAB,1)<0) std::cerr<<"警告: 无法独占\n";
    else std::cout<<"✅ 已独占: "<<dev<<"\n";
    struct pollfd pfd{}; pfd.fd=fd; pfd.events=POLLIN;
    struct input_event ev; int errs=0;
    while (global_running) {
        int pr=poll(&pfd,1,100);
        if (pr<0) { if(errno==EINTR)continue; global_running=false; break; }
        if (pr==0) continue;
        if (!(pfd.revents&POLLIN)) { std::cerr<<"鼠标断开\n"; global_running=false; break; }
        ssize_t n=read(fd,&ev,sizeof(ev));
        if (n==(ssize_t)sizeof(ev)) { errs=0;
            std::lock_guard<std::mutex> lk(st.mtx);
            if (ev.type==EV_REL) {
                if(ev.code==REL_X)st.rel_x+=ev.value;
                else if(ev.code==REL_Y)st.rel_y+=ev.value;
                else if(ev.code==REL_WHEEL)st.rel_wheel+=ev.value;
                else if(ev.code==REL_HWHEEL)st.rel_hwheel+=ev.value;
            } else if (ev.type==EV_KEY&&ev.code>=BTN_LEFT&&ev.code<=BTN_TASK) {
                int idx=ev.code-BTN_LEFT;
                if(ev.value)st.buttons|=(1<<idx); else st.buttons&=~(1<<idx); }
        } else if (n<0&&errno!=EINTR&&errno!=EAGAIN) {
            if(++errs>10){std::cerr<<"鼠标读取失败\n";global_running=false;break;} usleep(1000); }
    }
    close(fd);
}

// ========================= USB 鼠标身份 =========================
// VID/PID/bcdDevice = Linux Foundation 通用 gadget 身份 — 验收主机 Windows 侧
//   已按此身份装好驱动, 身份不变; 引导协议接口 (subclass=1/protocol=2) 与
//   100mA 上限 (bMaxPower=50 单位 2mA; VBUS 请求由该字节导出, 见 usbraw) 同为主机侧既有事实。
//   aarch64 小端, 描述符 __le16 字段直写数值。

// 报告描述符 — 全库唯一事实源 (HID 类描述符的 wDescriptorLength 由本数组长度
//   编译期导出): Report ID 0x02 + 16 键 (字节 1–2) + X/Y s16 相对位移 (3–6)
//   + 滚轮 s8 (7) + AC Pan s8 (8), 与 9 字节报文逐字段对应。
static constexpr uint8_t MOUSE_REPORT_DESC[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x02,       // Usage (Mouse)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x02,       //   Report ID (2)
    0x09, 0x01,       //   Usage (Pointer)
    0xA1, 0x00,       //   Collection (Physical)
    0x05, 0x09,       //     Usage Page (Button)
    0x19, 0x01,       //     Usage Minimum (Button 1)
    0x29, 0x10,       //     Usage Maximum (Button 16)
    0x15, 0x00,       //     Logical Minimum (0)
    0x25, 0x01,       //     Logical Maximum (1)
    0x75, 0x01,       //     Report Size (1)
    0x95, 0x10,       //     Report Count (16)
    0x81, 0x02,       //     Input (Data,Var,Abs) — 按钮 u16
    0x05, 0x01,       //     Usage Page (Generic Desktop)
    0x09, 0x30,       //     Usage (X)
    0x09, 0x31,       //     Usage (Y)
    0x16, 0x00, 0x80, //     Logical Minimum (-32768)
    0x26, 0xFF, 0x7F, //     Logical Maximum (32767)
    0x75, 0x10,       //     Report Size (16)
    0x95, 0x02,       //     Report Count (2)
    0x81, 0x06,       //     Input (Data,Var,Rel) — X/Y s16LE
    0x09, 0x38,       //     Usage (Wheel)
    0x15, 0x81,       //     Logical Minimum (-127)
    0x25, 0x7F,       //     Logical Maximum (127)
    0x75, 0x08,       //     Report Size (8)
    0x95, 0x01,       //     Report Count (1)
    0x81, 0x06,       //     Input (Data,Var,Rel) — 滚轮 s8
    0x05, 0x0C,       //     Usage Page (Consumer)
    0x0A, 0x38, 0x02, //     Usage (AC Pan)
    0x15, 0x81,       //     Logical Minimum (-127)
    0x25, 0x7F,       //     Logical Maximum (127)
    0x75, 0x08,       //     Report Size (8)
    0x95, 0x01,       //     Report Count (1)
    0x81, 0x06,       //     Input (Data,Var,Rel) — 水平滚轮 s8
    0xC0,             //   End Collection (Physical)
    0xC0,             // End Collection (Application)
};
constexpr uint16_t REPORT_DESC_LEN = sizeof(MOUSE_REPORT_DESC);

// 配置节 34B = config 9 + interface 9 + HID 类描述符 9 + endpoint 7; 端点
//   bInterval=1: 高速中断端点单位 125µs (USB 2.0 §9.6.6) → 主机服务能力 8kHz
//   ≥ 拍率 DEFAULT_FREQ=1000 (报告周期 1ms), 拍率改 500Hz 亦无需变此值。
static constexpr uint8_t MOUSE_CONFIG[34] = {
    0x09, USB_DT_CONFIG, (uint8_t)sizeof(MOUSE_CONFIG), 0x00,   // wTotalLength = 全长
    0x01,             //   bNumInterfaces = 1
    0x01,             //   bConfigurationValue = 1
    0x00,             //   iConfiguration (无配置串)
    0x80,             //   bmAttributes: 总线供电, 无远程唤醒
    0x32,             //   bMaxPower = 50 (单位 2mA) = 100mA (VBUS 请求由本字节导出)
    0x09, USB_DT_INTERFACE,
    0x00,             //   bInterfaceNumber = 0
    0x00,             //   bAlternateSetting = 0
    0x01,             //   bNumEndpoints = 1
    0x03,             //   bInterfaceClass = HID
    0x01,             //   bInterfaceSubClass = 1 (boot)
    0x02,             //   bInterfaceProtocol = 2 (mouse)
    0x00,             //   iInterface (无接口串)
    0x09, HID_DT_HID,
    0x11, 0x01,       //   bcdHID = 1.11 (HID 规范版本)
    0x00,             //   bCountryCode = 无本地化
    0x01,             //   bNumDescriptors = 1
    HID_DT_REPORT,    //   类描述符: 报告描述符
    (uint8_t)(REPORT_DESC_LEN & 0xFF),                        //   wDescriptorLength
    (uint8_t)(REPORT_DESC_LEN >> 8),                          //     = 报告描述符数组长度 (编译期导出)
    0x07, USB_DT_ENDPOINT,
    0x81,             //   bEndpointAddress = IN1
    0x03,             //   bmAttributes = interrupt
    (uint8_t)HID_REPORT_LEN, 0x00,                            //   wMaxPacketSize = HID_REPORT_LEN
    0x01,             //   bInterval = 1 (见上)
};
static_assert(MOUSE_CONFIG[2] == sizeof(MOUSE_CONFIG) && MOUSE_CONFIG[3] == 0,
              "配置节 wTotalLength 必须等于数组全长");

static const UsbRawStringDef MOUSE_STRINGS[] = {
    { 1, "Generic" }, { 2, "USB Mouse" }, { 3, "000000000001" },
};

// 设备限定符: 高速设备必答 (USB 2.0 §9.6.2) — bMaxPacketSize0 与设备描述符一致
static const usb_qualifier_descriptor MOUSE_QUALIFIER = {
    (uint8_t)sizeof(usb_qualifier_descriptor), USB_DT_DEVICE_QUALIFIER,
    0x0200, 0, 0, 0, 64, 1, 0,
};

static UsbRawDeviceDef build_mouse_usb() {
    UsbRawDeviceDef d{};
    usb_device_descriptor& dev = d.device;
    dev.bLength          = USB_DT_DEVICE_SIZE;
    dev.bDescriptorType  = USB_DT_DEVICE;
    dev.bcdUSB           = 0x0200;             // USB 2.0
    dev.bDeviceClass     = 0;                  // 类在接口级定义 (HID 设备惯例)
    dev.bDeviceSubClass  = 0;
    dev.bDeviceProtocol  = 0;
    dev.bMaxPacketSize0  = 64;                // = USBRAW_PKT_MAX (ep0 应答单包即完整)
    dev.idVendor         = 0x1d6b;            // Linux Foundation
    dev.idProduct        = 0x0104;            // 通用 gadget 身份 (见上)
    dev.bcdDevice        = 0x0300;
    dev.iManufacturer    = 1;
    dev.iProduct         = 2;
    dev.iSerialNumber    = 3;
    dev.bNumConfigurations = 1;

    d.qualifier          = &MOUSE_QUALIFIER;

    usb_endpoint_descriptor& ep = d.ep_in;
    ep.bLength           = USB_DT_ENDPOINT_SIZE;
    ep.bDescriptorType   = USB_DT_ENDPOINT;
    ep.bEndpointAddress  = 0x81;              // IN1
    ep.bmAttributes      = USB_ENDPOINT_XFER_INT;
    ep.wMaxPacketSize    = (uint16_t)HID_REPORT_LEN;    // 报文长度编译期绑定
    ep.bInterval         = 1;                 // 见配置节 bInterval 注

    d.config            = MOUSE_CONFIG;
    d.config_len        = sizeof(MOUSE_CONFIG);
    d.report_desc       = MOUSE_REPORT_DESC;
    d.report_desc_len   = REPORT_DESC_LEN;
    d.speed             = USB_SPEED_HIGH;     // 端点 bInterval=1 按高速微帧解释 (见配置节注)
    d.has_ep_out        = false;              // 鼠标只有中断 IN 端点
    d.strings           = MOUSE_STRINGS;
    d.string_count      = (uint8_t)(sizeof(MOUSE_STRINGS) / sizeof(MOUSE_STRINGS[0]));
    d.vendor_request    = nullptr;            // 鼠标无设备特有请求 → 一切非标准请求 STALL
    d.rate_trace        = false;              // HID 鼠标不发速率行 (报告率非其验收项)
    return d;
}

void hid_report_submit(UsbRawSession& usb, int16_t rx, int16_t ry, int8_t w, int8_t hw, uint16_t btns,
                       const std::function<void(std::array<uint8_t,HID_REPORT_LEN>&,int16_t,int16_t)>& overlay) {
    std::array<uint8_t,HID_REPORT_LEN> rpt{};
    rpt[0]=0x02; rpt[1]=btns&0xFF; rpt[2]=btns>>8;
    rpt[3]=rx&0xFF; rpt[4]=rx>>8; rpt[5]=ry&0xFF; rpt[6]=ry>>8;
    rpt[7]=w; rpt[8]=hw;
    if (overlay) overlay(rpt,rx,ry);
    usbraw_submit(usb, rpt.data(), HID_REPORT_LEN);
}

static std::thread g_reader;

bool hid_mouse_start(MouseState& st, UsbRawSession& usb) {
    std::string real_dev=find_mouse_device(DEFAULT_KEYWORD);
    if (real_dev.empty()) { std::cerr<<"❌ 未找到鼠标\n"; return false; }
    std::cout<<"✅ 鼠标: "<<real_dev<<"\n";
    g_reader = std::thread(reader_thread, real_dev, std::ref(st));
    static const UsbRawDeviceDef usb_dev = build_mouse_usb();   // 会话持有指针: 进程级寿命
    if (!usbraw_start(usb, usb_dev)) {
        global_running = false;               // 让读取线程退出 (poll 100ms 粒度)
        if (g_reader.joinable()) g_reader.join();
        return false;
    }
    return true;
}

void hid_mouse_stop(UsbRawSession& usb) {
    usbraw_stop(usb);                         // close 即解绑 UDC
    if (g_reader.joinable()) g_reader.join(); // 读取线程经 global_running 退出
}
