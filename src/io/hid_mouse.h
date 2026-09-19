// ============================================================================
//  hid_mouse.h — 真实鼠标输入与 USB 鼠标输出: evdev 设备发现与独占读取线程
//    (EVIOCGRAB), 累积位移的原子取出; USB 鼠标身份 (设备/配置/报告描述符)
//    与 9 字节 HID 报文组装 — 写入前经 overlay 回调把控制律 counts 合成进
//    位移字节, 提交 raw_gadget 会话的最新报告槽。
// ============================================================================

#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>

#include "core/state.h"
#include "io/usbraw.h"

void extract_and_clear(MouseState& s, int16_t& x, int16_t& y,
                       int8_t& w, int8_t& hw, uint16_t& btns);
std::string find_mouse_device(const std::string& kw);
void reader_thread(const std::string& dev, MouseState& st);

// 每拍组包 (9 字节 = Report ID + 按钮 u16 + X/Y s16LE + 滚轮 s8 + 水平滚轮 s8),
//   overlay 注入后覆盖提交报告槽 (非阻塞; 主机服务率跟不上时保留最新报告)。
void hid_report_submit(UsbRawSession& usb, int16_t rx, int16_t ry, int8_t w, int8_t hw,
                       uint16_t btns,
                       const std::function<void(std::array<uint8_t,HID_REPORT_LEN>&,int16_t,int16_t)>& overlay);

// 启动: 找真实鼠标设备 → 起 evdev 读取线程 → 起 raw_gadget USB 会话 (鼠标身份
//   内建); 失败返回 false, 可行动原因已打印。stop: 停会话 (close 即解绑 UDC)
//   并 join 读取线程。
bool hid_mouse_start(MouseState& st, UsbRawSession& usb);
void hid_mouse_stop(UsbRawSession& usb);
