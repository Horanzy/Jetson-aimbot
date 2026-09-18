// ============================================================================
//  hid_mouse.h — 真实鼠标输入与虚拟鼠标输出: evdev 设备发现与独占读取线程
//    (EVIOCGRAB), 累积位移的原子取出, 9 字节 HID 报文写 /dev/hidg0 — 写入前
//    经 overlay 回调把控制律 counts 合成进位移字节。
// ============================================================================

#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>

#include "core/state.h"

void extract_and_clear(MouseState& s, int16_t& x, int16_t& y,
                       int8_t& w, int8_t& hw, uint16_t& btns);
std::string find_mouse_device(const std::string& kw);
void reader_thread(const std::string& dev, MouseState& st);
void send_report(int fd, int16_t rx, int16_t ry, int8_t w, int8_t hw, uint16_t btns,
                 const std::function<void(std::array<uint8_t,HID_REPORT_LEN>&,int16_t,int16_t)>& overlay);
