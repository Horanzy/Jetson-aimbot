// ============================================================================
//  hid_mouse.h — 真实鼠标输入与 USB 鼠标输出: evdev 设备发现与独占读取线程
//    (EVIOCGRAB), 累积位移的原子取出; USB 鼠标身份 (设备/配置/报告描述符,
//    报告描述符是报文格式的唯一事实源) 与 9 字节 HID 报文组装 — 组装后经
//    overlay 回调把控制拍的 counts 合成进位移字节, 提交进 raw_gadget 会话的
//    最新报告槽 (io/usbraw)。
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
// 鼠标设备选择 (kw = /dev/input/by-id 名字的大小写不敏感子串):
//   kw 以 '/' 开头 → 直接当设备路径; kw 空 → 任一 *-event-mouse 中字典序首个
//   (确定性: 目录读取顺序在不同启动间不保证一致); kw 非空而命中多于一个 →
//   报错并列出全部候选 (歧义不猜 — 也要能点名真鼠标, 手柄插上时它的辅助鼠标
//   接口同样匹配默认模式)。
std::string find_mouse_device(const std::string& kw);
void reader_thread(const std::string& dev, MouseState& st);

// 每拍组包 (9 字节 = Report ID + 按钮 u16 + X/Y s16LE + 滚轮 s8 + 水平滚轮 s8),
//   overlay 注入后提交报告槽 (非阻塞; 主机服务率跟不上时保留最新报告)。
void hid_report_submit(UsbRawSession& usb, int16_t rx, int16_t ry, int8_t w, int8_t hw,
                       uint16_t btns,
                       const std::function<void(std::array<uint8_t,HID_REPORT_LEN>&,int16_t,int16_t)>& overlay);

// 启动: 找真实鼠标 (keyword 见 find_mouse_device) → 起 evdev 读取线程 → 起
//   raw_gadget USB 会话 (鼠标身份内建); 失败返回 false, 可行动原因已打印。
//   stop: 停会话 (close 即解绑 UDC) 并 join 读取线程。
bool hid_mouse_start(MouseState& st, UsbRawSession& usb, const std::string& keyword);
void hid_mouse_stop(UsbRawSession& usb);
