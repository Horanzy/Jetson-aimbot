// ============================================================================
//  pad_xinput.h — XInput 有线 Xbox 360 手柄输出后端 (pad 模式的 USB 输出侧):
//    设备字节与 20 字节输入报告的线格式编码, 外加消费 pad 发布点的报告循环。
//    分层: 逻辑态 (PadLogical, io/pad_input.h) 与线格式 (20B 报告) 只在本文件
//    相接 — usbraw 不含任何手柄知识, 本文件不含 USB 会话知识, 后续换协议
//    (如 DualSense) 是新增一个同构文件, 不动会话层与合并层。
// ============================================================================

#pragma once

#include <cstddef>
#include <cstdint>

#include "io/pad_input.h"                // PadLogical / PadBtn
#include "io/usbraw.h"

// 输入报告长度: 线上 20B (报头 2 + 按键 2 + 扳机 2 + 摇杆 4×2 + 保留 6), 在端点
//   wMaxPacketSize=32 内; EP_WRITE 长度 = 本值 (短包即包边界)。
constexpr size_t PAD_XINPUT_REPORT_LEN = 20;

// 设备定义 (设备/配置/字符串/端点/速度) —— 会话启动与单测共用的同一份事实
const UsbRawDeviceDef& pad_xinput_usb_def();

// 逻辑态 → 20 字节报告: 显式 u8[20] 手工组装, 摇杆 int16 小端。
//   符号约定: 逻辑态 "上/左为负" (Xbox 布局的 evdev 口径), 线上 "上为正"
//   (XInput API 口径, 见 AGENTS.md 的 pad 段) → 两个 Y 轴取反写入, X 轴直通
//   (左为负号在两套口径下相同); 按键按线上位表映射, 保留位恒 0。
void pad_xinput_report(const PadLogical& st, uint8_t out[PAD_XINPUT_REPORT_LEN]);

// 输出后端: raw_gadget 会话 + 报告循环 (发布点有更新即组装提交; 发送节拍由主机
//   端点轮询决定, 见 usbraw 发送线程)。失败 (设备/模块/UDC 不可用) 返回 false。
bool pad_xinput_start();
void pad_xinput_stop();
