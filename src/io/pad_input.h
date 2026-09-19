// ============================================================================
//  pad_input.h — 物理手柄输入 (pad 模式, 与 hid 鼠标互斥): evdev 游戏手柄的
//    by-id/名字查找与读取线程, 产出 Xbox 布局的逻辑手柄态。能力以设备 absinfo/
//    key 位图实测为准 (轴族与量程不写死); 掉线/未插 = 清键位 + 1s 周期重试重开,
//    不阻塞启动, 翻转各记一次日志。
// ============================================================================

#pragma once

#include <cstdint>
#include <mutex>
#include <string>

// 逻辑按钮位表 (Xbox 布局, 本仓库自定义, 与内核 BTN_* 解耦; btns 打印为 16 进制)
enum PadBtn : uint16_t {
    PADBTN_A     = 1u << 0,  PADBTN_B     = 1u << 1,  PADBTN_X  = 1u << 2,
    PADBTN_Y     = 1u << 3,  PADBTN_LB    = 1u << 4,  PADBTN_RB = 1u << 5,
    PADBTN_BACK  = 1u << 6,  PADBTN_START = 1u << 7,  PADBTN_GUIDE = 1u << 8,
    PADBTN_L3    = 1u << 9,  PADBTN_R3    = 1u << 10,
    PADBTN_DPAD_UP    = 1u << 11, PADBTN_DPAD_DOWN = 1u << 12,
    PADBTN_DPAD_LEFT  = 1u << 13, PADBTN_DPAD_RIGHT = 1u << 14,
};

// 摇杆逻辑满偏 (XInput 惯例 ±32767; 任意实测量程的设备都展开/直通到该域)
constexpr int PAD_AXIS_MAX = 32767;

// -P 缺省匹配关键字: 空 = 任意 *-event-joystick 节点 (与 hid 的 DEFAULT_KEYWORD 同约定)
constexpr const char* DEFAULT_PAD_KEYWORD = "";

// 逻辑手柄态: 摇杆 int16 (上/左为负, 实测量程已居中者原值直通、无符号者按实测
//   中心线性展开, 全分辨率); 扳机 0–255 模拟量 (设备 absinfo 实测量程线性直映,
//   无阈值); 按钮 1:1
struct PadLogical {
    int16_t lx=0, ly=0, rx=0, ry=0;
    uint8_t lt=0, rt=0;
    uint16_t btns=0;
};

// 互斥保护的手柄共享态 (读取线程写, 控制拍读)
struct PadState {
    std::mutex mtx;
    PadLogical st{};
};

// pad 设备查找: -P 子串 → /dev/input/by-id 的 *-event-joystick 节点 (大小写
//   不敏感子串, 附属 kbd/mouse 接口天然被后缀排除); by-id 无命中时回落扫
//   /dev/input/event* 按设备名 + 摇杆能力匹配 (uinput 虚拟手柄与蓝牙手柄无
//   by-id 节点)。绝对路径原样直通。kw 为空 = 任意。verbose 时打印多匹配/可选
//   列表 (启动诊断一次性; reader 重试路径传 false 防刷屏)。
std::string find_pad_device(const std::string& kw, bool verbose=false);

// 读取线程: 打开 + EVIOCGRAB + 事件解析入 PadState; 读错/设备消失 = 清键位
//   (防卡键) + 1s 周期重扫重开 (含启动时未插, 线程自身不退出不停机)。
void pad_reader_thread(const std::string& kw, PadState& st);

// 逻辑态快照 (互斥)
PadLogical pad_input_snapshot(PadState& st);
