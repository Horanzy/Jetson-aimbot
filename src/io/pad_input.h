// ============================================================================
//  pad_input.h — 物理手柄输入 (pad 输出模式; 与 hid 鼠标模式互斥): evdev 手柄
//    节点的查找与读取线程, 产出 Xbox 布局的逻辑手柄态。能力 (轴族 / 量程 /
//    按键 / dpad 形态) 一律按设备位图与 absinfo 实测判定, 不写死; 掉线或未插
//    = 清键位 + 1s 周期重扫重开, 不阻塞启动。无符号量程按"均匀步长 + 溢出由
//    端点吸收"的规则展开到 ±32767 (8 位家族查编译期表, 规则与推导见
//    pad_unsigned_code / PAD_AXIS8_TABLE), 已居中的器件原值直通。
// ============================================================================

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <mutex>
#include <string>

// 逻辑按钮位表 (Xbox 布局; 本仓库自定义, 与内核 BTN_* 解耦, 打印为 16 进制)
enum PadBtn : uint16_t {
    PADBTN_A     = 1u << 0,  PADBTN_B     = 1u << 1,  PADBTN_X  = 1u << 2,
    PADBTN_Y     = 1u << 3,  PADBTN_LB    = 1u << 4,  PADBTN_RB = 1u << 5,
    PADBTN_BACK  = 1u << 6,  PADBTN_START = 1u << 7,  PADBTN_GUIDE = 1u << 8,
    PADBTN_L3    = 1u << 9,  PADBTN_R3    = 1u << 10,
    PADBTN_DPAD_UP    = 1u << 11, PADBTN_DPAD_DOWN = 1u << 12,
    PADBTN_DPAD_LEFT  = 1u << 13, PADBTN_DPAD_RIGHT = 1u << 14,
    // 触摸板按下 (PS5 语义): 物理手柄没有这个键 — 由"分享/上传"一类键代位
    //   (实测 G7 Pro 的分享键报在**第二接口**: 摇杆节点无事件, 键盘节点报
    //   KEY_SYSRQ = 99, 见 pad_extra_key_bit)。XInput 线格式 (pad 模式) 没有这个
    //   位, 该键在 360 手柄的报文里无落点, 只在未来的 P5G 后端有意义。
    PADBTN_TOUCH = 1u << 15,
};

// 摇杆逻辑满偏 (XInput 惯例 ±32767): 任意实测量程的设备都展开/直通到该域
constexpr int PAD_AXIS_MAX = 32767;

// -P 缺省匹配关键字: 空 = 任意 *-event-joystick 节点 (与 hid 的 DEFAULT_KEYWORD 同约定)
constexpr const char* DEFAULT_PAD_KEYWORD = "";

// P5 General 加密狗 (0x2B81/0x0101, 未来 p5g 输出模式的主机侧签名外设) 的输入
//   报告在内核 hid 侧被映射成一个摇杆 evdev 节点 — 它是认证外设, 不是人手通道,
//   从手柄候选中显式排除 (判据是节点实测的 VID:PID: by-id 名字不含它们)。
constexpr uint16_t P5G_DONGLE_VID = 0x2B81;
constexpr uint16_t P5G_DONGLE_PID = 0x0101;

// 逻辑手柄态: 摇杆 int16 (上/左为负), 扳机 0–255 模拟量 (实测量程线性直映,
//   无阈值), 按钮 1:1
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

// 整数除法四舍五入 (半值远离零): 量化误差在轴的上下两半同向对称, 且端点值严格
//   贴合 (整数运算, 与浮点/--use_fast_math 无关)
inline int pad_round_div(long num, int den) {
    return (int)(num >= 0 ? (num + den / 2) / den : -((-num + den / 2) / den));
}

// ---- 无符号 2^n 量程的展开规则 (均匀步长 + 溢出由端点吸收) ----
// 物理理由: 8 位轴只有 255 个物理级, 把这 255 级**均匀铺满** ±32767 才谈得上
//   "完整体现 8 位精度"; 步长取 S = floor(32767/127) = 258 —— 让 127 个正半程
//   步长整体落在满偏内的最大整数 (126 个正级 × 258 = 32508 ≤ 32767)。对照:
//   S = 257 (= floor(32767/128)) 会把最高一级留在 32767 − 257×127 = 128 counts
//   ≈ 0.4% 满偏的欠偏处, 而游戏常按"满偏"判定冲刺/最大转速, 差一级就少一档;
//   S = 258 只差 1 count。代价是 128 个负级 × 258 = 33024 越过 int16 满偏,
//   由**端点数吸收** (clamp 到 −32767): 溢出只落在最深推的那一级, 而游戏本来
//   就会在深推一侧饱和。
//   构造: 中点码 c = mn + (mx−mn+1)/2 (8 位即 0x80 = 128, 即 HID 给 0..255 轴的
//   中立值) → 0; out[c+k] = min(+S·k, +32767) (k = 1..mx−c)、
//   out[c−k] = max(−S·k, −32767) (k = 1..c−mn)。于是整条映射单调不减、中点为
//   精确 0、最高一级为 +S(mx−c) = +32766 (8 位), 且只有最深一级被端点吸收。

// 规则本体: 无符号 [mn, mx] → 逻辑 ±32767, 步长即上式 (整数除法, 8 位给 258)。
//   8 位家族在运行期改走下面的查表 (同一规则在 256 个码值上的取值), 这里服务
//   其它无符号 2^n 量程。
constexpr int16_t pad_unsigned_code(int code, int mn, int mx) {
    if (mx <= mn) return 0;
    const int c = mn + (mx - mn + 1) / 2;                   // 中点码 (HID 约定)
    const int step = PAD_AXIS_MAX / (mx - c);                // 正半程步长 (整数)
    if (step <= 0) return 0;
    const int v = (code - c) * step;
    return (int16_t)(v < -PAD_AXIS_MAX ? -PAD_AXIS_MAX
                     : v > PAD_AXIS_MAX ? PAD_AXIS_MAX : v);
}

// 8 位家族的查表 (256 项, 编译期构造自上面的规则): 运行期一次索引, 不做算术。
//   表是常量数据, 单测用一张**独立写死的黄金表**逐值比对它 — 断言的期望值不来自
//   本文件的任何函数, 否则规则写错时表与断言会一起错。
constexpr std::array<int16_t, 256> make_pad_axis8_table() {
    std::array<int16_t, 256> t{};
    for (int i = 0; i < 256; ++i) t[i] = pad_unsigned_code(i, 0, 0xFF);
    return t;
}
inline constexpr std::array<int16_t, 256> PAD_AXIS8_TABLE = make_pad_axis8_table();

// 原始轴值 → 逻辑 int16: 量程已居中 (min<0) 的器件原值直通 (设备自己的 ±32767
//   域即逻辑域, 不查表); 无符号量程走上面那条规则 (8 位家族查表, 越界码值按端点
//   处理)。不施加设备声明的 flat/fuzz (静置残余偏转如实透传), 也不加人为死区
//   (游戏自己的死区管这件事)。
//   注: 实测 G7 Pro 的 Y/RZ 轴静止原始值是 124/125、126/128 —— 器件本身偏 3–4
//   counts (≈ 3% 满偏), 本库不补偿该偏置 (游戏死区吸收); 将来若要补, 取 absinfo
//   的 center 减一次即可 (一行), 不需要每台设备的中位捕获。
inline int16_t pad_axis_to_logical(int v, int mn, int mx) {
    if (mx <= mn) return 0;
    if (mn < 0) return (int16_t)(v < -PAD_AXIS_MAX ? -PAD_AXIS_MAX
                                  : v > PAD_AXIS_MAX ? PAD_AXIS_MAX : v);
    if (mn == 0 && mx == 0xFF)                                  // 8 位无符号家族
        return PAD_AXIS8_TABLE[std::clamp(v, 0, 0xFF)];
    return pad_unsigned_code(v, mn, mx);                        // 其它无符号量程
}

// 扳机行程 → 0–255 逻辑量 (实测 [min,max] 线性直映, 无阈值)
inline uint8_t pad_trig_to_logical(int v, int mn, int mx) {
    if (mx <= mn) return 0;
    const int n = pad_round_div((long)(v - mn) * 255, mx - mn);
    return (uint8_t)(n < 0 ? 0 : n > 255 ? 255 : n);
}

// pad 设备查找: -P 子串 → /dev/input/by-id 的 *-event-joystick 节点 (大小写
//   不敏感子串; 附属 kbd/mouse 接口天然被后缀排除); by-id 无命中时回落扫
//   /dev/input/event* 按设备名 + 摇杆能力匹配 (uinput 虚拟手柄与蓝牙手柄没有
//   by-id 节点)。绝对路径原样直通。kw 为空 = 任意。verbose 时打印多匹配/候选
//   列表 (启动诊断一次性; reader 的重试路径传 false 防刷屏)。
std::string find_pad_device(const std::string& kw, bool verbose=false);

// 读取线程: 打开 + EVIOCGRAB + 事件解析入 PadState; 读错/设备消失 = 清键位
//   (防卡键) + 1s 周期重扫重开 (含启动时未插; 线程自身不退出不停机)。
//   除摇杆节点外还按 **VID:PID 打开同一物理设备的所有兄弟 event 节点** (实测
//   G7 Pro = 摇杆 + 键盘 + 鼠标三个接口, 额外键走键盘接口 — 只读摇杆节点会漏掉
//   分享/上传这类键), 兄弟节点只消费 EV_KEY 并经 pad_extra_key_bit 翻译。
void pad_reader_thread(const std::string& kw, PadState& st);

// 兄弟节点翻译表 (兄弟接口上的 KEY_* → 逻辑位; 0 = 不关心): 只收引擎需要的那
//   几个键, 不做通用键盘映射 (手柄的键盘接口同时会报一堆无关键)。
uint16_t pad_extra_key_bit(uint16_t code);

// 逻辑态快照 (互斥)
PadLogical pad_input_snapshot(PadState& st);
