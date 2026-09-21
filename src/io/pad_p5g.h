// ============================================================================
//  pad_p5g.h — P5 General 输出后端 (-M p5g, PS5): 对 PS5 冒充 P5 General 加密狗
//    形态的 HID 手柄 (VID 0x2B81 / PID 0x0101, 全速, HID 报告描述符 165B), 同时
//    作为 USB 主机驱动插在本机口上的**真实加密狗** (同一 VID/PID, 经 /dev/hidraw):
//    PS5 下发的认证质询 (SET_REPORT Feature 0xF0) 原样转发给加密狗, 签名/nonce
//    (Feature 0xF1) 与签名状态 (Feature 0xF2) 原样取回; 每一份发往 PS5 的 64B
//    输入报告必须先写入加密狗、再由加密狗返回 (末 8 字节 hash 由它填写) 后才能
//    上线 — 发往 PS5 的字节永远是加密狗返回的字节, 本地只做透传与改写摇杆。
//
//  协议的全部常量与行为以 GP2040-CE 固件源码为事实源 (本仓库
//    docs/p5general/p5general-protocol.zh-CN.md 的逐行转述 + 描述符逐字节核对),
//    失败路径按本机需要做了偏离 (文件头 pad_p5g.cu 有逐条说明)。
//
//  分层: 与 pad_xinput 同构 — 逻辑态 (PadLogical) 只在本文件与线格式相接;
//    usbraw 不含设备知识, 合并层 (pad_output) 不含线格式知识。手柄输入、合并与
//    标定与 pad 模式完全共用 (同一 CAL_MODE_PAD、同一条 L_EST_PAD 回写、同一套
//    逐轴 spd 倍率), 仅输出后端不同。
// ============================================================================

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <mutex>

#include "io/pad_input.h"                // PadLogical / PadBtn / P5G_DONGLE_VID/PID
#include "io/usbraw.h"

// 输入报告长度: 线上 64B (报告 ID 0x01 占首字节), IN/OUT 端点 wMaxPacketSize=64
constexpr size_t PAD_P5G_REPORT_LEN = 64;

// ---- 摇杆 8 位口径: 逻辑域 → 线上码 ----
// 线上是 8 位 (描述符声明 6 个 8 位绝对轴, 逻辑 0–255, 中位 0x80; 参考固件把 16 位
//   内部状态右移 8 位放入该字段 — 三方一致, 见 docs/p5general §2/§4.1), 逻辑域是
//   pad_input 的 8→16 展开表: 中点 0x80 → 精确 0, 步长 258 = floor(32767/127),
//   最深一级 128×258 = 33024 越界后被端点吸收成 −32767。逆映射因此是**一次
//   floor(v/258)+128 再钳到 0..255**, 而不是参考固件的 >>8:
//     · 参考固件的 >>8 是它自己那条 16 位映射的精确逆 — GP2040-CE 的入站转换是
//       map(v, 0..255, 0..65535) = v×257, 而 (v×257)>>8 = v 对 0..255 逐个恒等
//       (v×257 = v×256 + v, 右移 8 位即 v + floor(v/256) = v);
//     · 本库的逻辑域不是 v×257 而是 (v−128)×258: 拿 (v+32768)>>8 (DualSense 口径)
//       或 (v>>8)+128 (算术右移口径) 去逆它, **256 个码值里有 127 个差一**
//       (码 1 → 线上 0, 码 2 → 1, …, 码 127 → 126; 正半程与两端恰好恒等, 因为
//       258 只在负半程不整除 256), 即人手通道被系统性缩小一档;
//     · floor(v/258) 是那张表的精确逆 (0 → −32767 → 0; 255 → +32766 → 255;
//       254 个中间码各自恒等), 且它读作"以逻辑步长为单位的算术右移" — 258 就是
//       该轴的一个线上台阶的 16 位长度。取整方向取 floor (向 −∞) 与算术右移
//       同向, 使下面的加法分解成立。
//   加法分解: 因码值对应的逻辑量都是 258 的整数倍 (码 0 的 −32767 除外, 它是端点
//   吸收的产物), floor((h+i)/258) = h/258 + floor(i/258) — 于是线上码就是
//   "人手的原始码 + 注入的 16 位量按同一台阶折算", 正是落点规则要求的形态: 人手
//   通道原样透传不扩大, 只有 aimbot 注入的那部分在 16 位域算完再折算进来。
//   (合并层的径向钳制会参与: 钳制后的和仍满足上式, 单测逐点钉住。)
inline uint8_t p5g_stick8(int16_t logical) {
    const int v = logical;
    const int q = v >= 0 ? v / 258 : -((-v + 257) / 258);   // floor(v/258), 向 −∞
    return (uint8_t)std::clamp(q + 128, 0, 255);
}

// 逻辑态 → 64 字节输入报告 (未签名形态): 报告 ID/6 摇杆轴/扳机/摇杆计数位/十字
//   hat/按键位表/触摸板(无触摸源: 双触点常态未按下于中心)/0x001A 特征字; 末 8B
//   hash 恒 0 (加密狗回填)。reportCounter/auth_seq_number 恒 0 — 参考固件从不
//   维护, 若线上要求单调性由加密狗完成 (协议文档 §10.9)。
void pad_p5g_report(const PadLogical& st, uint8_t out[PAD_P5G_REPORT_LEN]);

// 设备定义 (设备/配置/报告描述符/字符串/端点/全速/类请求钩子/报告率标签) ——
//   会话启动与单测共用的同一份事实
const UsbRawDeviceDef& pad_p5g_usb_def();

// ---- 认证状态机 (GP2040-CE P5GeneralAuthUSBListener 的逐规则移植) ------------
// 加密狗 I/O 以函数注入: 真机 = hidraw 控制传输, 单测 = 合成应答。全部阻塞式;
//   返回 false = 传输失败 (实现侧自行标记失联, 状态机只放弃本回合回 idle —
//   PS5 的超时重发会重启回合; 参考固件的失败路径把状态永久挂在 wait 上, 不复制)。
struct P5gAuthIo {
    void* ctx = nullptr;
    bool (*set_f0)(void*, const uint8_t* buf64);   // SET_REPORT(Feature,0xF0) 64B 质询
    bool (*get_f1)(void*, uint8_t* buf64);         // GET_REPORT(Feature,0xF1) ← 64B
    bool (*get_f2)(void*, uint8_t* buf16);         // GET_REPORT(Feature,0xF2) ← 16B
};

class P5gAuth {
public:
    // —— 设备侧 (usbraw 控制线程调用; 只动共享状态, 不做加密狗 I/O) ——
    // SET_REPORT(Feature,0xF0) 的 64B 载荷 (含 ID 字节): 仅 idle 态接受入队
    void accept_f0(const uint8_t* raw64);
    // GET_REPORT(Feature,0xF1): 64B 应答 = auth_buffer 全量 (滞后一次取数语义);
    //   idle 态则武装加密狗线程取下一份 F1
    void reply_f1(uint8_t* out64);
    // GET_REPORT(Feature,0xF2): 16B 应答 = [0xF2]+auth_buffer[1..15]; idle 态则
    //   武装 (与 F1 同一武装路径 — 参考固件两分支共代码, 保留该既有行为)
    void reply_f2(uint8_t* out16);
    // 重枚举 / 加密狗重插: 回 idle, f1_num 清零 (缓冲不清 — 内容只被整份覆盖)
    void reset();
    void set_dongle_ready(bool on);

    // —— 主机侧 (p5g 后端循环逐毫秒驱动; 推进转发/取回, 阻塞 I/O 在锁外) ——
    void tick(uint64_t now_us, const P5gAuthIo& io);

private:
    enum class St : uint8_t { Idle, SendF0, RecvF1, RecvF2Delay };
    void dispatch_locked(uint64_t now_us);         // F0 ACK 后按类型字节分派 (锁内)
    mutable std::mutex mtx_;
    St st_ = St::Idle;
    int f1_num_ = 0;                               // F1 后续取回配额 (4/1/0 按质询类型)
    uint64_t f2_at_us_ = 0;                        // F2 自动取回时刻 (ACK + 500ms)
    uint64_t io_deadline_us_ = 0;                  // 看门狗: 仅加密狗在位时计时
    uint8_t auth_buffer_[64]{};                    // 质询/签名/状态的共享缓冲 (整份覆盖)
    bool dongle_ready_ = false;
};

// ---- 签名流水线 (单槽; 与参考固件逐规则对齐, 纯决策无 I/O) -------------------
struct P5gFlow {
    uint8_t last[PAD_P5G_REPORT_LEN]{};            // 上一份提交加密狗的报告 (变化基线)
    bool have_last = false;
    int repeat = 0;                                // 内容不变时的重复发送配额 (变化时置 4)
    bool pending = false;                          // 已提交加密狗、签名未回
    uint64_t pending_us = 0;                       // pending 起始时刻 (看门狗)
    bool ready = false;                            // 签名已回、待发 PS5
    uint8_t finish[PAD_P5G_REPORT_LEN]{};          // 签名回读槽
};

// 决定是否把当前合并态提交加密狗: 是则写 out 并置 pending。变化 → 提交 + 配额 4;
//   不变且配额未尽 → 重发同一份、配额递减; 加密狗不在位或上一份签名未回 → 不产生
//   新报告 (加密狗未就绪前不产生任何输出 / pending 期间不再产生, 均为参考固件规则)。
bool p5g_flow_submit(const uint8_t* rpt64, bool dongle_ready, P5gFlow& f,
                     uint64_t now_us, uint8_t* out);
// 加密狗签名回填: 上一份尚未发走则丢弃新到的 (单槽背压靠丢弃, 参考固件同规则)
bool p5g_flow_signed(const uint8_t* in64, P5gFlow& f);
// 已发往 PS5, 清回读槽
void p5g_flow_sent(P5gFlow& f);
// pending 看门狗: 签名超时未回即放弃该份 (不放弃则一次加密狗抖动会永久卡死报告流)
void p5g_flow_tick(P5gFlow& f, uint64_t now_us);

// 输出后端: raw_gadget 会话 (对 PS5) + hidraw 加密狗主机驱动 + 后端循环。加密狗
//   缺席不阻塞启动 (1s 重扫); 失败 (raw_gadget 不可用) 返回 false。
bool pad_p5g_start();
void pad_p5g_stop();
