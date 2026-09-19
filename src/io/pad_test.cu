// ============================================================================
//  pad_test — build/pad_test 单测 (scripts/compile.sh 构建, calib_test 之后执行):
//    [1] 注入换算与合并钳制 (3000px/s: 1.5px/ms=半偏, 3px/ms=满偏; 人类+注入
//        ±32767 钳制, 两轴独立)
//    [2] 全透传 1:1: 按键/左摇杆/扳机逐位直通; 零注入时右摇杆=人类通道
//    [3] 摇杆账本 Σ(合并偏转×实际拍时长) (g_counts 不变式 3 的 pad 对应物)
//    [4] 触发判据与 -k 映射: RT≠0=fire / LT≠0=ads, 注入门 = 接管 × 保持窗
//    [5] own_motion_ledger 模式路由 (hid=g_counts / pad=摇杆账本)
//    [6] 手柄未在位: 查找空路径, reader 线程不阻塞不停机
//    [7] 发布点: pad_tick 覆盖写最新槽 + seq 单调递增; 无新拍 seq 不变,
//        内容 = 人类态合并零注入 (无目标干跑)
//    [8] XInput 线格式: 20B 报告的报头/按键位表/扳机直映/摇杆 int16LE 组装与
//        Y 轴口径 (逻辑态上为负 ↔ 线上上为正), 保留字节恒零
//    [9] XInput 设备字节: 身份/配置节/类特定 blob/双端点/无限定符/全速/vendor
//        不设钩子/字符串 — 上线外观的逐项断言
//  全部断言通过输出 ALL PASS 并返回 0。
// ============================================================================

#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>
#include <string>
#include <thread>

#include "core/control.h"
#include "core/state.h"
#include "io/pad_input.h"
#include "io/pad_output.h"
#include "io/pad_xinput.h"

static int g_fail = 0;

#define CHECK(cond, msg) do { \
    if (cond) { std::cout << "  ok  " << msg << "\n"; } \
    else { std::cerr << "  FAIL " << msg << "\n"; ++g_fail; } \
} while (0)

int main() {
    using namespace std::chrono;
    auto t0 = steady_clock::now();
    auto at = [&](int k) { return shift_ms(t0, 2.0 * k); };   // 2ms 步进 = 合成拍时长

    std::cout << "[1] 注入换算与合并钳制\n";
    {
        PadLogical h; h.rx = 100; h.lx = 100;
        PadLogical o = pad_merge(h, 1.5f, 0.0f, 3000.0f, at(0));
        CHECK(o.rx == 100 + 16384 && o.lx == 100,
              "1.5px/ms @3000px/s = 半偏 (16384), 人类通道直通");
        o = pad_merge(h, 3.0f, 0.0f, 3000.0f, at(1));
        CHECK(o.rx == 32767, "3px/ms @3000px/s = 满偏 (上限内)");
        o = pad_merge(h, 9.9f, 0.0f, 3000.0f, at(2));
        CHECK(o.rx == 32767, "超速注入+人类通道 → +满偏钳制");
        PadLogical hn; hn.rx = -100;
        o = pad_merge(hn, -9.9f, 0.0f, 3000.0f, at(3));
        CHECK(o.rx == -32767, "负向超速+人类通道 → −满偏钳制");
        o = pad_merge(h, 0.0f, 1.5f, 3000.0f, at(4));
        CHECK(o.ry == 16384 && o.rx == 100, "y 轴注入独立换算, x 轴不受扰");
        o = pad_merge(h, 1.5f, 0.0f, 6000.0f, at(5));
        CHECK(o.rx == 100 + 8192, "stick_gain 加倍 → 同速度注入减半 (满偏比例反比)");
    }

    std::cout << "[2] 全透传 1:1\n";
    {
        PadLogical h; h.lx = -32767; h.ly = 12345; h.lt = 255; h.rt = 137;
        h.btns = PADBTN_A | PADBTN_START | PADBTN_DPAD_LEFT;
        PadLogical o = pad_merge(h, 0.0f, 0.0f, 3000.0f, at(10));
        CHECK(o.lx == h.lx && o.ly == h.ly && o.lt == h.lt && o.rt == h.rt
              && o.btns == h.btns, "按键/左摇杆/扳机逐位直通 (扳机模拟量无阈值)");
        CHECK(o.rx == 0 && o.ry == 0, "零注入 → 右摇杆 = 人类通道 (合成数学 v=0 特例)");
    }

    std::cout << "[3] 摇杆账本 Σ(偏转·拍时长)\n";
    {
        PadLogical h; h.rx = 100;
        auto l0 = g_pad_ledger.cum();
        // 拍时刻按 2ms 步进合成 → h 精确; 每拍账本 = 合并偏转×2ms
        pad_merge(h, 1.5f, 0.0f, 3000.0f, at(11));    // rx=16484, h=2ms
        pad_merge(h, 3.0f, 0.0f, 3000.0f, at(12));    // rx=32767, h=2ms
        auto l1 = g_pad_ledger.cum();
        CHECK(l1.first - l0.first == (long long)(16484 + 32767) * 2
              && l1.second - l0.second == 0,
              "账本 = 合并偏转×实际拍时长 (偏转·ms), 未动轴不入账");
    }

    std::cout << "[4] 触发判据与 -k 映射\n";
    {
        bool saved_aim = g_aim_enabled.load();
        int saved_mode = g_aim_mode.load();
        g_aim_enabled.store(true);
        float vx, vy; bool g;
        g_aim_mode.store(0);                           // -k fire
        g = control_apply_pad(120, LEFT_KEY, vx, vy);
        CHECK(g, "fire 触发 (-k fire) → 门开");
        g = control_apply_pad(120, 0, vx, vy);
        CHECK(g, "松开后 KEEP_ALIVE 窗内门仍开");
        std::this_thread::sleep_for(milliseconds(KEEP_ALIVE_MS + 80));
        g = control_apply_pad(120, 0, vx, vy);
        CHECK(!g, "保持窗过后门关");
        g = control_apply_pad(120, RIGHT_KEY, vx, vy);
        CHECK(!g, "ads 触发在 -k fire 下门不开");
        g_aim_mode.store(1);                           // -k ads
        g = control_apply_pad(120, RIGHT_KEY, vx, vy);
        CHECK(g, "-k ads 下 LT 触发门开");
        std::this_thread::sleep_for(milliseconds(KEEP_ALIVE_MS + 80));
        g = control_apply_pad(120, LEFT_KEY, vx, vy);
        CHECK(!g, "-k ads 下 RT 不开门");
        g_aim_mode.store(2);                           // -k both
        g = control_apply_pad(120, LEFT_KEY, vx, vy);
        CHECK(g, "-k both 下任一触发门开");
        CHECK(vx == 0.0f && vy == 0.0f, "无有效目标时期望速度为 0 (门开≠注入)");
        g_aim_enabled.store(false);
        g = control_apply_pad(120, LEFT_KEY, vx, vy);
        CHECK(!g, "接管关闭 → 门关 (纯透传)");
        g_aim_enabled.store(saved_aim);
        g_aim_mode.store(saved_mode);
    }

    std::cout << "[5] own_motion_ledger 路由\n";
    {
        own_motion_ledger_set(false);
        CHECK(&own_motion_ledger() == &g_counts, "hid 路由 = g_counts");
        own_motion_ledger_set(true);
        CHECK(&own_motion_ledger() == &g_pad_ledger, "pad 路由 = 摇杆账本");
        own_motion_ledger_set(false);
    }

    std::cout << "[6] 手柄未在位\n";
    {
        const std::string absent = "no_such_pad_substr_xyz";
        CHECK(find_pad_device(absent).empty(), "无匹配子串 → 空路径");
        CHECK(find_pad_device("/nonexistent_dev_node").empty(), "不存在的绝对路径 → 空路径");
        PadState pts;
        std::thread r(pad_reader_thread, absent, std::ref(pts));
        std::this_thread::sleep_for(milliseconds(300));
        CHECK(global_running.load(), "reader 未在位时不停机不阻塞");
        global_running = false;
        r.join();
        global_running = true;
    }

    std::cout << "[7] 发布点 (输出后端接入契约)\n";
    {
        // 干跑: 无有效目标 → 律速度 0, 发布内容 = 人类态合并零注入
        PadState in;
        { std::lock_guard<std::mutex> lk(in.mtx);
          in.st.lx = -5000; in.st.ly = 7000; in.st.rx = 900;
          in.st.btns = PADBTN_B | PADBTN_DPAD_UP; }
        uint64_t s0, s1, s2;
        pad_publish_snapshot(&s0);
        pad_tick(120, in, false);
        PadLogical p = pad_publish_snapshot(&s1);
        CHECK(s1 == s0 + 1, "pad_tick → seq 单调 +1");
        CHECK(p.lx == -5000 && p.ly == 7000 && p.rx == 900 && p.btns == (PADBTN_B | PADBTN_DPAD_UP),
              "发布内容 = 合并逻辑态 (人类态 + 零注入, 逐字段)");
        pad_publish_snapshot(&s2);
        CHECK(s2 == s1, "无新拍 → seq 不变 (后端可跳过重发)");
    }

    std::cout << "[8] XInput 线格式 (20B 报告)\n";
    {
        auto i16le = [](const uint8_t* p) { return (int)(int16_t)(p[0] | (p[1] << 8)); };
        uint8_t r[PAD_XINPUT_REPORT_LEN];

        PadLogical n;
        pad_xinput_report(n, r);
        CHECK(r[0] == 0x00 && r[1] == 0x14 && r[2] == 0x00 && r[3] == 0x00,
              "中性报告前 4 字节 = 00 14 00 00 (报头 类型 0 / 长度 20 / 按键位全零)");
        bool zero_tail = true;
        for (int i = 4; i < 14; ++i) zero_tail = zero_tail && r[i] == 0;
        CHECK(zero_tail && i16le(r + 6) == 0 && i16le(r + 10) == 0,
              "中性报告: 扳机 0, 摇杆中心 0");

        PadLogical a; a.btns = PADBTN_A;
        pad_xinput_report(a, r);
        CHECK(r[3] == 0x10, "A 键 → byte3 bit4 = 0x10");
        a.btns = PADBTN_A | PADBTN_B;
        pad_xinput_report(a, r);
        CHECK(r[3] == 0x30, "A+B 组合 → byte3 = 0x30 (位叠加)");

        PadLogical d; d.btns = PADBTN_DPAD_UP | PADBTN_START | PADBTN_L3;
        pad_xinput_report(d, r);
        CHECK(r[2] == 0x51, "十字上+START+L3 → byte2 = 0x51 (bit0/4/6)");
        d.btns = PADBTN_DPAD_DOWN | PADBTN_DPAD_LEFT | PADBTN_DPAD_RIGHT
               | PADBTN_BACK | PADBTN_R3;
        pad_xinput_report(d, r);
        CHECK(r[2] == 0xAE, "十字下/左/右+BACK+R3 → byte2 = 0xAE (bit1/2/3/5/7)");
        d.btns = PADBTN_LB | PADBTN_RB | PADBTN_GUIDE | PADBTN_X | PADBTN_Y;
        pad_xinput_report(d, r);
        CHECK(r[3] == 0x07 + 0x40 + 0x80,
              "LB+RB+Guide+X+Y → byte3 = 0xC7 (保留 bit3 恒 0)");

        PadLogical t; t.lt = 255; t.rt = 137;
        pad_xinput_report(t, r);
        CHECK(r[4] == 255 && r[5] == 137, "扳机模拟量直映 (LT=255 / RT=137 原值)");

        PadLogical ax; ax.lx = 1234; ax.ly = 1234; ax.rx = -1; ax.ry = -32768;
        pad_xinput_report(ax, r);
        CHECK(r[6] == 0xD2 && r[7] == 0x04 && i16le(r + 6) == 1234,
              "LX int16 小端组装 (1234 → D2 04)");
        CHECK(r[8] == 0x2E && r[9] == 0xFB && i16le(r + 8) == -1234,
              "LY 线上上为正: 逻辑态上推 (+1234) → 线上 -1234 (口径取反, 非位反转)");
        CHECK(i16le(r + 10) == -1, "RX 符号直通 (左为负在两套口径下相同)");
        CHECK(i16le(r + 12) == 32767,
              "RY 逻辑满偏上推 → 线上 +32767 (满偏取反饱和, 不越 int16 域)");
        ax.ry = 32767;
        pad_xinput_report(ax, r);
        CHECK(i16le(r + 12) == -32767, "RY 逻辑满偏下推 → 线上 -32767 (满偏取反仍满偏)");

        bool tail0 = true;
        for (int i = 14; i < (int)PAD_XINPUT_REPORT_LEN; ++i) tail0 = tail0 && r[i] == 0;
        CHECK(tail0 && r[1] == (uint8_t)PAD_XINPUT_REPORT_LEN,
              "14–19 保留字节恒零, 长度字节 = 报告长");
    }

    std::cout << "[9] XInput 设备字节\n";
    {
        const UsbRawDeviceDef& d = pad_xinput_usb_def();
        CHECK(d.device.idVendor == 0x045E && d.device.idProduct == 0x028E
              && d.device.bcdDevice == 0x0572 && d.device.bcdUSB == 0x0200,
              "身份 = 微软有线 360 手柄 (0x045E/0x028E, bcdDevice 0x0572, USB 2.0)");
        CHECK(d.device.bDeviceClass == 0xFF && d.device.bDeviceSubClass == 0xFF
              && d.device.bDeviceProtocol == 0xFF && d.device.bMaxPacketSize0 == 64
              && d.device.bNumConfigurations == 1,
              "厂商类三元组 + ep0 64B + 单配置");
        CHECK(d.qualifier == nullptr,
              "无限定符 (全速设备) → GET_DESCRIPTOR(DEVICE_QUALIFIER) STALL");
        CHECK(d.speed == USB_SPEED_FULL, "全速枚举 (端点 bInterval 按 1ms 帧解释)");
        CHECK(d.config_len == 48 && d.config[2] == 48 && d.config[3] == 0
              && d.config[4] == 1 && d.config[5] == 1 && d.config[7] == 0x80,
              "配置节 48B 单接口单配置 / 总线供电 / wTotalLength 自洽");
        CHECK(d.config[8] == 0xFA, "bMaxPower = 0xFA → VBUS 请求 500mA");
        CHECK(d.config[18] == 0x10 && d.config[19] == 0x21 && d.config[24] == 0x81
              && d.config[25] == 0x14,
              "类特定 16B blob: bLength 0x10 / 类型 0x21 / IN 端点 0x81");
        CHECK(d.config[14] == 0xFF && d.config[15] == 0x5D && d.config[16] == 0x01
              && d.config[13] == 2,
              "接口 FF/5D/01 + 双端点 (XInput 绑定键)");
        CHECK(d.ep_in.bEndpointAddress == 0x81 && d.ep_in.wMaxPacketSize == 32
              && d.ep_in.bmAttributes == USB_ENDPOINT_XFER_INT && d.ep_in.bInterval == 4,
              "中断 IN 0x81 / 32B / bInterval=4 (=4ms @全速 → 250Hz)");
        CHECK(d.has_ep_out && d.ep_out.bEndpointAddress == 0x02
              && d.ep_out.wMaxPacketSize == 32 && d.ep_out.bInterval == 8,
              "中断 OUT 0x02 / 32B / bInterval=8 (LED/力反馈命令的落点)");
        CHECK(d.report_desc == nullptr && d.report_desc_len == 0,
              "无 HID 报告描述符 (厂商接口非 HID)");
        CHECK(d.vendor_request == nullptr,
              "不设 vendor 钩子 → xusb 初始化探测与一切 vendor 请求 STALL");
        CHECK(d.report_desc == nullptr && PAD_XINPUT_REPORT_LEN == 20
              && PAD_XINPUT_REPORT_LEN <= d.ep_in.wMaxPacketSize,
              "报告长 20B 在单包 (32B) 内: 一次 EP_WRITE = 一次传输");
        CHECK(d.string_count == 3 && !strcmp(d.strings[0].utf8, "GENERIC")
              && !strcmp(d.strings[1].utf8, "XINPUT CONTROLLER")
              && !strcmp(d.strings[2].utf8, "1.0"),
              "字符串 GENERIC / XINPUT CONTROLLER / 1.0");
    }

    std::cout << (g_fail ? "FAILED\n" : "ALL PASS\n");
    return g_fail ? 1 : 0;
}
