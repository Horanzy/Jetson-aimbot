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
//   [10] 手柄标定激励轨迹表: 段时长 (墙钟) 与每段单轴满偏转
//   [11] 标定状态机: L3+R3 长按 5s / 热参请求触发, 中途松开复位, 相位推进
//        (起始方块 → 激励采样 → 静置 → 请求计算 → 等回执 → 点头/摇头 → 空闲),
//        接管关闭时不可达且进行中的标定复位
//   [12] 激励期独占右摇杆 (人类右摇杆被忽略, 其余字段直通) 与账本入账
//   [13] 量纲换算 (px per 偏转·ms ↔ 满偏转屏速 px/s) 与设计带钳制/判定
//   [14] 标定拟合: 合成账本 + 相位位移 → 复原设计增益与延迟; 灵敏度钳制带只
//        在越界时咬合 (带内与开路带逐位同解) — hid 分支的既有语义
//   [15] 回写: persist_calibration 的 VAR 名参数化 (hid/pad 两套互不覆盖),
//        原子替换/权限保留/缺行追加
//  全部断言通过输出 ALL PASS 并返回 0。
// ============================================================================

#include <chrono>
#include <cmath>
#include <cfloat>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

#include "core/calib.h"
#include "core/control.h"
#include "core/state.h"
#include "io/pad_calib.h"
#include "io/pad_input.h"
#include "io/pad_output.h"
#include "io/pad_xinput.h"

static int g_fail = 0;

#define CHECK(cond, msg) do { \
    if (cond) { std::cout << "  ok  " << msg << "\n"; } \
    else { std::cerr << "  FAIL " << msg << "\n"; ++g_fail; } \
} while (0)

static double seg_ms(const CalibSeg& s) { return s.ticks * (double)TICK_MS; }
static bool seg_full_axis(const CalibSeg& s) {          // 单轴满偏转 (另一轴为 0)
    return std::abs(s.dx) + std::abs(s.dy) == PAD_AXIS_MAX;
}
static std::string read_file(const std::string& p) {
    std::ifstream in(p); std::ostringstream o; o << in.rdbuf(); return o.str();
}
static bool has_line(const std::string& text, const std::string& line) {
    std::istringstream in(text); std::string l;
    while (std::getline(in, l)) if (l.rfind(line, 0) == 0) return true;
    return false;
}

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

    std::cout << "[10] pad 标定激励轨迹表 (墙钟/幅度)\n";
    {
        CHECK(PAD_CAL_TRIGGER_TICKS * (double)TICK_MS == 5000.0,
              "L3+R3 长按触发 = 5000ms 墙钟 (拍数由墙钟导出)");
        bool full = true;
        for (const CalibSeg& s : PAD_CAL_START_SEQ) if (s.dx || s.dy) full &= seg_full_axis(s);
        for (const CalibSeg& s : PAD_CAL_EXCITE_SEQ) full &= seg_full_axis(s);
        for (const CalibSeg& s : PAD_CAL_END_OK_SEQ) full &= seg_full_axis(s);
        for (const CalibSeg& s : PAD_CAL_END_FAIL_SEQ) full &= seg_full_axis(s);
        CHECK(full, "所有运动段均为单轴满偏转 (±32767)");
        CHECK((int)(sizeof(PAD_CAL_START_SEQ) / sizeof(CalibSeg)) == 5
              && seg_ms(PAD_CAL_START_SEQ[0]) == 240.0
              && seg_ms(PAD_CAL_START_SEQ[4]) == 500.0,
              "起始方块 = 4 边 × 240ms + 500ms 停顿 (与 hid 标定同设计值)");
        CHECK((int)(sizeof(PAD_CAL_EXCITE_SEQ) / sizeof(CalibSeg)) == 4
              && seg_ms(PAD_CAL_EXCITE_SEQ[0]) == 250.0 && seg_ms(PAD_CAL_SETTLE_SEQ[0]) == 300.0,
              "激励单圈 4 边 × 250ms, 静置 300ms");
        CHECK(PAD_CAL_END_OK_SEQ[0].dy == PAD_AXIS_MAX && PAD_CAL_END_OK_SEQ[0].dx == 0
              && PAD_CAL_END_FAIL_SEQ[0].dx == PAD_AXIS_MAX && PAD_CAL_END_FAIL_SEQ[0].dy == 0
              && seg_ms(PAD_CAL_END_OK_SEQ[0]) == 60.0,
              "收尾: 成功纵向点头 / 失败横向摇头, 60ms/程");
    }

    std::cout << "[11] 标定触发与相位推进\n";
    {
        bool saved_aim = g_aim_enabled.load();
        g_aim_enabled.store(true);
        g_padcalib_request.store(false);
        const uint16_t both = PADBTN_L3 | PADBTN_R3;
        PadCalibStep s{false, 0, 0};
        auto idle_n = [&](int n) { for (int i = 0; i < n; ++i) s = pad_calib_step(0); };
        idle_n(5);

        for (int i = 0; i < PAD_CAL_TRIGGER_TICKS - 1; ++i) s = pad_calib_step(both);
        CHECK(!s.active, "L3+R3 长按差一拍 (4999 拍) → 不触发");
        s = pad_calib_step(0);
        CHECK(!s.active, "中途松开 → 仍空闲");
        for (int i = 0; i < PAD_CAL_TRIGGER_TICKS - 1; ++i) s = pad_calib_step(both);
        CHECK(!s.active, "松开后重新计时 (再差一拍仍不触发)");
        s = pad_calib_step(both);
        CHECK(s.active && s.dx == 0 && s.dy == 0,
              "满 5000 拍 → 触发 (触发拍不播激励, 与 hid 同形)");

        // 相位推进 (起始方块 → 激励 → 静置 → 请求 → 等回执 → 收尾 → 空闲):
        //   段/拍的推进一步一拍, 段切换发生在该段最后一拍 (切换拍返回的值仍属该段)
        auto run = [&](int n) { for (int i = 0; i < n; ++i) s = pad_calib_step(0); };
        const int edge = ms_to_ticks(240), exc = ms_to_ticks(250);
        run(1);
        CHECK(s.dx == PAD_AXIS_MAX && s.dy == 0, "起始方块 1/4: +X 满偏");
        run(edge - 1);
        run(edge); CHECK(s.dy == PAD_AXIS_MAX && s.dx == 0, "2/4: +Y 满偏");
        run(edge); CHECK(s.dx == -PAD_AXIS_MAX && s.dy == 0, "3/4: -X 满偏");
        run(edge); CHECK(s.dy == -PAD_AXIS_MAX && s.dx == 0, "4/4: -Y 满偏");
        run(ms_to_ticks(500) - 1);
        CHECK(!g_calib_collect.load(), "起始方块期不采样 (纯视觉开始信号)");
        run(1);
        CHECK(g_calib_collect.load(), "起始方块结束 → 采样开 (激励自下一拍起)");
        run(exc); CHECK(s.dx == PAD_AXIS_MAX, "激励 1/4: +X 满偏");
        run(exc); CHECK(s.dy == PAD_AXIS_MAX, "2/4: +Y");
        run(exc); CHECK(s.dx == -PAD_AXIS_MAX, "3/4: -X");
        run(exc); CHECK(s.dy == -PAD_AXIS_MAX, "4/4: -Y");
        run(exc * 4 * 4);
        run(ms_to_ticks(300) - 1);
        CHECK(g_calib_collect.load() && s.dx == 0 && s.dy == 0, "静置期仍采样 (样本收尾), 摇杆归零");
        run(1);
        CHECK(!g_calib_collect.load() && g_calib_request.load(),
              "激励结束 → 采样关, 请求 ai_thread 拟合");
        run(3);
        CHECK(s.active && s.dx == 0 && s.dy == 0, "等待回执期摇杆静置 (律让位)");
        g_calib_request.store(false);
        g_calib_done.store(1);
        run(1);                                  // 收到回执 → 进入收尾相位
        run(1);
        CHECK(s.dy == PAD_AXIS_MAX && s.dx == 0, "回执成功 → 纵向点头收尾");
        run(ms_to_ticks(60) * 6);
        run(1);
        CHECK(!s.active, "收尾结束 → 回到空闲");
        g_calib_done.store(0);

        // 热参请求 (一次消费即清) 与单键不触发
        for (int i = 0; i < PAD_CAL_TRIGGER_TICKS + 5; ++i) s = pad_calib_step(PADBTN_L3);
        CHECK(!s.active, "只按 L3 (满 5s) 不触发");
        g_padcalib_request.store(true);
        s = pad_calib_step(0);
        CHECK(s.active && s.dx == 0 && s.dy == 0, "热参 padcalib=1 → 同样触发");
        CHECK(!g_padcalib_request.load(), "标定请求一次消费即清 (exchange)");
        run(1);
        CHECK(s.active && s.dx == PAD_AXIS_MAX, "请求已清零但标定继续 (激励自下一拍起)");
        g_padcalib_request.store(true);
        run(1);
        CHECK(!g_padcalib_request.load() && s.active,
              "标定进行中的请求一次消费即清 (不重入, 本次不被打断)");

        // 接管关闭: 不可达 + 进行中的标定复位
        g_aim_enabled.store(false);
        s = pad_calib_step(0);
        CHECK(!s.active, "接管关闭 → 进行中的标定复位 (标定不可达)");
        g_padcalib_request.store(true);
        s = pad_calib_step(0);
        CHECK(!s.active && !g_padcalib_request.load(), "接管关闭 → 请求被忽略且消费");
        g_aim_enabled.store(true);
        for (int i = 0; i < PAD_CAL_TRIGGER_TICKS + 5; ++i) s = pad_calib_step(PADBTN_L3 | PADBTN_R3);
        CHECK(s.active, "重新开启接管 → 长按可再次触发 (状态机未卡死)");
        g_aim_enabled.store(false);
        pad_calib_step(0);                       // 复位
        g_aim_enabled.store(saved_aim);
        g_calib_collect.store(false); g_calib_request.store(false); g_calib_done.store(0);
    }

    std::cout << "[12] 激励期独占右摇杆与账本\n";
    {
        // 时间基挪到远未来: 上一用例的 pad_tick 按实测时钟入账, 此处用可控步进
        auto af = [&](int k) { return shift_ms(t0, 60000.0 + 2.0 * k); };
        PadLogical warm;
        pad_merge(warm, 0.0f, 0.0f, PAD_STICK_GAIN_DEFAULT, af(0));      // 预热拍
        auto l0 = g_pad_ledger.cum();
        PadLogical h; h.rx = 9999; h.ry = -7777; h.lx = -123; h.lt = 200; h.rt = 31;
        h.btns = PADBTN_A | PADBTN_DPAD_LEFT;
        PadLogical o = pad_excite(h, PAD_AXIS_MAX, -PAD_AXIS_MAX, af(1));
        CHECK(o.rx == PAD_AXIS_MAX && o.ry == -PAD_AXIS_MAX,
              "激励期右摇杆 = 激励偏转 (人类右摇杆被忽略)");
        CHECK(o.lx == h.lx && o.lt == h.lt && o.rt == h.rt && o.btns == h.btns,
              "激励期其余字段逐位直通人类态");
        auto l1 = g_pad_ledger.cum();
        CHECK(l1.first - l0.first == (long long)PAD_AXIS_MAX * 2
              && l1.second - l0.second == (long long)(-PAD_AXIS_MAX) * 2,
              "激励按偏转×拍时长入账 (标定拟合的数据源)");
    }

    std::cout << "[13] 量纲换算与设计带\n";
    {
        const float g = PAD_STICK_GAIN_DEFAULT, s_rp = pad_s_rp_from_gain(g);
        CHECK(std::abs(s_rp * PAD_AXIS_MAX * 1000.0f - g) < 1e-3f,
              "s_rp × 32767 × 1000 = 满偏转屏速 px/s");
        CHECK(std::abs(pad_gain_from_s_rp(s_rp) - g) < 1e-3f, "逆换算往返一致");
        CHECK(std::abs(s_rp * PAD_AXIS_MAX * TICK_MS - g * TICK_MS / 1000.0f) < 1e-3f,
              "满偏一拍的屏移 = 满偏屏速 × 拍长 (账本单位的物理含义)");
        CHECK(pad_gain_clamp(1.0f) == PAD_GAIN_MIN && pad_gain_clamp(1e9f) == PAD_GAIN_MAX
              && pad_gain_clamp(g) == g, "设计带钳制 (带内不动)");
        CHECK(pad_calib_accept(g) && !pad_calib_accept(PAD_GAIN_MIN)
              && !pad_calib_accept(PAD_GAIN_MAX) && !pad_calib_accept(0.0f),
              "带内判定: 落带边 (= 钳制咬合) 与带外 → 不接受");
        CHECK(CALIB_BAND_PAD.s_min == pad_s_rp_from_gain(PAD_GAIN_MIN)
              && CALIB_BAND_PAD.s_max == pad_s_rp_from_gain(PAD_GAIN_MAX),
              "run_calibration 的钳制带 = 设计带换算");
    }

    std::cout << "[14] 标定拟合 (合成账本 + 相位位移)\n";
    {
        // 合成: 账本 = 满偏方波 (100ms 换向), 屏幕位移 = 真灵敏度 × 账本增量,
        //   样本按 120fps 帧周期给出 — 与真实链路 (账本 + 块相位相关位移) 同构。
        //   每次合成用更晚的时间基: 账本按时间单调 (at() 二分查询的前提)。
        const double DT = 1000.0 / 120.0;
        const float L_true = 50.0f, SYN_TICK_MS = 2.0f;     // 合成拍长 2ms
        auto synth = [&](bool pad, float s_true, double base_ms,
                         std::deque<CalibSample>& hist) {
            own_motion_ledger_set(pad);
            CountsHistory& led = pad ? g_pad_ledger : g_counts;
            // 幅度: pad = 满偏转; hid = 2 counts/ms (激励方波设计值 2 counts/拍@1kHz)
            const float amp = pad ? (float)PAD_AXIS_MAX : 2.0f;
            for (int k = 0; k < 1500; ++k) {                      // 3s 账本
                float v = ((k / 50) % 2) ? amp : -amp;            // 100ms 换向
                led.add(shift_ms(t0, base_ms + SYN_TICK_MS * k),
                        (int)std::lround(v * SYN_TICK_MS), 0);
            }
            hist.clear();
            for (int i = 0; i < 200; ++i) {                       // 200 帧样本
                auto t = shift_ms(t0, base_ms + 500.0 + i * DT);
                auto c1 = led.at(shift_ms(t, -(double)L_true));
                auto c0 = led.at(shift_ms(t, -(double)L_true - DT));
                hist.push_back({t, (float)DT,
                                s_true * (float)(c1.first - c0.first), 0.0f});
            }
        };
        const float gain_true = 2345.0f, s_rp_true = pad_s_rp_from_gain(gain_true);
        std::deque<CalibSample> hist;
        synth(true, s_rp_true, 120000.0, hist);
        float s = 0, l = 60.0f;                                // l 初值 ≠ 真值 (扫描找)
        bool ok = run_calibration(hist, s, l, CALIB_BAND_PAD);
        CHECK(ok, "pad 分支: 拟合成功");
        CHECK(std::abs(pad_gain_from_s_rp(s) - gain_true) / gain_true < 1e-4f,
              "pad 分支: 复原满偏屏速 (设计值 2345 px/s)");
        CHECK(std::abs(l - L_true) < 0.5f, "pad 分支: 复原环路延迟 (设计值 50ms)");
        CHECK(pad_calib_accept(pad_gain_from_s_rp(s)), "复原值落设计带内 → 接受");

        // hid 分支: 同一闭式解在 counts 单位制下复原 px/count, 且带内与开路带
        //   逐位同解 (钳制只在越界时咬合 — 既有 hid 语义不变)
        const float s_hid = 0.3017f;
        synth(false, s_hid, 200000.0, hist);
        float s1 = 0, l1 = 60.0f, s2 = 0, l2 = 60.0f;
        ok = run_calibration(hist, s1, l1, CALIB_BAND_COUNTS);
        CHECK(ok && std::abs(s1 - s_hid) / s_hid < 1e-4f && std::abs(l1 - L_true) < 0.5f,
              "hid 分支: 复原 px/count 与延迟");
        run_calibration(hist, s2, l2, CalibBand{-FLT_MAX, FLT_MAX});
        CHECK(std::memcmp(&s1, &s2, sizeof(float)) == 0
              && std::memcmp(&l1, &l2, sizeof(float)) == 0,
              "hid 分支: 带内结果与开路带逐位一致 (带只钳制, 不改拟合)");
        // 越界时 hid 带按既有语义钳到带边
        synth(false, 100.0f, 280000.0, hist);                  // 100 px/count ≫ S_MAX
        float s3 = 0, l3 = 60.0f;
        run_calibration(hist, s3, l3, CALIB_BAND_COUNTS);
        CHECK(s3 == S_MAX, "hid 分支: 越界拟合钳到 S_MAX (既有行为)");
        own_motion_ledger_set(false);
    }

    std::cout << "[15] 标定回写 (VAR 名参数化)\n";
    {
        char tmpl[] = "/tmp/pad_test_persist_XXXXXX";
        char* dir = mkdtemp(tmpl);
        CHECK(dir != nullptr, "临时目录");
        if (dir) {
            const std::string path = std::string(dir) + "/game.sh";
            { std::ofstream o(path);
              o << "#!/bin/bash\nS_EST=9.9900  # 占位\nL_EST=9.9\n"
                   "PAD_STICK_GAIN=1.0000  # 占位\nL_EST_PAD=9.9\nOUTPUT_MODE=\"pad\"\n"; }
            chmod(path.c_str(), 0750);
            CHECK(persist_calibration(path, "S_EST", 1.25f, "L_EST", 61.5f),
                  "hid 回写 (S_EST/L_EST) 成功");
            std::string txt = read_file(path);
            CHECK(has_line(txt, "S_EST=1.2500") && has_line(txt, "L_EST=61.5"),
                  "hid 值写入: %.4f 与 %.1f 格式");
            CHECK(has_line(txt, "PAD_STICK_GAIN=1.0000") && has_line(txt, "L_EST_PAD=9.9")
                  && has_line(txt, "OUTPUT_MODE=\"pad\""),
                  "hid 回写不触碰 pad 的 VAR (两套互不覆盖)");
            CHECK(txt.find("# 占位") != std::string::npos, "行内注释保留");
            CHECK(persist_calibration(path, PAD_CAL_VAR_GAIN, 3123.5f, PAD_CAL_VAR_L, 58.5f),
                  "pad 回写 (PAD_STICK_GAIN/L_EST_PAD) 成功");
            txt = read_file(path);
            CHECK(has_line(txt, "PAD_STICK_GAIN=3123.5000") && has_line(txt, "L_EST_PAD=58.5"),
                  "pad 值写入");
            CHECK(has_line(txt, "S_EST=1.2500") && has_line(txt, "L_EST=61.5"),
                  "pad 回写不触碰 hid 的 VAR (回归)");
            struct stat st{};
            CHECK(stat(path.c_str(), &st) == 0 && (st.st_mode & 07777) == 0750,
                  "原文件权限 (0750) 保留");
            CHECK(access((path + ".tmp").c_str(), F_OK) != 0, "无 .tmp 残留 (原子替换)");

            const std::string path2 = std::string(dir) + "/old.sh";
            { std::ofstream o(path2);
              o << "#!/bin/bash\nS_EST=9.9900\nL_EST=9.9\n"; }        // 无 pad 的 VAR
            CHECK(persist_calibration(path2, PAD_CAL_VAR_GAIN, 3000.0f, PAD_CAL_VAR_L, 60.0f),
                  "缺行脚本回写成功");
            txt = read_file(path2);
            CHECK(has_line(txt, "PAD_STICK_GAIN=3000.0000") && has_line(txt, "L_EST_PAD=60.0")
                  && has_line(txt, "S_EST=9.9900"),
                  "缺失的 VAR 追加写入, 既有行不动");
            CHECK(!persist_calibration(std::string(dir) + "/no_such.sh", "S_EST", 1.0f,
                                       "L_EST", 1.0f),
                  "不存在的脚本 → 回写失败 (不创建文件)");
            unlink(path.c_str()); unlink(path2.c_str()); rmdir(dir);
        }
    }

    std::cout << (g_fail ? "FAILED\n" : "ALL PASS\n");
    return g_fail ? 1 : 0;
}
