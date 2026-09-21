// ============================================================================
//  pad_test — build/pad_test 单测 (scripts/compile.sh 构建并执行): pad 输出模式
//    的输入映射、注入/合并几何、账本与发布点契约、XInput 线格式与设备字节。
//    [1] 8 位轴 → 16 位逻辑域 (均匀步长 258 + 溢出由端点吸收): 编译期表逐值等于
//        单测里独立写死的黄金表, 256 个码值互不相同, 步长直方图 {258:254, 1:1},
//        中点精确 0 / 最高一级 +32766 / 最深一级被端点吸收为 −32767, 全程单调;
//        已居中的 ±32767 器件原值直通; 扳机 0..255 映射
//    [2] 注入换算与合并钳制 (逐轴有效满偏屏速: 1.5px/ms@3000px/s = 半偏, 3px/ms
//        = 满偏; 人类 + 注入 ±32767 钳制, 两轴各用各自的有效增益互不串扰);
//        行程形状 = 圆 (径向限幅) 与账本按最终提交值入账
//    [3] 全透传 1:1 (按键/左摇杆/扳机逐位直通; 零注入时右摇杆 = 人类通道)
//    [4] 摇杆账本 Σ(合并偏转×实际拍时长) — g_counts 不变式 3 的 pad 对应物
//    [5] 触发阈值与 -k 映射、注入门 (接管 × 保持窗)、接管关闭即关
//    [6] 逐轴速度帽 min(-x, A_eff/1000): 命令饱和时输出正好落在该轴帽上, 且随
//        spd 变 (有效满偏屏速是注入通道的物理上限)
//    [7] own_motion_ledger 路由 + own_motion_scale 逐轴比例 (含 spd/ADS 联动)
//    [8] 发布点: pad_tick 覆盖写最新槽 + seq 单调递增; 内容 = 人类态合并零注入
//    [9] XInput 线格式: 20B 报告的报头/按键位表/扳机直映/摇杆 int16LE 组装与
//        Y 轴口径, 保留字节恒零, 无 XInput 落点的逻辑位 (触摸板) 不上线
//   [10] XInput 设备字节: 身份/配置节/类特定 blob/双端点/端点间隔与枚举速度的
//        刷新率推导/无限定符/全速/vendor 无钩子/字符串 — 上线外观的逐项断言
//   [11] 手柄未在位: 查找空路径, reader 线程不阻塞不停机
//   [12] 附属接口键翻译表 (KEY_SYSRQ → 触摸板位) 与 8/16 位分辨率对照
//   [13] pad 后端会话启动参数: 设备定义自洽 (报告长 ≤ 单包, EP_ENABLE 描述符与
//        配置节逐字节一致)
//  全部断言通过输出 ALL PASS 并返回 0。
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <unistd.h>
#include <linux/input.h>          // KEY_SYSRQ/KEY_A/KEY_ENTER (附属接口翻译表的用例)

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

// 8 位无符号轴映射的黄金表 (独立写死): 断言不调用生产侧的构造规则, 否则规则写错
//   时表与断言会一起错。期望性质: out[0]=−32767, out[128]=0, out[255]=+32766,
//   256 个值互异, 255 个相邻步里 254 个 = 258、1 个 = 1 (最底 code0→code1)。
static const int16_t GOLDEN_AXIS8[256] = {
    -32767, -32766, -32508, -32250, -31992, -31734, -31476, -31218,
    -30960, -30702, -30444, -30186, -29928, -29670, -29412, -29154,
    -28896, -28638, -28380, -28122, -27864, -27606, -27348, -27090,
    -26832, -26574, -26316, -26058, -25800, -25542, -25284, -25026,
    -24768, -24510, -24252, -23994, -23736, -23478, -23220, -22962,
    -22704, -22446, -22188, -21930, -21672, -21414, -21156, -20898,
    -20640, -20382, -20124, -19866, -19608, -19350, -19092, -18834,
    -18576, -18318, -18060, -17802, -17544, -17286, -17028, -16770,
    -16512, -16254, -15996, -15738, -15480, -15222, -14964, -14706,
    -14448, -14190, -13932, -13674, -13416, -13158, -12900, -12642,
    -12384, -12126, -11868, -11610, -11352, -11094, -10836, -10578,
    -10320, -10062,  -9804,  -9546,  -9288,  -9030,  -8772,  -8514,
     -8256,  -7998,  -7740,  -7482,  -7224,  -6966,  -6708,  -6450,
     -6192,  -5934,  -5676,  -5418,  -5160,  -4902,  -4644,  -4386,
     -4128,  -3870,  -3612,  -3354,  -3096,  -2838,  -2580,  -2322,
     -2064,  -1806,  -1548,  -1290,  -1032,   -774,   -516,   -258,
         0,    258,    516,    774,   1032,   1290,   1548,   1806,
      2064,   2322,   2580,   2838,   3096,   3354,   3612,   3870,
      4128,   4386,   4644,   4902,   5160,   5418,   5676,   5934,
      6192,   6450,   6708,   6966,   7224,   7482,   7740,   7998,
      8256,   8514,   8772,   9030,   9288,   9546,   9804,  10062,
     10320,  10578,  10836,  11094,  11352,  11610,  11868,  12126,
     12384,  12642,  12900,  13158,  13416,  13674,  13932,  14190,
     14448,  14706,  14964,  15222,  15480,  15738,  15996,  16254,
     16512,  16770,  17028,  17286,  17544,  17802,  18060,  18318,
     18576,  18834,  19092,  19350,  19608,  19866,  20124,  20382,
     20640,  20898,  21156,  21414,  21672,  21930,  22188,  22446,
     22704,  22962,  23220,  23478,  23736,  23994,  24252,  24510,
     24768,  25026,  25284,  25542,  25800,  26058,  26316,  26574,
     26832,  27090,  27348,  27606,  27864,  28122,  28380,  28638,
     28896,  29154,  29412,  29670,  29928,  30186,  30444,  30702,
     30960,  31218,  31476,  31734,  31992,  32250,  32508,  32766,
};

// 合成拍: 每次调用前进 2ms (账本按实际拍时长入账, 时序必须单调), 故逐拍入账的
//   断言可以直接按 2ms 写
static std::chrono::steady_clock::time_point next_tick() {
    static std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now();
    t = shift_ms(t, 2.0);
    return t;
}

int main() {
    std::cout << "[1] 8 位轴 → 16 位逻辑域 (均匀步长 258 + 溢出由端点吸收)\n";
    {
        // 实测器件 (G7 Pro): 四个摇杆轴 0..255, 静止原始值 128。
        //   规则: 中点 0x80 = 128 → 0, 步长 258 = floor(32767/127) (127 个正半程
        //   步长整体落在满偏内的最大整数), 最深一级 128×258 = 33024 越过 int16
        //   满偏由端点数吸收。
        CHECK(pad_axis_to_logical(128, 0, 255) == 0,
              "静止 (中点 0x80=128) → 精确 0 (几何中点 127.5 会给出 +129 的偏置)");
        CHECK(pad_axis_to_logical(0, 0, 255) == -PAD_AXIS_MAX
              && pad_axis_to_logical(1, 0, 255) == -32766
              && pad_axis_to_logical(255, 0, 255) == 32766,
              "最深一级溢出被端点吸收 (0 → −32767), 最高一级 +32766 (258×127)");
        // 黄金表逐值比对 (期望值独立写死, 不来自生产侧构造规则)
        int golden_bad = -1;
        for (int v = 0; v <= 255; ++v)
            if (pad_axis_to_logical(v, 0, 255) != GOLDEN_AXIS8[v]) { golden_bad = v; break; }
        CHECK(golden_bad < 0, "256 个码值逐值等于黄金表 (独立期望值)");
        // 256 个码值互不相同
        int distinct = 0;
        for (int v = 0; v <= 255; ++v) {
            bool seen = false;
            for (int u = 0; u < v; ++u)
                if (GOLDEN_AXIS8[u] == GOLDEN_AXIS8[v]) seen = true;
            if (!seen) ++distinct;
        }
        CHECK(distinct == 256, "256 个码值互不相同 (无映射塌缩)");
        // 步长直方图: 255 个相邻步里 254 个 = 258, 1 个 = 1 (最底 code0→code1)
        int n258 = 0, n1 = 0, other = 0;
        for (int v = 0; v < 255; ++v) {
            const int d = pad_axis_to_logical(v + 1, 0, 255) - pad_axis_to_logical(v, 0, 255);
            if (d == 258) ++n258;
            else if (d == 1) { ++n1; if (v != 0) ++other; }
            else ++other;
        }
        CHECK(n258 == 254 && n1 == 1 && other == 0,
              "步长直方图 {258: 254, 1: 1} 且唯一的小步落在 code0→code1 (端点吸收的代价只此一处)");
        bool mono = true;
        int prev = pad_axis_to_logical(0, 0, 255);
        for (int v = 1; v <= 255; ++v) {
            const int cur = pad_axis_to_logical(v, 0, 255);
            if (cur < prev) mono = false;
            prev = cur;
        }
        CHECK(mono, "0..255 全程单调不减");
        CHECK(pad_axis_to_logical(129, 0, 255) == 258
              && pad_axis_to_logical(127, 0, 255) == -258,
              "中点两侧一级 = ±258 (1 count 的物理级 = 0.003% 满偏)");
        CHECK(pad_axis_to_logical(-32767, -32767, 32767) == -PAD_AXIS_MAX
              && pad_axis_to_logical(12345, -32767, 32767) == 12345
              && pad_axis_to_logical(32767, -32767, 32767) == PAD_AXIS_MAX,
              "已居中的 ±32767 器件原值直通 (不查表; uinput e2e 的虚拟手柄即此族)");
        CHECK(pad_axis_to_logical(5000, 0, 255) == 32766
              && pad_axis_to_logical(-5, 0, 255) == -PAD_AXIS_MAX,
              "越界码值钳到端点码 (最高一级 +32766 / 最深一级 −32767): 查表即域, 不越 int16");
        CHECK(pad_axis_to_logical(7, 5, 5) == 0, "退化量程 (min=max) → 0");
        // 同一规则对无符号 10 位量程 (0..1023): 中点 512 → 0, 步长 = floor(32767/511) = 64
        CHECK(pad_axis_to_logical(512, 0, 1023) == 0
              && pad_axis_to_logical(0, 0, 1023) == -PAD_AXIS_MAX
              && pad_axis_to_logical(1023, 0, 1023) == 32704
              && pad_axis_to_logical(511, 0, 1023) == -64,
              "无符号 10 位量程: 中点 512 → 0, 步长 64 (最高一级 64×511 = 32704, "
              "最深一级溢出吸收为 −32767)");
        CHECK(pad_trig_to_logical(1023, 0, 1023) == 255
              && pad_trig_to_logical(512, 0, 1023) == 128
              && pad_trig_to_logical(256, 0, 1023) == 64
              && pad_trig_to_logical(0, 0, 1023) == 0,
              "扳机 0..1023 → 0..255 线性直映 (512 → 128, 四舍五入)");
        CHECK(pad_trig_to_logical(255, 0, 255) == 255 && pad_trig_to_logical(1, 0, 255) == 1,
              "8 位扳机原值直通");
    }

    std::cout << "[2] 注入换算与合并钳制 (逐轴有效满偏屏速)\n";
    {
        PadLogical h; h.rx = 100; h.lx = 100;
        PadLogical o = pad_merge(h, 1.5f, 0.0f, 3000.0f, 3000.0f, next_tick());
        CHECK(o.rx == 100 + 16384 && o.lx == 100,
              "1.5px/ms @3000px/s = 半偏 (16384), 人类通道直通");
        o = pad_merge(h, 3.0f, 0.0f, 3000.0f, 3000.0f, next_tick());
        CHECK(o.rx == 32767, "3px/ms @3000px/s = 满偏 (上限内)");
        o = pad_merge(h, 9.9f, 0.0f, 3000.0f, 3000.0f, next_tick());
        CHECK(o.rx == 32767, "超速注入 + 人类通道 → +满偏钳制");
        PadLogical hn; hn.rx = -100;
        o = pad_merge(hn, -9.9f, 0.0f, 3000.0f, 3000.0f, next_tick());
        CHECK(o.rx == -32767, "负向超速 + 人类通道 → −满偏钳制");
        o = pad_merge(h, 0.0f, 1.0f, 3000.0f, 1200.0f, next_tick());
        CHECK(o.ry == (int16_t)std::lround(1.0f * 1000.0f / 1200.0f * PAD_AXIS_MAX)
              && o.rx == 100,
              "两轴各按自己的有效满偏屏速换算 (y=1200 → 27306), x 轴不受扰");
        o = pad_merge(h, 0.0f, 1.0f, 3000.0f, 3000.0f, next_tick());
        CHECK(o.ry == 10922, "同一注入在 3000px/s 下 = 1/3 满偏 (10922)");
        o = pad_merge(h, 1.5f, 0.0f, 6000.0f, 3000.0f, next_tick());
        CHECK(o.rx == 100 + 8192, "有效满偏屏速加倍 → 同速度注入减半 (满偏比例反比)");
        // 线性满偏比例 d = v/A_eff 逐点成立 (与曲线模型无关)
        for (float v : {0.0f, 0.25f, 1.5f, 3.0f, 9.9f}) {
            const PadLogical q = pad_merge(h, v, 0.0f, 3000.0f, 3000.0f, next_tick());
            const int want = (int)std::lround((double)v * 1000.0 / 3000.0 * PAD_AXIS_MAX);
            int exp = 100 + want;
            const float m = std::hypot((float)exp, 0.0f);
            if (m > (float)PAD_AXIS_MAX) exp = (int)std::lround((float)exp * PAD_AXIS_MAX / m);
            CHECK(q.rx == exp, "注入 = 线性满偏比例 v/A_eff");
        }
        // 行程形状 = 圆 (实测驱动): 对角超速 → 径向收到满偏, 两轴各 ≈0.71 满偏
        PadLogical o2 = pad_merge(PadLogical{}, 3.0f, 3.0f, 3000.0f, 3000.0f, next_tick());
        CHECK(std::hypot((double)o2.rx, (double)o2.ry) <= 32767.0 + 1.0
              && o2.rx == o2.ry && std::abs(o2.rx - 23170) <= 1,
              "对角超速 → 径向收到满偏 (非逐轴满偏, 方向保持)");
        PadLogical hc; hc.rx = 23170; hc.ry = 23170;
        PadLogical o3 = pad_merge(hc, 1.5f, 1.5f, 3000.0f, 3000.0f, next_tick());
        CHECK(std::hypot((double)o3.rx, (double)o3.ry) <= 32767.0 + 1.0
              && std::abs(o3.rx - o3.ry) <= 1 && o3.rx >= 23170,
              "圆上人类 + 同向注入 → 径向压回圆内, 幅度不超满偏");
    }

    std::cout << "[3] 全透传 1:1\n";
    {
        PadLogical h; h.lx = -32767; h.ly = 12345; h.lt = 255; h.rt = 137;
        h.btns = PADBTN_A | PADBTN_START | PADBTN_DPAD_LEFT;
        PadLogical o = pad_merge(h, 0.0f, 0.0f, 3000.0f, 3000.0f, next_tick());
        CHECK(o.lx == h.lx && o.ly == h.ly && o.lt == h.lt && o.rt == h.rt
              && o.btns == h.btns, "按键/左摇杆/扳机逐位直通 (扳机模拟量无阈值)");
        CHECK(o.rx == 0 && o.ry == 0, "零注入 → 右摇杆 = 人类通道 (v=0 特例)");
    }

    std::cout << "[4] 摇杆账本 Σ(偏转·拍时长)\n";
    {
        PadLogical h; h.rx = 100;
        auto l0 = g_pad_ledger.cum();
        pad_merge(h, 1.5f, 0.0f, 3000.0f, 3000.0f, next_tick());    // rx=16484, h=2ms
        pad_merge(h, 3.0f, 0.0f, 3000.0f, 3000.0f, next_tick());    // rx=32767, h=2ms
        auto l1 = g_pad_ledger.cum();
        CHECK(l1.first - l0.first == (long long)(16484 + 32767) * 2
              && l1.second - l0.second == 0,
              "账本 = 合并偏转×实际拍时长 (偏转·ms), 未动轴不入账");
        // 账本 × s_rp = 该偏转的像素运动 (自身运动补偿口径 = 应用给游戏的那条命令)
        const float A = 3000.0f, h_ms = 2.0f;
        for (int16_t d : {(int16_t)9830, (int16_t)16384, (int16_t)22937, (int16_t)-19660}) {
            const double px_ledger = (double)d * (double)h_ms * (double)pad_s_rp_from_gain(A);
            const double px_true = (double)d / PAD_AXIS_MAX * A * (double)h_ms / 1000.0;
            CHECK(std::fabs(px_ledger - px_true) < 0.02 * std::fabs(px_true) + 0.05,
                  "账本 × s_rp = 该偏转的像素运动 (单位恒等式)");
        }
        auto l2 = g_pad_ledger.cum();
        PadLogical hc; hc.rx = 23170; hc.ry = 23170;
        PadLogical o = pad_merge(hc, 1.5f, 1.5f, 3000.0f, 3000.0f, next_tick());
        auto l3 = g_pad_ledger.cum();
        CHECK(l3.first - l2.first == (long long)o.rx * 2
              && l3.second - l2.second == (long long)o.ry * 2,
              "账本 = 径向钳制后的最终提交值 × 拍时长");
    }

    std::cout << "[5] 触发阈值与 -k 映射、注入门\n";
    {
        CHECK(pad_trig_thr_counts(0.0f) == 1 && pad_trig_thr_counts(100.0f) == 255
              && pad_trig_thr_counts(PAD_TRIG_THR_PCT) == 15
              && pad_trig_thr_counts(50.0f) == 128 && pad_trig_thr_counts(-5.0f) == 1,
              "阈值换算: 0%=1(任何非零) / 6%=15(=实测 flat 15) / 50%=128 / 100%=255 (越界钳制)");
        bool saved_aim = g_aim_enabled.load();
        int saved_mode = g_aim_mode.load();
        g_aim_enabled.store(true);
        g_aim_mode.store(0);
        float vx, vy; bool g;
        g = control_apply_pad(120, LEFT_KEY, vx, vy);
        CHECK(g, "fire 触发 (-k fire) → 门开");
        g = control_apply_pad(120, 0, vx, vy);
        CHECK(g, "松开后 KEEP_ALIVE 窗内门仍开");
        std::this_thread::sleep_for(std::chrono::milliseconds(KEEP_ALIVE_MS + 80));
        g = control_apply_pad(120, 0, vx, vy);
        CHECK(!g, "保持窗过后门关");
        g = control_apply_pad(120, RIGHT_KEY, vx, vy);
        CHECK(!g, "ads 触发在 -k fire 下门不开");
        g_aim_mode.store(1);
        g = control_apply_pad(120, RIGHT_KEY, vx, vy);
        CHECK(g, "-k ads 下 LT 触发门开");
        std::this_thread::sleep_for(std::chrono::milliseconds(KEEP_ALIVE_MS + 80));
        g = control_apply_pad(120, LEFT_KEY, vx, vy);
        CHECK(!g, "-k ads 下 RT 不开门");
        g_aim_mode.store(2);
        g = control_apply_pad(120, LEFT_KEY, vx, vy);
        CHECK(g, "-k both 下任一触发门开");
        CHECK(vx == 0.0f && vy == 0.0f, "无有效目标时期望速度为 0 (门开 ≠ 注入)");
        g_aim_enabled.store(false);
        g = control_apply_pad(120, LEFT_KEY, vx, vy);
        CHECK(!g, "接管关闭 → 门关 (纯透传)");
        g_aim_enabled.store(saved_aim);
        g_aim_mode.store(saved_mode);
    }

    std::cout << "[6] 逐轴速度帽 min(-x, A_eff/1000)\n";
    {
        own_motion_ledger_set(true);                 // pad 运行态 (账本来源 = 摇杆账本)
        const float saved_max = g_max_v.load();
        const int saved_x = g_spd_x.load(), saved_y = g_spd_y.load();
        g_max_v.store(2.0f);                         // -x 2000 px/s
        g_aim_enabled.store(true);
        g_aim_mode.store(2);
        const auto cap_cmd = [](int spd_x, int spd_y, float plx, float ply,
                                float& ox, float& oy) {
            g_spd_x.store(spd_x); g_spd_y.store(spd_y);
            { std::lock_guard<std::mutex> lk(g_target.mtx);
              g_target.px = plx; g_target.py = ply;
              g_target.vx = 0; g_target.vy = 0;
              g_target.ax_e = g_target.ay_e = 0;
              g_target.last_dt = PRED_DT0; g_target.last_alpha = PRED_ALPHA0;
              g_target.last_beta = PRED_BETA0; g_target.cs = 0;
              g_target.valid = true; g_target.t_pub = std::chrono::steady_clock::now(); }
            control_apply_pad(120, LEFT_KEY, ox, oy);               // -k both: RT 单独即开门
        };
        float ox = 0, oy = 0;
        cap_cmd(100, 100, -900.0f, -900.0f, ox, oy);
        CHECK(ox == -std::min(2.0f, GAIN_PAD_BASE / 1000.0f)
              && oy == -std::min(2.0f, GAIN_PAD_BASE / 1000.0f),
              "spd=100: 帽 = min(-x, 3000/1000) = -x (2.0 px/ms), 命令饱和后落在帽上");        // spd=500 → 有效满偏屏速 600 px/s = 0.6 px/ms < -x → 帽由有效增益给出
        cap_cmd(500, 500, -900.0f, -900.0f, ox, oy);
        CHECK(std::fabs(ox + 0.6f) < 1e-4f && std::fabs(oy + 0.6f) < 1e-4f,
              "spd=500: 帽 = A_eff/1000 = 0.6 px/ms (注入通道打满即该轴满偏行程)");
        cap_cmd(500, 100, -900.0f, -900.0f, ox, oy);
        CHECK(std::fabs(ox + 0.6f) < 1e-4f && std::fabs(oy + 2.0f) < 1e-4f,
              "逐轴独立: x 轴按 spdx 收帽, y 轴不受扰");
        { std::lock_guard<std::mutex> lk(g_target.mtx); g_target.valid = false; }
        g_max_v.store(saved_max);
        g_spd_x.store(saved_x); g_spd_y.store(saved_y);
        g_aim_enabled.store(true);
    }

    std::cout << "[7] own_motion_ledger 路由与 own_motion_scale\n";
    {
        g_spd_x.store(SPD_BASE); g_spd_y.store(SPD_BASE);
        g_ads_spd_x.store(SPD_BASE); g_ads_spd_y.store(SPD_BASE);
        own_motion_ledger_set(false);
        CHECK(&own_motion_ledger() == &g_counts, "hid 路由 = g_counts");
        LedgerPxScale hsc = own_motion_scale();
        CHECK(hsc.x == S_HID_BASE && hsc.y == S_HID_BASE,
              "hid 比例 = 该轴有效灵敏度 (spd=100 即基线, 两轴同值)");
        g_spd_y.store(200);                          // y 轴倍率: 有效 s 减半
        LedgerPxScale hsy = own_motion_scale();
        CHECK(hsy.x == S_HID_BASE && std::fabs(hsy.y - S_HID_BASE * 0.5f) < 1e-6f,
              "hid 的 y 轴比例按 spdy 单独缩放 (x 轴不受扰)");
        g_spd_y.store(SPD_BASE);
        own_motion_ledger_set(true);
        CHECK(&own_motion_ledger() == &g_pad_ledger, "pad 路由 = 摇杆账本");
        LedgerPxScale psc = own_motion_scale();
        CHECK(psc.x == pad_s_rp_from_gain(GAIN_PAD_BASE)
              && psc.y == pad_s_rp_from_gain(GAIN_PAD_BASE),
              "pad 比例 = 有效满偏屏速的账本换算 (与 hid 的 px/count 区分开)");
        g_spd_y.store(200);
        LedgerPxScale psy = own_motion_scale();
        CHECK(std::fabs(psy.y - pad_s_rp_from_gain(GAIN_PAD_BASE * 0.5f)) < 1e-9f
              && psy.x == psc.x,
              "pad 的 y 轴比例按 spdy 缩放 (增益与换算同源)");
        g_spd_y.store(SPD_BASE);
        g_ads_spd_x.store(SPD_BASE * 2);             // ADS 那一套: x 轴有效增益减半
        g_ads_down.store(true);
        LedgerPxScale asc = own_motion_scale();
        CHECK(std::fabs(asc.x - pad_s_rp_from_gain(GAIN_PAD_BASE * 0.5f)) < 1e-9f
              && asc.y == psc.y,
              "ADS 键按住时整套换成 adsspd 那一对 (逐轴)");
        g_ads_down.store(false);
        g_ads_spd_x.store(SPD_BASE);
        own_motion_ledger_set(false);
    }

    std::cout << "[8] 发布点 (输出后端接入契约)\n";
    {
        PadState in;
        { std::lock_guard<std::mutex> lk(in.mtx);
          in.st.lx = -5000; in.st.ly = 7000; in.st.rx = 900;
          in.st.btns = PADBTN_B | PADBTN_DPAD_UP; }
        uint64_t s0, s1, s2;
        pad_publish_snapshot(&s0);
        pad_tick(120, in, false);
        PadLogical p = pad_publish_snapshot(&s1);
        CHECK(s1 == s0 + 1, "pad_tick → seq 单调 +1");
        CHECK(p.lx == -5000 && p.ly == 7000 && p.rx == 900
              && p.btns == (PADBTN_B | PADBTN_DPAD_UP),
              "发布内容 = 合并逻辑态 (人类态 + 零注入, 逐字段)");
        pad_publish_snapshot(&s2);
        CHECK(s2 == s1, "无新拍 → seq 不变 (后端可跳过重发)");
    }

    std::cout << "[9] XInput 线格式 (20B 报告)\n";
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
        CHECK(r[3] == 0x07 + 0x40 + 0x80, "LB+RB+Guide+X+Y → byte3 = 0xC7 (保留 bit3 恒 0)");
        d.btns = PADBTN_TOUCH;
        pad_xinput_report(d, r);
        CHECK(r[2] == 0x00 && r[3] == 0x00,
              "触摸板位 (PS5 语义, 由分享键代位) 在上线报文里无落点 → 全零 (仅 p5g 有意义)");

        PadLogical t; t.lt = 255; t.rt = 137;
        pad_xinput_report(t, r);
        CHECK(r[4] == 255 && r[5] == 137, "扳机模拟量直映 (LT=255 / RT=137 原值)");

        PadLogical ax; ax.lx = 1234; ax.ly = 1234; ax.rx = -1; ax.ry = -32768;
        pad_xinput_report(ax, r);
        CHECK(r[6] == 0xD2 && r[7] == 0x04 && i16le(r + 6) == 1234,
              "LX int16 小端组装 (1234 → D2 04)");
        CHECK(r[8] == 0x2E && r[9] == 0xFB && i16le(r + 8) == -1234,
              "LY 线上上为正: 逻辑态上推 (+1234) → 线上 -1234 (口径取反)");
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

    std::cout << "[10] XInput 设备字节与刷新率推导\n";
    {
        const UsbRawDeviceDef& d = pad_xinput_usb_def();
        CHECK(d.device.idVendor == 0x045E && d.device.idProduct == 0x028E
              && d.device.bcdDevice == 0x0572 && d.device.bcdUSB == 0x0200,
              "身份 = 微软有线 360 手柄 (0x045E/0x028E, bcdDevice 0x0572, USB 2.0)");
        CHECK(d.device.bDeviceClass == 0xFF && d.device.bDeviceSubClass == 0xFF
              && d.device.bDeviceProtocol == 0xFF && d.device.bMaxPacketSize0 == 64
              && d.device.bNumConfigurations == 1,
              "厂商类三元组 + ep0 64B + 单配置 (单接口形态 → 宿主不经 usbccgp 复合)");
        CHECK(d.qualifier != nullptr && d.qualifier->bDescriptorType == USB_DT_DEVICE_QUALIFIER
              && d.qualifier->bMaxPacketSize0 == d.device.bMaxPacketSize0
              && d.qualifier->bNumConfigurations == d.device.bNumConfigurations
              && d.qualifier->bDeviceClass == d.device.bDeviceClass,
              "设备限定符 = 同一设备的全速运行形态 (高速设备必答, 字段与设备描述符一致)");
        CHECK(d.speed == USB_SPEED_HIGH,
              "高速枚举 (端点 bInterval 按 2^(bInterval−1) 微帧解释)");
        CHECK(d.config_len == 48 && d.config[2] == 48 && d.config[3] == 0
              && d.config[4] == 1 && d.config[5] == 1 && d.config[7] == 0x80,
              "配置节 48B 单接口单配置 / 总线供电 / wTotalLength 自洽");
        CHECK(d.config[8] == 0xFA, "bMaxPower = 0xFA → VBUS 请求 500mA");
        CHECK(d.config[18] == 0x10 && d.config[19] == 0x21 && d.config[24] == 0x81
              && d.config[25] == 0x14,
              "类特定 16B blob: bLength 0x10 / 类型 0x21 / IN 端点 0x81 (宿主枚举的必要形态)");
        CHECK(d.config[14] == 0xFF && d.config[15] == 0x5D && d.config[16] == 0x01
              && d.config[13] == 2,
              "接口 FF/5D/01 + 双端点 (XInput 绑定键)");
        CHECK(d.ep_in.bEndpointAddress == 0x81 && d.ep_in.wMaxPacketSize == 32
              && d.ep_in.bmAttributes == USB_ENDPOINT_XFER_INT && d.ep_in.bInterval == 4,
              "中断 IN 0x81 / 32B / bInterval=4 (高速 2^3 微帧 = 1ms = 1000Hz 轮询)");
        CHECK(d.has_ep_out && d.ep_out.bEndpointAddress == 0x02
              && d.ep_out.wMaxPacketSize == 32 && d.ep_out.bInterval == 8,
              "中断 OUT 0x02 / 32B / bInterval=8 (宿主 LED/力反馈命令的落点, 必须有人收)");
        // 终端刷新率 = 枚举速度 × bInterval 的合成 (与 usbraw_start 的打印同一算式)
        const int poll_hz = d.speed == USB_SPEED_FULL
                                ? 1000 / (int)d.ep_in.bInterval
                                : 8000 >> ((int)d.ep_in.bInterval - 1);
        CHECK(poll_hz == 1000 && DEFAULT_FREQ == 1000 && poll_hz >= DEFAULT_FREQ,
              "端点轮询上限 1000Hz ≥ 提交拍率 1000Hz (全速 + bInterval=1 同样声明 1000Hz, "
              "但宿主侧状态不跟随 — 见 io/pad_xinput.cu 的实测依据)");
        CHECK(PAD_XINPUT_REPORT_LEN == 20
              && PAD_XINPUT_REPORT_LEN <= d.ep_in.wMaxPacketSize,
              "20B 报告在单包内 (短包即包边界, EP_WRITE 长度 = 提交长度)");
        CHECK(d.report_desc == nullptr && d.report_desc_len == 0,
              "无 HID 报告描述符 (厂商接口非 HID)");
        CHECK(d.vendor_request == nullptr,
              "不设 vendor 钩子 → 一切 vendor 请求 STALL");
        CHECK(d.strings != nullptr && d.string_count == 3
              && !std::strcmp(d.strings[0].utf8, "GENERIC")
              && !std::strcmp(d.strings[1].utf8, "XINPUT CONTROLLER"),
              "字符串表 (厂商/产品取参考实现的中性值; 序列号在运行期由本机身份派生)");
        CHECK(d.strings[2].index == 3 && std::strlen(d.strings[2].utf8) == 12,
              "序列号 12 位 (本机身份 × 本设备定义的哈希, 派生规则见 io/pad_xinput.cu)");
        CHECK(d.rate_trace && d.rate_tag && !std::strcmp(d.rate_tag, "PAD-USB"),
              "报告率观测开启 (提交率与写完成率分开报)");
        // EP_ENABLE 描述符与配置节内的端点字节必须逐字节一致 (上线的两个视图)
        for (size_t off : {(size_t)34, (size_t)41}) {
            const usb_endpoint_descriptor& e = (off == 34) ? d.ep_in : d.ep_out;
            CHECK(d.config[off] == e.bLength && d.config[off + 1] == e.bDescriptorType
                  && d.config[off + 2] == e.bEndpointAddress
                  && d.config[off + 3] == e.bmAttributes
                  && d.config[off + 4] == (uint8_t)(e.wMaxPacketSize & 0xFF)
                  && d.config[off + 6] == e.bInterval,
                  "配置节端点字节与 EP_ENABLE 描述符一致");
        }
    }

    std::cout << "[11] 手柄未在位\n";
    {
        const std::string absent = "no_such_pad_substr_xyz";
        CHECK(find_pad_device(absent).empty(), "无匹配子串 → 空路径");
        CHECK(find_pad_device("/nonexistent_dev_node").empty(), "不存在的绝对路径 → 空路径");
        PadState pts;
        std::thread r(pad_reader_thread, absent, std::ref(pts));
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        CHECK(global_running.load(), "reader 未在位时不停机不阻塞");
        global_running = false;
        r.join();
        global_running = true;
    }

    std::cout << "[12] 附属接口键翻译与 8/16 位分辨率对照\n";
    {
        CHECK(pad_extra_key_bit(KEY_SYSRQ) == PADBTN_TOUCH,
              "KEY_SYSRQ (分享/上传键实测报在键盘接口) → 触摸板位");
        CHECK(pad_extra_key_bit(KEY_A) == 0 && pad_extra_key_bit(KEY_ENTER) == 0,
              "键盘接口其余键一概不关心 (不做通用键盘映射)");
        // 0.5% 满偏的小命令: 16 位域里是一个良分辨的数 (164 counts), 8 位域里只剩
        //   1.275 counts — 量化台阶与命令同量级 (砍到 1 即 −22%), 这正是注入必须
        //   落在 16 位域的原因 (合并/账本/补偿也全在同一域里)
        const float A = 3000.0f;                       // px/s
        const float v = 0.005f * A / 1000.0f;          // 满偏屏速的 0.5% = 0.015 px/ms
        const int d16 = (int)std::lround(pad_defl_ratio(v, A) * PAD_AXIS_MAX);
        const int d8 = (int)std::lround(pad_defl_ratio(v, A) * 255.0f);
        CHECK(d16 == 164 && d8 == 1,
              "0.5% 满偏命令: 16 位域 = 164 counts (台阶 0.003%), 8 位域只剩 1 count "
              "(台阶 0.39%, 量化误差 ≈22%)");
    }

    std::cout << (g_fail ? "FAILED" : "ALL PASS") << " (" << g_fail << " 失败)\n";
    return g_fail ? 1 : 0;
}
