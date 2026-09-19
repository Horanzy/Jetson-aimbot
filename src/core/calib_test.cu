// ============================================================================
//  calib_test — build/calib_test 单测 (scripts/compile.sh 构建并执行): 标定
//    拍数常数的墙钟语义 — 触发/超时/激励表每段的 拍数×TICK_MS 必须还原设计
//    时长, 每边位移必须还原设计 counts (换拍率不改标定物理轨迹)。全部为
//    calib.h 头常数断言, 无需链接任何模块对象。
//  全部断言通过输出 ALL PASS 并返回 0。
// ============================================================================

#include <cmath>
#include <iostream>

#include "core/calib.h"
#include "core/state.h"

static int g_fail = 0;

#define CHECK(cond, msg) do { \
    if (cond) { std::cout << "  ok  " << msg << "\n"; } \
    else { std::cerr << "  FAIL " << msg << "\n"; ++g_fail; } \
} while (0)

static double seg_ms(const CalibSeg& s) { return s.ticks * (double)TICK_MS; }

int main() {
    std::cout << "[1] 触发/超时墙钟\n";
    CHECK(CALIB_TRIGGER_TICKS * (double)TICK_MS == 5000.0,
          "双侧键长按触发 = 5000ms 墙钟");
    CHECK(CALIB_WAIT_TIMEOUT * (double)TICK_MS == 2000.0,
          "等待计算超时 = 2000ms 墙钟");

    std::cout << "[2] 起始十字 (纯视觉信号, 不参与采样)\n";
    CHECK((int)(sizeof(CAL_START_SEQ) / sizeof(CalibSeg)) == 9, "9 段 (8 腿 + 停顿)");
    CHECK(CAL_START_SEQ[0].dx == 2 && seg_ms(CAL_START_SEQ[0]) == 240.0,
          "2px/拍, 240ms/腿");
    CHECK(CAL_START_SEQ[0].dx * CAL_START_SEQ[0].ticks == 480, "每腿 480 counts");
    CHECK(CAL_START_SEQ[8].dx == 0 && CAL_START_SEQ[8].dy == 0
          && seg_ms(CAL_START_SEQ[8]) == 500.0, "收尾停顿 500ms");
    {   // 腿序 +x,−x,−x,+x ∈ 每轴 4 腿: 行程以起点为中心 ±480, 每圈回到起点
        long x = 0, y = 0, mxx = 0, mnx = 0, mxy = 0, mny = 0;
        for (int i = 0; i < 8; ++i) {
            x += (long)CAL_START_SEQ[i].dx * CAL_START_SEQ[i].ticks;
            y += (long)CAL_START_SEQ[i].dy * CAL_START_SEQ[i].ticks;
            mxx = std::max(mxx, x); mnx = std::min(mnx, x);
            mxy = std::max(mxy, y); mny = std::min(mny, y);
        }
        CHECK(mxx == 480 && mnx == -480 && mxy == 480 && mny == -480 && x == 0 && y == 0,
              "起始十字对称于起点 (±480 两侧都有) 且净位移 0 — 老方波是 0..+480 单侧");
    }

    std::cout << "[3] 激励十字 + 腿间停顿 (采样段: 时长/速度/每腿位移为设计量)\n";
    CHECK((int)(sizeof(CAL_EXCITE_SEQ) / sizeof(CalibSeg)) == 16,
          "单圈 16 段 (8 腿 + 每腿后的零指令停顿)");
    for (int j = 0; j < 16; ++j) {
        const CalibSeg& s = CAL_EXCITE_SEQ[j];
        if (j % 2 == 0) {
            CHECK(std::abs(s.dx) + std::abs(s.dy) == 2 && seg_ms(s) == 250.0,
                  "2px/拍, 250ms/腿 (2000 counts/s)");
            CHECK((std::abs(s.dx) + std::abs(s.dy)) * s.ticks == 500, "每腿 500 counts");
        } else {
            CHECK(s.dx == 0 && s.dy == 0 && seg_ms(s) == 150.0,
                  "腿间零指令停顿 150ms (与 pad 的 PAUSE 同源: L 上界 100ms + 余量)");
        }
    }
    {   // 每圈回到起点, 每轴包络 ±500 (对称), 16 个边沿
        long x = 0, y = 0, mxx = 0, mnx = 0, mxy = 0, mny = 0;
        for (int i = 0; i < 16; ++i) {
            x += (long)CAL_EXCITE_SEQ[i].dx * CAL_EXCITE_SEQ[i].ticks;
            y += (long)CAL_EXCITE_SEQ[i].dy * CAL_EXCITE_SEQ[i].ticks;
            mxx = std::max(mxx, x); mnx = std::min(mnx, x);
            mxy = std::max(mxy, y); mny = std::min(mny, y);
        }
        CHECK(mxx == 500 && mnx == -500 && mxy == 500 && mny == -500 && x == 0 && y == 0,
              "激励单圈行程对称于起点 ±500 counts 且回到起点 (画面内容相似→相关更稳)");
        int edges = 0, cx = 0, cy = 0;
        for (int i = 0; i < 16; ++i) {
            if (CAL_EXCITE_SEQ[i].dx != cx || CAL_EXCITE_SEQ[i].dy != cy) ++edges;
            cx = CAL_EXCITE_SEQ[i].dx; cy = CAL_EXCITE_SEQ[i].dy;
        }
        CHECK(edges == 16, "每圈 16 个指令边沿 (老方波 4 个) → lag 对齐的辨识更强");
        CHECK(CAL_EXCITE_LOOPS == 3
              && CAL_EXCITE_LOOPS * 16 * 400 == 19200,
              "3 圈 × 16 段 × 400ms = 9.6s 激励 (24 条腿, 多于老方波的 5 圈 × 4 = 20)");
    }

    std::cout << "[4] 静置与收尾甩动\n";
    CHECK(seg_ms(CAL_SETTLE_SEQ[0]) == 300.0, "静置 300ms");
    CHECK((int)(sizeof(CAL_END_OK_SEQ) / sizeof(CalibSeg)) == 6
          && (int)(sizeof(CAL_END_FAIL_SEQ) / sizeof(CalibSeg)) == 6, "甩动各 6 程");
    CHECK(CAL_END_OK_SEQ[0].dy == 4 && CAL_END_OK_SEQ[0].dx == 0
          && seg_ms(CAL_END_OK_SEQ[0]) == 60.0, "成功 = 纵向点头 4px/拍, 60ms/程");
    CHECK(CAL_END_OK_SEQ[0].dy * CAL_END_OK_SEQ[0].ticks == 240, "每程 240 counts");
    CHECK(CAL_END_FAIL_SEQ[0].dx == 4 && CAL_END_FAIL_SEQ[0].dy == 0
          && seg_ms(CAL_END_FAIL_SEQ[0]) == 60.0, "失败 = 横向摇头 4px/拍, 60ms/程");

    std::cout << (g_fail ? "FAILED\n" : "ALL PASS\n");
    return g_fail ? 1 : 0;
}
