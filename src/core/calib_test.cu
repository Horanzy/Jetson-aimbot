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

    std::cout << "[2] 起始方块 (纯视觉信号, 不参与采样)\n";
    CHECK((int)(sizeof(CAL_START_SEQ) / sizeof(CalibSeg)) == 5, "5 段 (4 边 + 停顿)");
    CHECK(CAL_START_SEQ[0].dx == 2 && seg_ms(CAL_START_SEQ[0]) == 240.0,
          "2px/拍, 240ms/边");
    CHECK(CAL_START_SEQ[0].dx * CAL_START_SEQ[0].ticks == 480, "每边 480 counts");
    CHECK(CAL_START_SEQ[4].dx == 0 && CAL_START_SEQ[4].dy == 0
          && seg_ms(CAL_START_SEQ[4]) == 500.0, "收尾停顿 500ms");

    std::cout << "[3] 激励方波 (采样段: 段时长/速度/每边位移为设计量)\n";
    CHECK((int)(sizeof(CAL_EXCITE_SEQ) / sizeof(CalibSeg)) == 4, "单圈 4 边");
    for (int j = 0; j < 4; ++j) {
        const CalibSeg& s = CAL_EXCITE_SEQ[j];
        CHECK(std::abs(s.dx) + std::abs(s.dy) == 2 && seg_ms(s) == 250.0,
              "2px/拍, 250ms/边 (2000 counts/s)");
        CHECK((std::abs(s.dx) + std::abs(s.dy)) * s.ticks == 500, "每边 500 counts");
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
