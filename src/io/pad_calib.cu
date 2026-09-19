// ============================================================================
//  pad_calib.cu — pad_calib.h 的实现: L3+R3 长按 / 热参请求触发的标定状态机
//    (相位与 hid 标定同形, 状态自成一态), 激励轨迹按满偏转播放, 采样/计算/
//    回写经 g_calib_collect/g_calib_request/g_calib_done 三原子与 io/capture.cu
//    的 ai_thread 交接 (与 hid 标定共用这条链路)。
// ============================================================================

#include "io/pad_calib.h"

#include <iostream>
#include <vector>

#include "core/state.h"

namespace {

// 标定相位 (与 hid 状态机的 cal=0..6 同构; 状态机是本模块私有的单个 static,
//   与 hid 的状态机只共享三个交接原子)
enum PadCalPhase {
    PC_IDLE = 0,      // 空闲: L3+R3 长按 / 热参请求计数
    PC_START,         // 起始方块 (纯视觉开始信号, 不采样)
    PC_EXCITE,        // 激励方波 (g_calib_collect 开, 采样中)
    PC_WAIT,          // 等 ai_thread 拟合 (g_calib_done; 摇杆静置)
    PC_END_OK,        // 收尾: 成功点头
    PC_END_FAIL,      // 收尾: 失败摇头
    PC_SETTLE,        // 激励后静置 (采样窗收尾)
};

struct PadCalState {
    PadCalPhase phase = PC_IDLE;
    int hold = 0;                        // L3+R3 长按计数 (拍)
    int wt = 0;                          // 等计算超时计数 (拍)
    const CalibSeg* seq = nullptr;
    int slen = 0, si = 0, st = 0;
    std::vector<CalibSeg> excite;        // 激励方波展开 (5 圈)
};

template <size_t N>
void enter(PadCalState& s, const CalibSeg (&seq)[N], PadCalPhase ph) {
    s.seq = seq; s.slen = (int)N; s.si = s.st = 0; s.phase = ph;
}

} // namespace

PadCalibStep pad_calib_step(uint16_t btns) {
    static PadCalState s;
    const bool req_raw = g_padcalib_request.exchange(false);
    bool req = req_raw;
    PadCalibStep out{false, 0, 0};

    if (req && s.phase != PC_IDLE) {           // 请求在标定进行中到达: 一次消费即清
        std::cout << "[标定] 忽略请求: 标定进行中\n";
        req = false;
    }

    if (!g_aim_enabled.load()) {
        // 接管关闭 = 纯透传: 激励是程序注入的移动, 与透传互斥 — 进行中的标定
        //   复位, 触发不可达 (与 hid 标定分支同一纪律)
        if (s.phase != PC_IDLE) {
            s.phase = PC_IDLE; g_calib_collect = false;
            s.seq = nullptr; s.slen = s.si = s.st = 0;
            std::cout << "[标定] 中断: 接管关闭 (纯透传)\n";
        }
        s.hold = 0;
        if (req) std::cout << "[标定] 忽略请求: 接管关闭 (纯透传)\n";
        return out;
    }

    if (s.phase == PC_IDLE) {
        const bool both = (btns & (PADBTN_L3 | PADBTN_R3)) == (PADBTN_L3 | PADBTN_R3);
        if (both) ++s.hold; else s.hold = 0;
        if (req || s.hold >= PAD_CAL_TRIGGER_TICKS) {
            s.hold = 0;
            std::cout << "[标定] 触发\n";
            enter(s, PAD_CAL_START_SEQ, PC_START);
            out.active = true;        // 触发拍不播激励 (激励自下一拍起, 与 hid 同形)
            return out;
        }
    }

    if (s.phase == PC_WAIT) {
        out.active = true;                       // 等计算期摇杆静置 (dx=dy=0)
        int done = g_calib_done.load();
        if (done != 0 || ++s.wt > CALIB_WAIT_TIMEOUT)
            enter(s, done == 1 ? PAD_CAL_END_OK_SEQ : PAD_CAL_END_FAIL_SEQ,
                  done == 1 ? PC_END_OK : PC_END_FAIL);
        return out;
    }

    if (s.phase != PC_IDLE) {
        out.active = true;
        if (s.si < s.slen) {
            out.dx = (int16_t)s.seq[s.si].dx; out.dy = (int16_t)s.seq[s.si].dy;
            if (++s.st >= s.seq[s.si].ticks) { s.st = 0; ++s.si; }
        }
        if (s.si >= s.slen) {
            if (s.phase == PC_START) {
                s.excite.clear();
                constexpr int nseg = (int)(sizeof(PAD_CAL_EXCITE_SEQ) / sizeof(CalibSeg));
                for (int i = 0; i < 5; ++i)
                    for (int j = 0; j < nseg; ++j) s.excite.push_back(PAD_CAL_EXCITE_SEQ[j]);
                s.seq = s.excite.data(); s.slen = (int)s.excite.size(); s.si = s.st = 0;
                g_calib_collect = true; s.phase = PC_EXCITE;
            } else if (s.phase == PC_EXCITE) {
                enter(s, PAD_CAL_SETTLE_SEQ, PC_SETTLE);   // 静置 (采集仍开, 样本收尾)
            } else if (s.phase == PC_SETTLE) {
                g_calib_collect = false; g_calib_done = 0; g_calib_request = true;
                s.wt = 0; s.phase = PC_WAIT;
            } else {
                s.phase = PC_IDLE; s.seq = nullptr; s.slen = s.si = s.st = 0;
            }
        }
    }
    return out;
}
