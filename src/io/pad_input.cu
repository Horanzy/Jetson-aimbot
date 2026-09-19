// ============================================================================
//  pad_input.cu — pad_input.h 的实现: /dev/input/by-id 与 event* 名字扫描,
//    EVIOCGBIT/EVIOCGABS 能力实测 (轴族选择 / 摇杆量程与中心 / 扳机轴 / dpad
//    形态), evdev 事件 → 逻辑态翻译, 独占读取与掉线自愈 (清键位 + 1s 重试)。
//
//  实测依据 (GameSir-G7 Pro, hid-generic 绑定): ABS=X,Y,Z,RZ,GAS,BRAKE,HAT0X,
//    HAT0Y (位图 0x30627) — 四摇杆轴均 0..255 无符号, GAS=RT / BRAKE=LT 亦
//    0..255, dpad 走 HAT0 ±1; 按键 BTN_A..BTN_THUMBL/R (0x130–0x13E)。轴族
//    按能力位图逐设备判定, 不写死: 实体 G7 Pro 是 HID 手柄风格 (Z/RZ=右摇杆,
//    BRAKE/GAS=扳机), uinput e2e 的虚拟手柄刻意用 xpad 风格 (RX/RY=右摇杆,
//    Z/RZ=扳机) 覆盖另一族分支。
// ============================================================================

#include "io/pad_input.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "core/state.h"                     // global_running, DEV_SEARCH_PATH

namespace {

bool test_bit(int bit, const unsigned char* arr) {
    return arr[bit >> 3] & (1u << (bit & 7));
}
std::string lower_copy(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    return s;
}

// 内核 BTN_* → 逻辑位 (0 = 非手柄键)
int pad_key_bit(uint16_t code) {
    switch (code) {
        case BTN_A:       return PADBTN_A;
        case BTN_B:       return PADBTN_B;
        case BTN_X:       return PADBTN_X;      // BTN_NORTH
        case BTN_Y:       return PADBTN_Y;      // BTN_WEST
        case BTN_TL:      return PADBTN_LB;
        case BTN_TR:      return PADBTN_RB;
        case BTN_SELECT:  return PADBTN_BACK;
        case BTN_START:   return PADBTN_START;
        case BTN_MODE:    return PADBTN_GUIDE;
        case BTN_THUMBL:  return PADBTN_L3;
        case BTN_THUMBR:  return PADBTN_R3;
        case BTN_DPAD_UP: return PADBTN_DPAD_UP;
        case BTN_DPAD_DOWN:  return PADBTN_DPAD_DOWN;
        case BTN_DPAD_LEFT:  return PADBTN_DPAD_LEFT;
        case BTN_DPAD_RIGHT: return PADBTN_DPAD_RIGHT;
        default:          return 0;
    }
}

// 连接期实测的能力集 (轴族与量程来自 absinfo/key 位图, 不写死):
//   右摇杆两族布局并存 — xpad 风格 (RX/RY = 右摇杆, Z/RZ = 扳机) 与 HID 游戏
//   手柄风格 (Z/RZ = 右摇杆, BRAKE/GAS = 扳机; 实测 GameSir-G7 Pro 即此族),
//   按位图择一。
struct PadCaps {
    int rx_code=0, ry_code=0;                // 右摇杆轴 (RX/RY, 回落 Z/RZ)
    int lt_code=0, rt_code=0;                // 扳机轴 (xpad: Z/RZ; HID: BRAKE/GAS)
    int lt_min=0, lt_max=0, rt_min=0, rt_max=0;
    int mn[4]={0,0,0,0}, mx[4]={0,0,0,0};    // 摇杆量程: LX,LY,RX,RY 各轴实测
    bool hat0=false;                         // dpad 走 ABS_HAT0X/Y
    bool dpad_keys=false;                    // dpad 走 BTN_DPAD_* 按键
    bool joystick=false;                     // 左+右摇杆轴齐备 = 游戏手柄节点
};

// 摇杆原始值 → 逻辑 int16 全分辨率: 量程已居中 (min<0, xpad 风格 ±32767) =
//   原值 1:1 直通; 无符号 0..N 布局 (实测 GameSir-G7 Pro 四轴均 0..255) 按实测
//   absinfo 中心线性展开到 ±满偏 — 1:1 指物理偏转比例, 展开不引入任何阈值
//   (设备声明的 flat/fuzz 不代为施加, 静置残余偏转如实透传)
inline int16_t stick16(int v, int mn, int mx) {
    if (mx <= mn) return 0;
    if (mn < 0) return (int16_t)std::clamp(v, -PAD_AXIS_MAX, PAD_AXIS_MAX);
    float c = (mn + mx) / 2.0f;
    float f = (v >= c) ? (v - c) * PAD_AXIS_MAX / (mx - c)
                       : (v - c) * PAD_AXIS_MAX / (c - mn);
    return (int16_t)std::clamp((int)std::lround(f), -PAD_AXIS_MAX, PAD_AXIS_MAX);
}

PadCaps read_caps(int fd) {
    PadCaps c;
    unsigned char abits[(ABS_CNT + 7) / 8] = {}, kbits[(KEY_CNT + 7) / 8] = {};
    if (ioctl(fd, EVIOCGBIT(EV_ABS, sizeof(abits)), abits) < 0
        || ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(kbits)), kbits) < 0) return c;
    if (!test_bit(ABS_X,abits) || !test_bit(ABS_Y,abits)) return c;
    auto range=[&](int code, int& mn, int& mx) {
        input_absinfo ai{};
        if (ioctl(fd, EVIOCGABS(code), &ai) == 0) { mn=ai.minimum; mx=ai.maximum; } };
    range(ABS_X,c.mn[0],c.mx[0]);
    range(ABS_Y,c.mn[1],c.mx[1]);
    if (test_bit(ABS_RX,abits) && test_bit(ABS_RY,abits)) {
        c.rx_code=ABS_RX; c.ry_code=ABS_RY;
        c.lt_code = test_bit(ABS_Z,abits)?ABS_Z : test_bit(ABS_BRAKE,abits)?ABS_BRAKE : 0;
        c.rt_code = test_bit(ABS_RZ,abits)?ABS_RZ : test_bit(ABS_GAS,abits)?ABS_GAS : 0;
    } else if (test_bit(ABS_Z,abits) && test_bit(ABS_RZ,abits)) {
        c.rx_code=ABS_Z; c.ry_code=ABS_RZ;                 // HID 手柄风格: Z/RZ = 右摇杆
        c.lt_code = test_bit(ABS_BRAKE,abits)?ABS_BRAKE : 0;
        c.rt_code = test_bit(ABS_GAS,abits)?ABS_GAS : 0;
    } else return c;
    range(c.rx_code,c.mn[2],c.mx[2]);
    range(c.ry_code,c.mn[3],c.mx[3]);
    if (c.lt_code) range(c.lt_code,c.lt_min,c.lt_max);
    if (c.rt_code) range(c.rt_code,c.rt_min,c.rt_max);
    c.hat0 = test_bit(ABS_HAT0X,abits) && test_bit(ABS_HAT0Y,abits);
    c.dpad_keys = test_bit(BTN_DPAD_UP,kbits) && test_bit(BTN_DPAD_DOWN,kbits);
    c.joystick = true;
    return c;
}

std::string caps_line(const PadCaps& c) {
    auto ax=[&](int code){ return code==ABS_RX?"ABS_RX":code==ABS_RY?"ABS_RY"
        :code==ABS_Z?"ABS_Z":code==ABS_RZ?"ABS_RZ":code==ABS_BRAKE?"ABS_BRAKE"
        :code==ABS_GAS?"ABS_GAS":"-"; };
    char buf[256];
    snprintf(buf,sizeof(buf),
             "L %d..%d R %d..%d (%s/%s), LT=%s %d..%d, RT=%s %d..%d, dpad=%s",
             c.mn[0],c.mx[0],c.mn[2],c.mx[2], ax(c.rx_code),ax(c.ry_code),
             ax(c.lt_code),c.lt_min,c.lt_max, ax(c.rt_code),c.rt_min,c.rt_max,
             c.hat0?"hat0":c.dpad_keys?"keys":"none");
    return buf;
}

// 扳机行程 → 0–255 逻辑量 (实测 [min,max] 线性直映, 无阈值)
inline uint8_t trig255(int v, int mn, int mx) {
    if (mx <= mn) return 0;
    float f = (float)(v - mn) * 255.0f / (float)(mx - mn);
    return (uint8_t)std::clamp((int)std::lround(f), 0, 255);
}

bool open_grab(const std::string& dev, int& fd, PadCaps& c) {
    fd = open(dev.c_str(), O_RDONLY);
    if (fd < 0) return false;
    c = read_caps(fd);
    if (!c.joystick) { close(fd); fd = -1; return false; }
    if (ioctl(fd, EVIOCGRAB, 1) < 0) std::cerr<<"⚠ 手柄无法独占 (可能有其他读取者)\n";
    return true;
}

} // namespace

PadLogical pad_input_snapshot(PadState& st) {
    std::lock_guard<std::mutex> lk(st.mtx);
    return st.st;
}

std::string find_pad_device(const std::string& kw, bool verbose) {
    if (!kw.empty() && kw.front()=='/') return access(kw.c_str(),R_OK)==0?kw:"";
    const std::string low = lower_copy(kw);
    std::vector<std::string> hits;
    // by-id: 稳定 USB 节点; -event-joystick 后缀排除 if01 kbd/mouse 附属接口
    if (DIR* dp = opendir(DEV_SEARCH_PATH)) {
        const std::string suf = "-event-joystick";
        while (dirent* e = readdir(dp)) {
            std::string nm = e->d_name;
            if (nm.size() <= suf.size()
                || nm.compare(nm.size()-suf.size(), suf.size(), suf) != 0) continue;
            if (lower_copy(nm).find(low) != std::string::npos)
                hits.push_back(std::string(DEV_SEARCH_PATH) + nm);
        }
        closedir(dp);
    }
    if (hits.empty()) {
        // 回落: 按设备名 + 摇杆能力匹配 (uinput 虚拟手柄/蓝牙手柄无 by-id 节点;
        //   能力核对天然排除同名 kbd/mouse 附属接口)
        if (DIR* dp = opendir("/dev/input")) {
            while (dirent* e = readdir(dp)) {
                std::string nm = e->d_name;
                if (nm.rfind("event",0) != 0) continue;
                int fd = open((std::string("/dev/input/") + nm).c_str(), O_RDONLY);
                if (fd < 0) continue;
                char name[128] = {};
                ioctl(fd, EVIOCGNAME(sizeof(name)-1), name);
                bool name_hit = low.empty() || lower_copy(name).find(low) != std::string::npos;
                bool is_pad = name_hit && read_caps(fd).joystick;
                close(fd);
                if (is_pad) hits.push_back("/dev/input/" + nm);
            }
            closedir(dp);
        }
    }
    if (verbose && hits.size() != 1) {
        std::cerr<<"❌ 手柄 \""<<kw<<"\" "<<(hits.empty()?"没有匹配":"匹配到多个")<<"\n";
        for (auto& h : hits) std::cerr<<"   "<<h<<"\n";
    }
    return hits.size() == 1 ? hits[0] : "";
}

void pad_reader_thread(const std::string& kw, PadState& st) {
    bool logged_absent = false;
    while (global_running) {
        std::string dev = find_pad_device(kw);
        int fd = -1; PadCaps c;
        if (dev.empty() || !open_grab(dev, fd, c)) {
            if (!logged_absent) {                       // 缺席只记一次, 重试静默
                std::cout<<"⚠ 手柄未找到 (-P '"<<kw<<"'), 每秒重试, 不阻塞启动\n";
                logged_absent = true; }
            for (int i=0; i<10 && global_running; ++i) usleep(100'000);
            continue;
        }
        logged_absent = false;
        std::cout<<"✅ 手柄: "<<dev<<" ("<<caps_line(c)<<")\n";
        struct pollfd pfd{}; pfd.fd=fd; pfd.events=POLLIN;
        struct input_event ev;
        bool dead = false;
        while (global_running && !dead) {
            int pr = poll(&pfd,1,100);
            if (pr < 0) { if (errno==EINTR) continue; dead = true; break; }
            if (pr == 0) continue;
            if (pfd.revents & (POLLHUP|POLLERR)) { dead = true; break; }
            ssize_t n = read(fd,&ev,sizeof(ev));
            if (n == (ssize_t)sizeof(ev)) {
                std::lock_guard<std::mutex> lk(st.mtx);
                PadLogical& s = st.st;
                if (ev.type == EV_ABS) {
                    switch (ev.code) {
                        case ABS_X:  s.lx=stick16(ev.value,c.mn[0],c.mx[0]); break;
                        case ABS_Y:  s.ly=stick16(ev.value,c.mn[1],c.mx[1]); break;
                        default:
                            if (ev.code==c.rx_code) s.rx=stick16(ev.value,c.mn[2],c.mx[2]);
                            else if (ev.code==c.ry_code) s.ry=stick16(ev.value,c.mn[3],c.mx[3]);
                            else if (ev.code==c.lt_code) s.lt=trig255(ev.value,c.lt_min,c.lt_max);
                            else if (ev.code==c.rt_code) s.rt=trig255(ev.value,c.rt_min,c.rt_max);
                            else if (c.hat0 && ev.code==ABS_HAT0X) {
                                if (ev.value<0) s.btns|=PADBTN_DPAD_LEFT; else s.btns&=~PADBTN_DPAD_LEFT;
                                if (ev.value>0) s.btns|=PADBTN_DPAD_RIGHT; else s.btns&=~PADBTN_DPAD_RIGHT; }
                            else if (c.hat0 && ev.code==ABS_HAT0Y) {
                                if (ev.value<0) s.btns|=PADBTN_DPAD_UP; else s.btns&=~PADBTN_DPAD_UP;
                                if (ev.value>0) s.btns|=PADBTN_DPAD_DOWN; else s.btns&=~PADBTN_DPAD_DOWN; }
                    }
                } else if (ev.type == EV_KEY) {
                    int bit = pad_key_bit(ev.code);
                    if (bit) { if (ev.value) s.btns |= (uint16_t)bit;
                               else           s.btns &= (uint16_t)~bit; }
                }
            } else if (n <= 0 && errno!=EINTR && errno!=EAGAIN) dead = true;
        }
        close(fd);
        if (!global_running) break;
        { std::lock_guard<std::mutex> lk(st.mtx); st.st = PadLogical{}; }   // 清键位防卡键
        std::cerr<<"⚠ 手柄断开, 已清键位, 1s 重试\n";
        for (int i=0; i<10 && global_running; ++i) usleep(100'000);
    }
}
