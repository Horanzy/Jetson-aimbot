// ============================================================================
//  pad_input.cu — pad_input.h 的实现: /dev/input/by-id 与 event* 名字扫描,
//    EVIOCGBIT/EVIOCGABS 能力实测 (轴族选择 / 摇杆量程与中点 / 扳机轴 / dpad
//    形态), evdev 事件 → 逻辑态翻译, 独占读取与掉线自愈 (清键位 + 1s 重试)。
//
//  实测依据 (GameSir G7 Pro, hid-generic 绑定): ABS=X,Y,Z,RZ,GAS,BRAKE,HAT0X,
//    HAT0Y — 四摇杆轴均 0..255 无符号, GAS=RT / BRAKE=LT 亦 0..255, dpad 走
//    HAT0 ±1; 按键 BTN_A..BTN_THUMBL/R (0x130–0x13E)。轴族按能力位图逐设备判定,
//    不写死: 实体 G7 Pro 是 HID 手柄风格 (Z/RZ = 右摇杆, BRAKE/GAS = 扳机),
//    uinput e2e 的虚拟手柄刻意用 xpad 风格 (RX/RY = 右摇杆, Z/RZ = 扳机) 覆盖
//    另一族分支。
// ============================================================================

#include "io/pad_input.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <climits>                          // PATH_MAX (兄弟节点按 realpath 判同)
#include <cstdio>
#include <cstdlib>                          // realpath
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
// 加密狗节点排除的判据: EVIOCGID 的实测 VID:PID (by-id 名字是
//   usb-<厂商>_<产品>-event-joystick 形态, 不含 VID:PID 十六进制, 故名字匹配
//   对它无效 — 两条候选路径都把 fd 传进来)
bool is_dongle_node(int fd) {
    input_id id{};
    if (fd >= 0 && ioctl(fd, EVIOCGID, &id) == 0)
        return id.vendor == P5G_DONGLE_VID && id.product == P5G_DONGLE_PID;
    return false;
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

// 同一物理设备的 event 节点扫描 (判据 = EVIOCGID 的 VID:PID 与摇杆节点相同; 与
//   轴族选择同一纪律: 实测标识, 不写死节点名/接口编号)。实测依据 (GameSir
//   G7 Pro): 一个 USB 设备下三个 input 节点 — 摇杆 (js1/event2)、键盘 (event3)、
//   鼠标 (event4); 分享/上传键只在**键盘接口**上出现, 摇杆节点全程无事件 →
//   只读摇杆节点会漏掉它。
std::vector<std::string> find_sibling_nodes(uint16_t vid, uint16_t pid,
                                            const std::string& primary) {
    std::vector<std::string> out;
    // 摇杆节点本身要排除: 路径可能一个是 by-id 软链一个是 /dev/input/eventN,
    //   故按 realpath 判同一节点 (否则同一节点被开两次: 事件处理两遍 + 一条
    //   无意义的"无法独占"告警)
    char pr[PATH_MAX];
    const std::string prim = realpath(primary.c_str(), pr) ? std::string(pr) : primary;
    if (DIR* dp = opendir("/dev/input")) {
        while (dirent* e = readdir(dp)) {
            const std::string nm = e->d_name;
            if (nm.rfind("event", 0) != 0) continue;
            const std::string path = "/dev/input/" + nm;
            char cr[PATH_MAX];
            const std::string cand = realpath(path.c_str(), cr) ? std::string(cr) : path;
            if (cand == prim) continue;
            int fd = open(path.c_str(), O_RDONLY);
            if (fd < 0) continue;
            input_id id{};
            const bool same = ioctl(fd, EVIOCGID, &id) == 0
                           && id.vendor == vid && id.product == pid;
            close(fd);
            if (same) out.push_back(path);
        }
        closedir(dp);
    }
    std::sort(out.begin(), out.end());            // 稳定顺序 (日志与复现一致)
    return out;
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

bool open_grab(const std::string& dev, int& fd, PadCaps& c) {
    fd = open(dev.c_str(), O_RDONLY);
    if (fd < 0) return false;
    c = read_caps(fd);
    if (!c.joystick) { close(fd); fd = -1; return false; }
    if (ioctl(fd, EVIOCGRAB, 1) < 0) std::cerr<<"⚠ 手柄无法独占 (可能有其他读取者)\n";
    return true;
}

// 兄弟节点打开 (无能力要求: 键盘/鼠标接口没有摇杆轴); 独占失败只提示一次
bool open_grab_plain(const std::string& dev, int& fd) {
    fd = open(dev.c_str(), O_RDONLY);
    if (fd < 0) return false;
    if (ioctl(fd, EVIOCGRAB, 1) < 0) {
        static bool warned = false;
        if (!warned) { warned = true;
            std::cerr<<"⚠ 手柄附属接口无法独占 (可能有其他读取者): "<<dev<<"\n"; }
    }
    return true;
}

} // namespace

uint16_t pad_extra_key_bit(uint16_t code) {
    switch (code) {
        // 实测 (GameSir G7 Pro 的"分享/上传"键): 摇杆节点无事件, **键盘接口**报
        //   KEY_SYSRQ (99 = PrintScreen — 手柄厂把截图/分享类键映射到它的常见
        //   做法), 并带自动重复 (value=2)。逻辑态是电平位, 自动重复无害 (只认
        //   0 = 抬起)。触摸板按下是 PS5 侧语义, 物理手柄没有这个键, 故由它代位。
        case KEY_SYSRQ: return PADBTN_TOUCH;
        default: return 0;                        // 键盘接口其余键一概不关心
    }
}

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
            if (lower_copy(nm).find(low) == std::string::npos) continue;
            // 打开只为读设备 ID (EVIOCGID): 加密狗排除的判据见 is_dongle_node。
            //   打不开的候选只静默跳过 (扫描每秒一次, 逐次报错会刷屏), 但首次
            //   失败如实提示一次 — 非 root 运行时 EACCES 会被误读成"手柄没插"。
            int fd = open((std::string(DEV_SEARCH_PATH) + nm).c_str(), O_RDONLY);
            if (fd < 0) {
                static bool open_warned = false;
                if (!open_warned && errno == EACCES) {
                    open_warned = true;
                    std::cerr << "⚠ 手柄节点 " << nm << " 不可读 (权限) — 以 sudo 运行, "
                                 "或补 /dev/input 读取权限\n";
                }
                continue;
            }
            bool dongle = is_dongle_node(fd);
            close(fd);
            if (!dongle) hits.push_back(std::string(DEV_SEARCH_PATH) + nm);
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
                bool is_pad = name_hit && !is_dongle_node(fd) && read_caps(fd).joystick;
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
        // 兄弟接口 (同 VID:PID 的其他 event 节点): 额外键在这里 (分享/上传键实测
        //   走键盘接口), 只消费 EV_KEY 并经 pad_extra_key_bit 翻译 — 摇杆节点的
        //   语义不变
        std::vector<int> xfd;
        std::vector<std::string> xname;
        {
            input_id id{};
            if (ioctl(fd, EVIOCGID, &id) == 0)
                for (const std::string& xd : find_sibling_nodes(id.vendor, id.product, dev)) {
                    int xf = -1;
                    if (open_grab_plain(xd, xf)) { xfd.push_back(xf); xname.push_back(xd); }
                }
            if (!xfd.empty()) {
                std::cout<<"   附属接口: ";
                for (size_t i=0;i<xname.size();++i) std::cout<<(i?", ":"")<<xname[i];
                std::cout<<" (只取额外键)\n";
            }
        }
        std::vector<pollfd> pfds(1 + xfd.size());
        for (size_t i=0;i<pfds.size();++i) { pfds[i].fd = i==0? fd : xfd[i-1];
                                             pfds[i].events = POLLIN; }
        struct input_event ev;
        bool dead = false;
        while (global_running && !dead) {
            int pr = poll(pfds.data(), pfds.size(), 100);
            if (pr < 0) { if (errno==EINTR) continue; dead = true; break; }
            if (pr == 0) continue;
            for (size_t i=0;i<pfds.size();++i) {
                if (!(pfds[i].revents & (POLLIN|POLLHUP|POLLERR))) continue;
                if (pfds[i].revents & (POLLHUP|POLLERR)) { dead = true; break; }
                ssize_t n = read(pfds[i].fd,&ev,sizeof(ev));
                if (n == (ssize_t)sizeof(ev)) {
                    std::lock_guard<std::mutex> lk(st.mtx);
                    PadLogical& s = st.st;
                    if (i > 0) {                         // 兄弟接口: 只认 pad_extra_key_bit
                        if (ev.type == EV_KEY) {
                            const uint16_t bit = pad_extra_key_bit(ev.code);
                            if (bit) { if (ev.value) s.btns |= bit;
                                       else           s.btns &= (uint16_t)~bit; }
                        }
                        continue;
                    }
                    if (ev.type == EV_ABS) {
                        switch (ev.code) {
                            case ABS_X:  s.lx=pad_axis_to_logical(ev.value,c.mn[0],c.mx[0]); break;
                            case ABS_Y:  s.ly=pad_axis_to_logical(ev.value,c.mn[1],c.mx[1]); break;
                            default:
                                if (ev.code==c.rx_code) s.rx=pad_axis_to_logical(ev.value,c.mn[2],c.mx[2]);
                                else if (ev.code==c.ry_code) s.ry=pad_axis_to_logical(ev.value,c.mn[3],c.mx[3]);
                                else if (ev.code==c.lt_code) s.lt=pad_trig_to_logical(ev.value,c.lt_min,c.lt_max);
                                else if (ev.code==c.rt_code) s.rt=pad_trig_to_logical(ev.value,c.rt_min,c.rt_max);
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
                } else if (n <= 0 && errno!=EINTR && errno!=EAGAIN) { dead = true; break; }
            }
        }
        close(fd);
        for (int xf : xfd) close(xf);
        if (!global_running) break;
        { std::lock_guard<std::mutex> lk(st.mtx); st.st = PadLogical{}; }   // 清键位防卡键
        std::cerr<<"⚠ 手柄断开, 已清键位, 1s 重试\n";
        for (int i=0; i<10 && global_running; ++i) usleep(100'000);
    }
}
