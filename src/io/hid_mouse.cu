// ============================================================================
//  hid_mouse.cu — hid_mouse.h 的实现: evdev 事件解析 (REL_*/BTN_* 累积),
//    /dev/input/by-id 设备名匹配, 独占读取循环, HID 报文组包与错误处理
//    (瞬时节流丢帧, 持续硬错误停机)。
// ============================================================================

#include "io/hid_mouse.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <iostream>

#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

void extract_and_clear(MouseState& s, int16_t& x, int16_t& y,
                       int8_t& w, int8_t& hw, uint16_t& btns) {
    std::lock_guard<std::mutex> lk(s.mtx);
    x=(int16_t)std::clamp(s.rel_x,-32768,32767);
    y=(int16_t)std::clamp(s.rel_y,-32768,32767);
    w=(int8_t)std::clamp(s.rel_wheel,-128,127);
    hw=(int8_t)std::clamp(s.rel_hwheel,-128,127);
    btns=s.buttons;
    s.rel_x=s.rel_y=s.rel_wheel=s.rel_hwheel=0;
}
std::string find_mouse_device(const std::string& kw) {
    if (!kw.empty() && kw.front()=='/') return access(kw.c_str(),R_OK)==0?kw:"";
    std::string cmd="find "+std::string(DEV_SEARCH_PATH)
                   +" -name '*"+kw+"*-event-mouse' -print -quit 2>/dev/null";
    FILE* fp=popen(cmd.c_str(),"r"); if (!fp) return {};
    char buf[512]; std::string r;
    if (fgets(buf,sizeof(buf),fp)) { r=buf; if(!r.empty()&&r.back()=='\n') r.pop_back(); }
    pclose(fp); return r;
}
void reader_thread(const std::string& dev, MouseState& st) {
    int fd=open(dev.c_str(),O_RDONLY);
    if (fd<0) { std::cerr<<"无法打开鼠标\n"; global_running=false; return; }
    if (ioctl(fd,EVIOCGRAB,1)<0) std::cerr<<"警告: 无法独占\n";
    else std::cout<<"✅ 已独占: "<<dev<<"\n";
    struct pollfd pfd{}; pfd.fd=fd; pfd.events=POLLIN;
    struct input_event ev; int errs=0;
    while (global_running) {
        int pr=poll(&pfd,1,100);
        if (pr<0) { if(errno==EINTR)continue; global_running=false; break; }
        if (pr==0) continue;
        if (!(pfd.revents&POLLIN)) { std::cerr<<"鼠标断开\n"; global_running=false; break; }
        ssize_t n=read(fd,&ev,sizeof(ev));
        if (n==(ssize_t)sizeof(ev)) { errs=0;
            std::lock_guard<std::mutex> lk(st.mtx);
            if (ev.type==EV_REL) {
                if(ev.code==REL_X)st.rel_x+=ev.value;
                else if(ev.code==REL_Y)st.rel_y+=ev.value;
                else if(ev.code==REL_WHEEL)st.rel_wheel+=ev.value;
                else if(ev.code==REL_HWHEEL)st.rel_hwheel+=ev.value;
            } else if (ev.type==EV_KEY&&ev.code>=BTN_LEFT&&ev.code<=BTN_TASK) {
                int idx=ev.code-BTN_LEFT;
                if(ev.value)st.buttons|=(1<<idx); else st.buttons&=~(1<<idx); }
        } else if (n<0&&errno!=EINTR&&errno!=EAGAIN) {
            if(++errs>10){std::cerr<<"鼠标读取失败\n";global_running=false;break;} usleep(1000); }
    }
    close(fd);
}
void send_report(int fd, int16_t rx, int16_t ry, int8_t w, int8_t hw, uint16_t btns,
                 const std::function<void(std::array<uint8_t,HID_REPORT_LEN>&,int16_t,int16_t)>& overlay) {
    std::array<uint8_t,HID_REPORT_LEN> rpt{};
    rpt[0]=0x02; rpt[1]=btns&0xFF; rpt[2]=btns>>8;
    rpt[3]=rx&0xFF; rpt[4]=rx>>8; rpt[5]=ry&0xFF; rpt[6]=ry>>8;
    rpt[7]=w; rpt[8]=hw;
    if (overlay) overlay(rpt,rx,ry);
    static int errs=0;
    ssize_t n;
    do { n=::write(fd,rpt.data(),HID_REPORT_LEN); } while (n<0&&errno==EINTR);  // 信号中断: 重试
    if (n==(ssize_t)HID_REPORT_LEN) { errs=0; return; }
    if (n<0&&(errno==EAGAIN||errno==EWOULDBLOCK)) return;        // 瞬时节流: 丢这一帧, 不停机
    if (++errs>10) { std::cerr<<"写入虚拟鼠标失败\n"; global_running=false; }  // 持续硬错误才停
}
