// ============================================================================
//  hotctl.cu — hotctl_thread 的实现: loopback-only UDP 套接字 + 白名单整对分发;
//    白名单外的 key 忽略, 数值/枚举/布尔各自的钳制与拒绝在 hotctl_apply 里,
//    应用即打印 (经日志确认生效)。
// ============================================================================

#include "io/hotctl.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "core/state.h"

// 数值 token 解析: 整对消费成功才有效 (尾随垃圾一律拒绝)
static bool num_val(const char* val, float& out) {
    char* end=nullptr;
    out=strtof(val,&end);
    return end!=val && *end==0;
}

bool hotctl_apply(const char* key, const char* val) {
    if (!strcmp(key,"t")||!strcmp(key,"y")||!strcmp(key,"x")||!strcmp(key,"fov")) {
        float v=0;
        if (!num_val(val,v)) { std::cout<<"[热参] 忽略 "<<key<<"="<<val<<" (非数值)\n"; return false; }
        if      (!strcmp(key,"t"))   { v=std::clamp(v,0.0f,1.0f);       g_conf_thr.store(v); }
        else if (!strcmp(key,"y"))   { v=std::clamp(v,0.0f,100.0f);     g_y_off_pct.store(v); }
        else if (!strcmp(key,"x"))   { v=std::clamp(v,100.0f,20000.0f); g_max_v.store(v/1000.0f); }
        else                         { v=std::clamp(v,10.0f,1000.0f);   g_fov_radius.store(v); }
        std::cout<<"[热参] "<<key<<"="<<v<<"\n";
        return true;
    }
    // 拉枪速度倍率: 整数刻度 (100 = 基线), 四舍五入到整数后夹取 (同 CLI)
    if (!strcmp(key,"spdx")||!strcmp(key,"spdy")
        ||!strcmp(key,"adsspdx")||!strcmp(key,"adsspdy")) {
        float v=0;
        if (!num_val(val,v)) { std::cout<<"[热参] 忽略 "<<key<<"="<<val<<" (非数值)\n"; return false; }
        const bool ads=!strncmp(key,"ads",3);
        const bool y=key[ads?6:3]=='y';                    // spdy / adsspdy
        const int iv=spd_clamp(std::lround(v));
        (ads ? (y ? g_ads_spd_y : g_ads_spd_x) : (y ? g_spd_y : g_spd_x)).store(iv);
        std::cout<<"[热参] "<<key<<"="<<iv<<"\n";
        return true;
    }
    if (!strcmp(key,"k")) {
        int m=!strcmp(val,"fire")?0:!strcmp(val,"ads")?1:!strcmp(val,"both")?2:-1;
        if (m>=0) { g_aim_mode.store(m); std::cout<<"[热参] k="<<val<<"\n"; return true; }
        std::cout<<"[热参] 忽略 k="<<val<<" (须 fire/ads/both)\n";
        return false;
    }
    if (!strcmp(key,"aim")||!strcmp(key,"cap_fire")
        ||!strcmp(key,"cap_det")||!strcmp(key,"cap_auto")) {
        if (!strcmp(val,"0")||!strcmp(val,"1")) {
            bool on=(val[0]=='1');
            if      (!strcmp(key,"aim"))      g_aim_enabled.store(on);
            else if (!strcmp(key,"cap_fire")) g_cap_fire.store(on);
            else if (!strcmp(key,"cap_det"))  g_cap_det.store(on);
            else                              g_cap_auto.store(on);
            std::cout<<"[热参] "<<key<<"="<<val<<"\n";
            return true;
        }
        std::cout<<"[热参] 忽略 "<<key<<"="<<val<<" (须 0/1)\n";
        return false;
    }
    std::cout<<"[热参] 忽略未知 key: "<<key<<"\n";
    return false;
}

// UDP 127.0.0.1 收 "key=value;key=value" (一个数据报可带多对, 分号/换行分隔):
// 白名单外整对忽略, 数值在应用侧强制钳制 (不信任发送方)。
void hotctl_thread() {
    int fd=socket(AF_INET,SOCK_DGRAM,0);
    if (fd<0) { std::cerr<<"热参数通道: socket 创建失败\n"; return; }
    sockaddr_in addr{}; addr.sin_family=AF_INET;
    addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    addr.sin_port=htons(HOT_CTL_PORT);
    if (bind(fd,(sockaddr*)&addr,sizeof(addr))<0) {
        std::cerr<<"⚠ 热参数通道绑定失败 (端口 "<<HOT_CTL_PORT<<" 被占), 热参不可用\n";
        close(fd); return; }
    std::cout<<"✅ 热参数通道: 127.0.0.1:"<<HOT_CTL_PORT
             <<" (t/y/x/fov/spdx/spdy/adsspdx/adsspdy/k/aim/cap_*)\n";
    struct pollfd pfd{}; pfd.fd=fd; pfd.events=POLLIN;
    char buf[256];
    while (global_running) {
        int pr=poll(&pfd,1,200);
        if (pr<=0) continue;
        ssize_t n=recvfrom(fd,buf,sizeof(buf)-1,0,nullptr,nullptr);
        if (n<=0) continue;
        buf[n]=0;
        for (char* tok=strtok(buf,";\r\n"); tok; tok=strtok(nullptr,";\r\n")) {
            char* eq=strchr(tok,'=');
            if (!eq) continue;
            *eq=0;
            hotctl_apply(tok,eq+1);
        }
    }
    close(fd);
}
