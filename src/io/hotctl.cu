// ============================================================================
//  hotctl.cu — hotctl_thread 的实现: loopback-only UDP 套接字, 白名单外的
//    key=value 整对忽略, 数值/枚举/布尔各自的钳制与拒绝, 应用即打印。
//    白名单: t/y/x/fov (数值) · k (枚举) · aim/cap_*/padcalib (0/1)。
// ============================================================================

#include "io/hotctl.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "core/state.h"

// UDP 127.0.0.1 收 "key=value;key=value" (一个数据报可带多对, 分号/换行分隔):
// 白名单外整对忽略, 数值在此强制钳制 (不信任发送方); 应用即打印, 经日志确认生效。
void hotctl_thread() {
    int fd=socket(AF_INET,SOCK_DGRAM,0);
    if (fd<0) { std::cerr<<"热参数通道: socket 创建失败\n"; return; }
    sockaddr_in addr{}; addr.sin_family=AF_INET;
    addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    addr.sin_port=htons(HOT_CTL_PORT);
    if (bind(fd,(sockaddr*)&addr,sizeof(addr))<0) {
        std::cerr<<"⚠ 热参数通道绑定失败 (端口 "<<HOT_CTL_PORT<<" 被占), 热参不可用\n";
        close(fd); return; }
    std::cout<<"✅ 热参数通道: 127.0.0.1:"<<HOT_CTL_PORT<<" (t/y/x/fov/k/aim/cap_*/padcalib)\n";
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
            const char* key=tok; const char* val=eq+1;
            if (!strcmp(key,"t")||!strcmp(key,"y")||!strcmp(key,"x")||!strcmp(key,"fov")) {
                char* end=nullptr;
                float v=strtof(val,&end);
                if (end==val||*end!=0) { std::cout<<"[热参] 忽略 "<<key<<"="<<val<<" (非数值)\n"; continue; }
                if      (!strcmp(key,"t"))   { v=std::clamp(v,0.0f,1.0f);       g_conf_thr.store(v); }
                else if (!strcmp(key,"y"))   { v=std::clamp(v,0.0f,100.0f);     g_y_off_pct.store(v); }
                else if (!strcmp(key,"x"))   { v=std::clamp(v,100.0f,20000.0f); g_max_v.store(v/1000.0f); }
                else                         { v=std::clamp(v,10.0f,1000.0f);   g_fov_radius.store(v); }
                std::cout<<"[热参] "<<key<<"="<<v<<"\n";
            } else if (!strcmp(key,"k")) {
                int m=!strcmp(val,"fire")?0:!strcmp(val,"ads")?1:!strcmp(val,"both")?2:-1;
                if (m>=0) { g_aim_mode.store(m); std::cout<<"[热参] k="<<val<<"\n"; }
                else std::cout<<"[热参] 忽略 k="<<val<<" (须 fire/ads/both)\n";
            } else if (!strcmp(key,"aim")||!strcmp(key,"cap_fire")
                       ||!strcmp(key,"cap_det")||!strcmp(key,"cap_auto")
                       ||!strcmp(key,"padcalib")) {
                if (!strcmp(val,"0")||!strcmp(val,"1")) {
                    bool on=(val[0]=='1');
                    if      (!strcmp(key,"aim"))      g_aim_enabled.store(on);
                    else if (!strcmp(key,"cap_fire")) g_cap_fire.store(on);
                    else if (!strcmp(key,"cap_det"))  g_cap_det.store(on);
                    else if (!strcmp(key,"cap_auto")) g_cap_auto.store(on);
                    // padcalib: 1 = 请求开始手柄标定 (pad 拍一次消费即清, 与
                    //   g_calib_request 同一 exchange 语义); 0 = 无操作
                    else                              g_padcalib_request.store(on);
                    std::cout<<"[热参] "<<key<<"="<<val<<"\n";
                } else std::cout<<"[热参] 忽略 "<<key<<"="<<val<<" (须 0/1)\n";
            } else {
                std::cout<<"[热参] 忽略未知 key: "<<key<<"\n";
            }
        }
    }
    close(fd);
}
