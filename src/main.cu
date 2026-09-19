// ============================================================================
//  aimbot — AI 视觉自瞄 (鼠标透传式) + 可选训练数据采集
//
//  链路: 采集卡 (UVC 1080p NV12, -d 按名字选择) → GStreamer nvvidconv
//        → CUDA 预处理 → TensorRT YOLO 检测 → alpha-beta 目标跟踪
//        → 控制律 (极点配置 PI + type-2 速度前馈) → USB raw_gadget 鼠标透传
//
//  控制律: 收敛带宽 wn 由标定延迟 L 自动导出 (wn=(90°−PM)π/180/L, PM=50°, 免手调),
//    ζ=1 临界阻尼; type-2 速度前馈 (FF_GAIN_VAL=1) 补匀速跟踪零拖尾; 创新均值反演 â
//    修正 α-β 对加速目标的结构性滞后 (重建抑制/自身活动门/显著性地板三重门控, 无加速
//    时 â≡0 指令流与纯 ff_pi 一致)。结构参数为头部常量, 详见 arena/laws/ff_pi_acc.py
//    与 AGENTS.md。
//
//  采集 (可选): 传 -o 输出目录即开启, 按三源触发自动截图 (开火 / 检测 / 定时),
//    截图 = 模型输入同款中心裁剪, 按来源分子目录, 异步写盘不阻塞推理。不传 -o 则纯自瞄。
//    三源各有独立开关 (-e, 热参 cap_fire/cap_det/cap_auto), 间隔参数见 -F/-A/-C。
//
//  鼠标接管: -a n (或热参 aim=0) 时固件纯透传真实鼠标 — 不注入任何移动, 检测/采集照常。
//    模型未完善但需要采集数据的运行形态; aim=1 即恢复控制输出。
//
//  热参数: UDP 127.0.0.1 上的极小本地控制通道 (白名单 t/y/x/fov/k/aim/cap_*, 固件侧强制
//    钳制), webui 保存后即时生效不重启; 结构常量仍为编译期, 与"无手调魔法数字"哲学一致。
//
//  标定: 双侧键长按 5 秒, 程序自动生成激励轨迹 (画正方形), 块相位相关测背景位移,
//    最小二乘估计灵敏度 s (px/count) + 环路延迟 L (ms); 经 -S 传入脚本路径时自动回写。
//
//  本文件为程序入口: 参数解析, 设备打开, 线程孵化与 timerfd 控制主循环
//    (拍率 = DEFAULT_FREQ, 见 core/state.h);
//    模块划分 — core/ (控制律/估计器/标定/TRT 辅助/共享状态), io/ (采集/USB
//    鼠标/raw_gadget 会话/热参)。
// ============================================================================

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <thread>

#include <signal.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include "core/calib.h"
#include "core/control.h"
#include "core/state.h"
#include "io/capture.h"
#include "io/hid_mouse.h"
#include "io/hotctl.h"
#include "io/usbraw.h"

// ========================= 命令行交互 =========================
static std::string get_input_with_default(const std::string& prompt, const std::string& def) {
    std::cout << prompt << " [" << def << "]: ";
    std::string line; std::getline(std::cin, line);
    return line.empty() ? def : line;
}

// ========================= main =========================
int main(int argc, char* argv[]) {
    std::cout<<"========================================\n"
             <<"  AI 视觉自瞄 (ff_pi 控制律)\n"
             <<"========================================\n";

    std::string a_m,a_c,a_t,a_y,a_d,a_f,a_x,a_s,a_l,a_S,a_k,a_v,a_r;
    std::string a_o,a_a,a_e; bool have_e=false;
    int fire_ms=300; double auto_s=10;
    int cooldown_ms=500; int jpeg_q=95;

    for (int i=1;i<argc;++i) {
        std::string arg=argv[i];
        if      (arg=="-m"&&i+1<argc) a_m=argv[++i];
        else if (arg=="-c"&&i+1<argc) a_c=argv[++i];
        else if (arg=="-t"&&i+1<argc) a_t=argv[++i];
        else if (arg=="-y"&&i+1<argc) a_y=argv[++i];
        else if (arg=="-d"&&i+1<argc) a_d=argv[++i];
        else if (arg=="-f"&&i+1<argc) a_f=argv[++i];
        else if (arg=="-x"&&i+1<argc) a_x=argv[++i];
        else if (arg=="-s"&&i+1<argc) a_s=argv[++i];
        else if (arg=="-l"&&i+1<argc) a_l=argv[++i];
        else if (arg=="-S"&&i+1<argc) a_S=argv[++i];
        else if (arg=="-k"&&i+1<argc) a_k=argv[++i];
        else if (arg=="-v"&&i+1<argc) a_v=argv[++i];
        else if (arg=="-o"&&i+1<argc) a_o=argv[++i];
        else if (arg=="-a"&&i+1<argc) a_a=argv[++i];
        else if (arg=="-e"&&i+1<argc) { a_e=argv[++i]; have_e=true; }
        else if (arg=="-F"&&i+1<argc) fire_ms=std::stoi(argv[++i]);
        else if (arg=="-A"&&i+1<argc) auto_s=std::stod(argv[++i]);
        else if (arg=="-C"&&i+1<argc) cooldown_ms=std::stoi(argv[++i]);
        else if (arg=="-q"&&i+1<argc) jpeg_q=std::stoi(argv[++i]);
        else if (arg=="-r"&&i+1<argc) a_r=argv[++i];
        else if (arg=="-h"||arg=="--help") {
            std::cout<<"用法: "<<argv[0]<<" [自瞄选项] [采集选项]\n"
                "\n自瞄选项:\n"
                "  -m <路径>  模型       -c <ID>   类别      -t <阈值> 置信度\n"
                "  -y <偏移>  部位       -d <采集卡> 名字或 /dev/videoN\n"
                "  -f <帧率>  120/60     -x <速度> 最大px/s\n"
                "  -s <s>     初始灵敏度 -l <L>    初始延迟\n"
                "  -S <脚本>  回写路径   -k <键>   fire/ads/both  -v <y/n> 预览\n"
                "  -r <半径>  FOV 半径 px (默认 150, 10–1000)\n"
                "  -a <y/n>   鼠标接管 (默认 y; n=纯透传: 不动鼠标, 检测/采集照常)\n"
                "\n采集选项 (不传 -o 则纯自瞄不截图):\n"
                "  -o <目录>  输出目录 (自动建 fire/ det/ auto/ 子目录)\n"
                "  -e <列表>  启用的截图源 fire/det/auto 逗号分隔 (默认全部; 也可运行中热切)\n"
                "  -F <ms>    开火截图间隔 (默认 300)\n"
                "  -A <秒>    定时截图间隔 (默认 10, 随机 0.5x~1.5x)\n"
                "  -C <ms>    检测/定时截图冷却 (默认 500, 开火不受限)\n"
                "  -q <1-100> JPEG 质量 (默认 95)\n";
            return 0;
        }
    }

    std::string model_path;
    if (!a_m.empty()) { model_path=a_m;
        if (!std::ifstream(model_path).good()) { std::cerr<<"❌ 模型不存在\n"; return 1; }
    } else { while(true) { std::cout<<"模型路径: "; std::getline(std::cin,model_path);
             if (std::ifstream(model_path).good()) break; std::cerr<<"文件不存在\n"; } }

    int   cls     =std::stoi(!a_c.empty()?a_c:get_input_with_default("类别ID","0"));
    float conf    =std::stof(!a_t.empty()?a_t:get_input_with_default("置信度","0.4"));
    float y_off   =std::stof(!a_y.empty()?a_y:get_input_with_default("Y偏移","65"));
    int   cam_fps =std::stoi(!a_f.empty()?a_f:get_input_with_default("帧率","120"));
    cam_fps=std::clamp(cam_fps,1,240);
    float max_spd =std::stof(!a_x.empty()?a_x:get_input_with_default("最大速度","1500"));
    max_spd=std::clamp(max_spd,100.0f,20000.0f);
    const float max_v=max_spd/1000.0f;
    float init_s=std::clamp(std::stof(a_s.empty()?"1.0":a_s),S_MIN,S_MAX);
    float init_l=std::clamp(std::stof(a_l.empty()?"60":a_l),L_MIN,L_MAX);
    const std::string persist_path=a_S;
    std::string aim_key=!a_k.empty()?a_k:get_input_with_default("触发键","fire");
    int aim_mode=0;
    if(aim_key=="ads")aim_mode=1; else if(aim_key=="both")aim_mode=2;
    else if(aim_key!="fire")std::cerr<<"未知触发键, 用 fire\n";
    std::string pv=!a_v.empty()?a_v:get_input_with_default("预览(y/n)","n");
    bool preview=(pv=="y"||pv=="Y");
    if(!preview) unsetenv("DISPLAY");
    jpeg_q=std::clamp(jpeg_q,1,100);
    float fov_r=std::clamp(std::stof(a_r.empty()?"150":a_r),10.0f,1000.0f);

    // 鼠标接管 (默认开) 与截图源 (默认全开; -e 给出时以该列表为准, 可为空 = 全关)
    bool aim_on=!(a_a=="n"||a_a=="N");
    bool fire_on=true, det_on=true, auto_on=true;
    if (have_e) {
        fire_on=det_on=auto_on=false;
        std::string e=a_e;
        e.erase(std::remove(e.begin(),e.end(),' '),e.end());
        for (size_t p=0; p<e.size(); ) {
            size_t q=e.find(',',p); if (q==std::string::npos) q=e.size();
            std::string tok=e.substr(p,q-p);
            if      (tok=="fire") fire_on=true;
            else if (tok=="det")  det_on=true;
            else if (tok=="auto") auto_on=true;
            else std::cerr<<"未知截图源 \""<<tok<<"\" (可用: fire det auto)\n";
            p=q+1;
        }
    }

    // 热参数原子初始化 = CLI 值 (不接热参即纯 CLI 语义)
    g_conf_thr.store(conf); g_y_off_pct.store(y_off); g_max_v.store(max_v);
    g_aim_mode.store(aim_mode); g_fov_radius.store(fov_r);
    g_aim_enabled.store(aim_on);
    g_cap_fire.store(fire_on); g_cap_det.store(det_on); g_cap_auto.store(auto_on);

    const bool do_collect=!a_o.empty();
    if (do_collect) { ensure_dir(a_o); ensure_dir(a_o+"/fire");
                      ensure_dir(a_o+"/det"); ensure_dir(a_o+"/auto"); }

    std::string cam_dev=resolve_cam_device(a_d.empty()?"/dev/video0":a_d);
    if (cam_dev.empty()) return 1;
    if (access(cam_dev.c_str(),F_OK)!=0) {
        std::cerr<<"❌ 采集卡设备不存在: "<<cam_dev<<"\n"; return 1; }
    std::cout<<"✅ 采集卡: "<<cam_dev<<"\n";

    MouseState state;
    UsbRawSession usb_session;
    if (!hid_mouse_start(state,usb_session)) return 1;   // 可行动原因已打印 (设备/模块/UDC 占用)

    signal(SIGINT,signal_handler); signal(SIGTERM,signal_handler);
    { std::lock_guard<std::mutex> lk(g_target.mtx);
      g_target.s_est=init_s; g_target.l_est_ms=init_l; }

    std::cout<<"初始: s="<<init_s<<" L="<<init_l<<" fov="<<fov_r<<"\n";
    std::cout<<"鼠标接管: "<<(aim_on?"开":"关 (纯透传: 不注入移动, 检测/采集照常)")<<"\n";
    if (do_collect) {
        std::string srcs;
        auto add_src=[&](bool on,const char* n){ if(on){ if(!srcs.empty()) srcs+=","; srcs+=n; } };
        add_src(fire_on,"fire"); add_src(det_on,"det"); add_src(auto_on,"auto");
        std::cout<<"采集: "<<a_o<<"  源="<<(srcs.empty()?"(无)":srcs)
                 <<"  开火="<<fire_ms<<"ms  定时="<<auto_s<<"s  冷却="<<cooldown_ms<<"ms\n";
    } else
        std::cout<<"采集: 关闭 (未传 -o)\n";

    std::thread writer; if (do_collect) writer=std::thread(writer_thread,jpeg_q);
    std::thread hot(hotctl_thread);
    std::thread ai(ai_thread,model_path,cls,cam_dev,cam_fps,preview,
                   init_s,init_l,persist_path,
                   a_o,fire_ms,auto_s,cooldown_ms,jpeg_q);

    int tfd=timerfd_create(CLOCK_MONOTONIC,0);
    struct itimerspec its{}; its.it_value.tv_nsec=1;
    its.it_interval.tv_nsec=1'000'000'000/DEFAULT_FREQ;
    timerfd_settime(tfd,0,&its,nullptr);
    int ep=epoll_create1(0);
    struct epoll_event evt{}; evt.events=EPOLLIN; evt.data.fd=tfd;
    epoll_ctl(ep,EPOLL_CTL_ADD,tfd,&evt);

    std::cout<<"✅ "<<DEFAULT_FREQ<<"Hz 运行中, Ctrl+C 停止\n";
    while (global_running) {
        struct epoll_event evs[1];
        int nf=epoll_wait(ep,evs,1,500);
        if(nf<0&&errno==EINTR)continue; if(nf<=0)continue;
        uint64_t exp; read(tfd,&exp,sizeof(exp));
        int16_t x,y;int8_t w,hw;uint16_t b;
        extract_and_clear(state,x,y,w,hw,b);
        hid_report_submit(usb_session,x,y,w,hw,b,[cam_fps](std::array<uint8_t,HID_REPORT_LEN>& rpt,
                                                           int16_t rx, int16_t ry) {
            control_apply(cam_fps,rpt.data(),rx,ry); });
    }

    global_running=false;
    g_save_cv.notify_all();
    hid_mouse_stop(usb_session);
    hot.join(); ai.join();
    if (do_collect) writer.join();
    close(tfd);close(ep);
    std::cout<<"已停止\n";
    return 0;
}
