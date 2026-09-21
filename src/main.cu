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
//    时 â≡0 指令流与纯 PI+FF 一致)。结构参数为头部常量, 详见 arena/laws/ff_pi_acc.py
//    与 AGENTS.md。
//
//  采集 (可选): 传 -o 输出目录即开启, 按三源触发自动截图 (开火 / 检测 / 定时),
//    截图 = 模型输入同款中心裁剪, 按来源分子目录, 异步写盘不阻塞推理。不传 -o 则纯自瞄。
//    三源各有独立开关 (-e, 热参 cap_fire/cap_det/cap_auto), 间隔参数见 -F/-A/-C。
//
//  鼠标接管: -a n (或热参 aim=0) 时固件纯透传真实鼠标 — 不注入任何移动, 检测/采集照常。
//    模型未完善但需要采集数据时的运行形态; aim=1 即恢复控制输出。
//
//  手柄模式 (-M pad / -M p5g, 与鼠标模式互斥): 物理手柄 (Xbox 布局) 全透传 (按键/
//    摇杆/扳机模拟量 1:1, 扳机只有"是否触发自瞄"的判定过阈值), 控制律期望速度按
//    **逐轴有效满偏屏速**折算成右摇杆注入偏转, 与人类通道径向合并 (±满偏钳制);
//    摇杆账本 Σ(偏转·ms) 供估计器/律的自身运动补偿 (按轴换算回像素)。合并后的最终
//    逻辑态经发布点交给输出后端 —— 两种手柄模式共用整条输入/合并/标定链, 只有后端
//    不同: pad 以 raw_gadget 呈现微软有线 360 手柄 (0x045E/0x028E) 给宿主 (宿主侧即
//    XInput 手柄, 见 io/pad_xinput); p5g 面向 PS5, 呈现 P5 General 加密狗形态的 HID
//    手柄 (0x2B81/0x0101) 并把一枚真加密狗插在本机口上作签名协处理器 — 每份报告
//    先交它签名, 只把它返回的字节发上线 (见 io/pad_p5g)。
//
//  热参数: UDP 127.0.0.1 上的极小本地控制通道 (白名单 t/y/x/fov/padthr/spdx/spdy/
//    adsspdx/adsspdy/k/aim/cap_*, 固件侧强制钳制), webui 保存后即时生效不重启;
//    结构常量仍为编译期, 与"无手调魔法数字"哲学一致。

//
//  标定: 只标**环路延迟 L**。hid = 鼠标双侧键长按 5 秒; 手柄模式 = L3+R3 长按 5 秒
//    或 webui 的「开始标定」按钮 (热参 padcalib=1)。激励期程序独占注入通道 (手柄
//    模式整只手柄归中, 按键照旧透传), 每段行程到位即停 + 段后停顿, 停顿里给出
//    三个独立读数 (尾迹和 = 主读数, 停止沿, 起始沿) → 中位 + MAD 判定;
//    成功 = 点头 + 只回写本输出模式的延迟 VAR (hid: HID_L_EST / pad: PAD_L_EST /
//    p5g: P5G_L_EST — 三套输出各一格, 互不覆盖), 失败 =
//    摇头 + 原因, 绝不写编造的值。速度一概不标 (手感走下面四个倍率)。
//
//  拉枪速度: 四个逐轴倍率 —— 腰射一对 (--spd), ADS 键 (右键) 按住期间一对
//    (--ads-spd); 有效灵敏度 = 基线/(倍率/100), 100 = 基线, 调大 = 更快。
//
//  本文件为程序入口: 参数解析, 设备打开, 线程孵化与 timerfd 控制主循环 (拍率 =
//    DEFAULT_FREQ, core/state.h); 其余按归属分模块 — core/ (共享状态/控制律/
//    估计器/标定/TRT 辅助), io/ (采集/鼠标输入与 USB 输出/热参)。
// ============================================================================

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <signal.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include "core/calib.h"
#include "core/control.h"
#include "core/state.h"
#include "io/calib_run.h"
#include "io/capture.h"
#include "io/hid_mouse.h"
#include "io/hotctl.h"
#include "io/pad_input.h"
#include "io/pad_output.h"
#include "io/pad_p5g.h"
#include "io/pad_xinput.h"
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
             <<"  AI 视觉自瞄 (ff_pi_acc 控制律)\n"
             <<"========================================\n";

    std::string a_m,a_c,a_t,a_y,a_d,a_f,a_x,a_l,a_S,a_k,a_v,a_r,a_D;
    std::string a_o,a_a,a_e,a_spd,a_adsspd; bool have_e=false;
    std::string a_M,a_P,a_T; bool pad_dump=false;
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
        else if (arg=="-D"&&i+1<argc) a_D=argv[++i];
        else if (arg=="-M"&&i+1<argc) a_M=argv[++i];
        else if (arg=="-P"&&i+1<argc) a_P=argv[++i];
        else if (arg=="-T"&&i+1<argc) a_T=argv[++i];
        else if (arg=="--pad-dump") pad_dump=true;
        else if (arg=="--spd"&&i+1<argc) a_spd=argv[++i];
        else if (arg=="--ads-spd"&&i+1<argc) a_adsspd=argv[++i];
        else if (arg=="-h"||arg=="--help") {
            std::cout<<"用法: "<<argv[0]<<" [自瞄选项] [采集选项]\n"
                "\n自瞄选项:\n"
                "  -m <路径>  模型       -c <ID>   类别      -t <阈值> 置信度\n"
                "  -y <偏移>  部位       -d <采集卡> 名字或 /dev/videoN\n"
                "  -f <帧率>  120/60     -x <速度> 最大px/s\n"
                "  -l <L>     初始延迟\n"
                "  -S <脚本>  回写路径 (标定只写本输出模式的延迟 VAR:\n"
                "             hid → HID_L_EST, pad → PAD_L_EST, p5g → P5G_L_EST)\n"
                "  -k <键>   fire/ads/both  -v <y/n> 预览\n"
                "  --spd <x>[,<y>]      拉枪速度倍率逐轴 (默认 100 = 基线; 调大=更快; 热参 spdx/spdy)\n"
                "  --ads-spd <x>[,<y>]  ADS 键按住时的同一对 (默认 100; 热参 adsspdx/adsspdy)\n"
                "  -r <半径>  FOV 半径 px (默认 150, 10–1000)\n"
                "  -a <y/n>   鼠标接管 (默认 y; n=纯透传: 不注入, 检测/采集照常)\n"
                "\n输出模式 (互斥, 缺省 hid):\n"
                "  -M <模式>  hid=USB raw_gadget 鼠标 (游戏内鼠标灵敏度生效)\n"
                "             pad=XInput 手柄输出: 物理手柄全透传 + 控制律注入右摇杆,\n"
                "                 以 raw_gadget 呈现微软有线 360 手柄 (0x045E/0x028E) 给宿主\n"
                "             p5g=PS5 手柄输出: 同一输入/合并/标定链, 对 PS5 呈现 P5 General\n"
                "                 加密狗 (0x2B81/0x0101), 真加密狗插本机 USB 口作签名协处理器\n"
                "                 (每份报告经它签名后才上线; 缺席时不产生任何报告)\n"
                "  -P <子串>  手柄 /dev/input/by-id 匹配子串 (默认空 = 任一 *-event-joystick\n"
                "             节点; P5 General 加密狗自身的节点已被排除)\n"
                "  -T <百分比> 手柄触发阈值 (默认 6 = 该手柄扳机实测 flat 15/255; RT/LT\n"
                "             两键共享; 只作用于触发判定, 扳机模拟量仍 1:1 透传; 热参 padthr)\n"
                "  --pad-dump 叠加调试输出: ≥50ms 打印合并后逻辑态 (透传/注入验证)\n"
                "             标定期打印的就是激励波形 (右摇杆按计划偏转, 人手通道归中)\n"
                "\n鼠标输入选项 (hid 模式):\n"
                "  -D <子串>  鼠标 /dev/input/by-id 匹配子串 (默认空 = 任一 *-event-mouse\n"
                "             字典序首个; 手柄插着时建议指定, 否则可能选中其附属鼠标接口)\n"
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
    // 交互默认与启动模板/文档同值 (置信度 0.5; 速度上限 2000 = scripts/game/
    //   template.sh.example 里 MAX_SPEED 的成文推导的落点) — 只交互式跑固件的人与经
    //   webui/模板启动的人落在同一个工作点上。
    float conf    =std::stof(!a_t.empty()?a_t:get_input_with_default("置信度","0.5"));
    float y_off   =std::stof(!a_y.empty()?a_y:get_input_with_default("Y偏移","65"));
    int   cam_fps =std::stoi(!a_f.empty()?a_f:get_input_with_default("帧率","120"));
    cam_fps=std::clamp(cam_fps,1,240);
    float max_spd =std::stof(!a_x.empty()?a_x:get_input_with_default("最大速度","2000"));
    max_spd=std::clamp(max_spd,100.0f,20000.0f);
    const float max_v=max_spd/1000.0f;
    float init_l=std::clamp(std::stof(a_l.empty()?"60":a_l),L_MIN,L_MAX);
    const std::string persist_path=a_S;
    // 拉枪速度倍率 (核心语义见 core/state.h): 有效灵敏度 = 基线/(倍率/100),
    //   --spd <x>[,<y>] (y 省略 = 与 x 同) / --ads-spd <x>[,<y>]; 缺省 = 100 = 基线。
    //   整数步进; 越界按防误输入夹取到 [SPD_MIN, SPD_MAX] (有意义的带是 5..2000)。
    auto parse_spd=[&](const std::string& spec,int& x,int& y,const char* flag)->bool {
        if (spec.empty()) return true;
        auto val=[&](const std::string& t)->int {
            size_t p=0; int v=0;
            try { v=std::stoi(t,&p); } catch (const std::exception&) { p=0; }
            if (p!=t.size()) throw std::invalid_argument(flag);
            return spd_clamp(v); };
        size_t comma=spec.find(',');
        try { x=val(spec.substr(0,comma));
              y=(comma==std::string::npos)?x:val(spec.substr(comma+1)); }
        catch (const std::exception&) {
            std::cerr<<"❌ "<<flag<<" 须为 <x>[,<y>] 整数\n"; return false; }
        return true;
    };
    int spd_x=SPD_BASE,spd_y=SPD_BASE,ads_spd_x=SPD_BASE,ads_spd_y=SPD_BASE;
    if (!parse_spd(a_spd,spd_x,spd_y,"--spd")) return 1;
    if (!parse_spd(a_adsspd,ads_spd_x,ads_spd_y,"--ads-spd")) return 1;

    // 输出模式: hid (USB raw_gadget 鼠标) / pad (XInput 手柄输出) / p5g (PS5 手柄
    //   输出) — 互斥, 缺省 hid。三者的参数展开一致 (pad 与 p5g 只换输出后端,
    //   输入/合并/标定链与逐轴 spd 倍率完全共用), 各自独占 UDC 的 raw_gadget 会话。
    const std::string out_mode=a_M.empty()?"hid":a_M;
    if (out_mode!="hid"&&out_mode!="pad"&&out_mode!="p5g") {
        std::cerr<<"❌ 未知模式 \""<<out_mode<<"\" (用 hid / pad / p5g)\n"; return 1;
    }
    const bool p5g_mode=(out_mode=="p5g");
    const bool pad_mode=(out_mode!="hid");        // 手柄通道: 输入/合并/标定/账本路由
    // 标定回写的延迟 VAR: 三套输出各一格 (各脚本里各占一行), 按本次输出模式三选一。
    //   激励计划仍只有两套 (hid 一套, pad 与 p5g 共用), 变的是值落在哪个槽里。
    const char* cal_var = p5g_mode ? CAL_VAR_P5G : (pad_mode ? CAL_VAR_PAD : CAL_VAR_HID);
    if (!a_P.empty()&&!pad_mode) std::cout<<"⚠ 忽略 -P (仅手柄模式: 手柄选择)\n";
    if (!a_T.empty()&&!pad_mode) std::cout<<"⚠ 忽略 -T (仅手柄模式: 扳机触发阈值)\n";
    if (pad_dump&&!pad_mode) std::cout<<"⚠ 忽略 --pad-dump (仅手柄模式)\n";
    if (!a_D.empty()&&pad_mode) std::cout<<"⚠ 忽略 -D (仅 hid 模式: 鼠标选择)\n";
    const std::string pad_kw=a_P.empty()?DEFAULT_PAD_KEYWORD:a_P;
    // pad 触发阈值 (% 满量程): -T 或设计缺省 (出处见 io/pad_output.h) — 两键共享,
    //   热参 padthr 运行中可调, 只影响触发判定 (扳机模拟量不受影响)
    float pad_trig_thr=PAD_TRIG_THR_PCT;
    if (!a_T.empty()) {
        try { pad_trig_thr=std::stof(a_T); }
        catch (const std::exception&) { std::cerr<<"❌ -T 需为 <百分比>\n"; return 1; }
        pad_trig_thr=std::clamp(pad_trig_thr,0.0f,100.0f);
    }
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

    // 热参数原子初始化 = CLI 值 (未收热参时运行行为与不给热参完全一致)
    g_conf_thr.store(conf); g_y_off_pct.store(y_off); g_max_v.store(max_v);
    g_aim_mode.store(aim_mode); g_fov_radius.store(fov_r);
    g_aim_enabled.store(aim_on);
    g_cap_fire.store(fire_on); g_cap_det.store(det_on); g_cap_auto.store(auto_on);
    g_spd_x.store(spd_x); g_spd_y.store(spd_y);
    g_ads_spd_x.store(ads_spd_x); g_ads_spd_y.store(ads_spd_y);
    g_pad_trig_thr.store(pad_trig_thr);

    const bool do_collect=!a_o.empty();
    if (do_collect) { ensure_dir(a_o); ensure_dir(a_o+"/fire");
                      ensure_dir(a_o+"/det"); ensure_dir(a_o+"/auto"); }

    std::string cam_dev=resolve_cam_device(a_d.empty()?"/dev/video0":a_d);
    if (cam_dev.empty()) return 1;
    if (access(cam_dev.c_str(),F_OK)!=0) {
        std::cerr<<"❌ 采集卡设备不存在: "<<cam_dev<<"\n"; return 1; }
    std::cout<<"✅ 采集卡: "<<cam_dev<<"\n";

    MouseState state;
    UsbRawSession usb;
    PadState padst;
    // 三模式单次运行只居其一, 且各自独占 UDC: hid = USB raw_gadget 鼠标,
    //   pad = XInput 有线手柄 (0x045E/0x028E), p5g = P5G 加密狗形态 HID 手柄
    //   (0x2B81/0x0101; 真加密狗经 hidraw 作签名协处理器, 缺席不阻塞启动)
    if      (p5g_mode) { if (!pad_p5g_start())   return 1; }
    else if (pad_mode) { if (!pad_xinput_start()) return 1; }
    else               { if (!hid_mouse_start(state,usb,a_D)) return 1; }

    signal(SIGINT,signal_handler); signal(SIGTERM,signal_handler);
    { std::lock_guard<std::mutex> lk(g_target.mtx); g_target.l_est_ms=init_l; }
    // 自身运动账本的来源与账本→像素比例随输出模式 (io/pad_output.h; 单次运行一模式)
    own_motion_ledger_set(pad_mode);

    // 启动行: 模式名 + 本模式那一组参数 (延迟 4 个 spd) + 延迟落在哪条 VAR 上 ——
    //   现场第一眼要看清"现在用的是哪个槽"
    std::cout<<"初始: 模式="<<out_mode<<" L="<<init_l<<" ("<<cal_var<<")"
             <<" spd_x="<<spd_x<<" spd_y="<<spd_y
             <<" ads_spd_x="<<ads_spd_x<<" ads_spd_y="<<ads_spd_y<<" fov="<<fov_r<<"\n";
    if (p5g_mode)
        std::cout<<"模式: p5g (对 PS5 呈现 P5 General 手柄 0x2B81/0x0101, 真加密狗"
                    "经 hidraw 签名; 物理手柄全透传"
                 <<(pad_dump?", --pad-dump 叠加打印":"")<<")  触发阈值="<<pad_trig_thr<<"%\n";
    else if (pad_mode)
        std::cout<<"模式: pad (XInput 手柄 0x045E/0x028E 呈现给宿主, 物理手柄全透传"
                 <<(pad_dump?", --pad-dump 叠加打印":"")<<")  触发阈值="<<pad_trig_thr<<"%\n";
    else
        std::cout<<"模式: hid (USB raw_gadget 通用鼠标, 物理鼠标经 EVIOCGRAB 独占读取"
                    "并合并注入)\n";
    std::cout<<(pad_mode?"辅助瞄准注入: ":"鼠标接管: ")
             <<(aim_on?"开":"关 (纯透传: 原样透传, 检测/采集照常)")<<"\n";
    if (do_collect) {
        std::string srcs;
        auto add_src=[&](bool on,const char* n){ if(on){ if(!srcs.empty()) srcs+=","; srcs+=n; } };
        add_src(fire_on,"fire"); add_src(det_on,"det"); add_src(auto_on,"auto");
        std::cout<<"采集: "<<a_o<<"  源="<<(srcs.empty()?"(无)":srcs)
                 <<"  开火="<<fire_ms<<"ms  定时="<<auto_s<<"s  冷却="<<cooldown_ms<<"ms\n";
    } else
        std::cout<<"采集: 关闭 (未传 -o)\n";

    std::thread pad_reader;
    if (pad_mode) {
        // 启动一次性诊断: 手柄在位/节点提示 + -P 多匹配报错 (verbose); 读取线程
        //   自带缺席重试与掉线自愈, 未在位不阻塞启动
        std::string pd=find_pad_device(pad_kw,true);
        if (pd.empty()) std::cout<<"⚠ 手柄当前不在位 (读取线程每秒重试)\n";
        else            std::cout<<"✅ 手柄节点: "<<pd<<"\n";
        pad_reader=std::thread(pad_reader_thread,pad_kw,std::ref(padst));
    }

    std::thread writer; if (do_collect) writer=std::thread(writer_thread,jpeg_q);
    std::thread hot(hotctl_thread);
    std::thread ai(ai_thread,model_path,cls,cam_dev,cam_fps,preview,
                   init_l,persist_path,cal_var,
                   pad_mode?CAL_MODE_PAD:CAL_MODE_HID,
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
        if (pad_mode) {
            pad_tick(cam_fps,padst,pad_dump);   // 手柄拍: 快照 → 触发 → 律 → 合并 → 发布
            continue;
        }
        int16_t x,y;int8_t w,hw;uint16_t b;
        extract_and_clear(state,x,y,w,hw,b);
        hid_report_submit(usb,x,y,w,hw,b,[cam_fps](std::array<uint8_t,HID_REPORT_LEN>& rpt,
                                                   int16_t rx, int16_t ry) {
            control_apply(cam_fps,rpt.data(),rx,ry); });
    }

    global_running=false;
    g_save_cv.notify_all();
    // 输出后端先停 (它持有 UDC 的 raw_gadget 会话), 再停输入读取线程与其它线程
    if (p5g_mode) pad_p5g_stop();
    else if (pad_mode) pad_xinput_stop();
    else hid_mouse_stop(usb);
    if (pad_reader.joinable()) pad_reader.join();
    hot.join(); ai.join();
    if (do_collect) writer.join();
    close(tfd);close(ep);
    std::cout<<"已停止\n";
    return 0;
}
