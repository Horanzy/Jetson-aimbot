// ============================================================================
//  control.cu — ff_pi_acc 的控制拍执行 (拍率 = DEFAULT_FREQ): Smith ê 组装
//    (含 â 的 ε 修正与 ½â·W² 外推) → 极点配置 PI (条件积分 + 距离门控) →
//    type-2 速度前馈 (信任度插值门控 + 检测间隙衰减); 双侧键触发的标定状态机
//    (标定引擎见 io/calib_run.h) 也在此驱动。律只写一次 (law_tick), hid 与手柄
//    两条通道 (手柄通道即 pad/p5g 两个输出模式) 共用同一份数学 — 差别只在指标
//    去向: hid 量化成 counts 写报文位移字节并记 g_counts, 手柄通道交付期望速度
//    (px/ms) 并跳过 hid 专属尾巴。跨帧控制状态 (积分器/量化余量/状态机相位) 为
//    函数内 static。
//    自身运动账本 (在飞补偿) 的来源与账本→像素比例随输出模式路由
//    (io/pad_output.h: hid = g_counts + 逐轴有效灵敏度, pad = 摇杆账本 + 逐轴
//    有效满偏屏速); 注入换算与同一份比例同口径 — spd 放大命令多少, 补偿跟随多少。
// ============================================================================

#include "core/control.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "core/state.h"
#include "io/calib_run.h"       // cal_step: 标定状态机 (hid 由本文件的律拍驱动)
#include "io/pad_output.h"      // own_motion_ledger/own_motion_scale: 账本来源与
                                //   账本→像素比例随输出模式 (hid/pad)

namespace {

// 控制拍的输入与去向: hid 与 pad 只差"指标写在哪" (报文位移字节 vs 期望速度)
struct LawIn {
    int cam_fps = 120;
    int16_t real_x = 0, real_y = 0;      // 真实鼠标本拍累计位移 (hid 透传合入)
    uint16_t btns = 0;                   // 触发键位字 (fire→LEFT_KEY, ads→RIGHT_KEY)
    bool pad = false;
    uint8_t* rpt = nullptr;              // hid: HID_REPORT_LEN 字节报文 (只改写位移)
};
struct LawOut {
    float vx = 0, vy = 0;                // pad: 期望屏幕速度 (px/ms)
    bool gate = false;                   // pad: 注入门 (接管开 × 触发保持窗内)
};

// ---- hid 标定 ---------------------------------------------------------------
// 状态机在 io/calib_run.cu (hid 与手柄两套激励计划共用同一份状态机与拟合; pad 与 p5g
//   共用其中手柄那一套, 差的是回写的延迟 VAR); 本文件只把键位字递给它并接回本拍
//   的注入命令。激励期整条注入通道归激励独占, 律本拍不参与 (接管关闭时状态机自
//   复位: 激励是程序注入的移动, 与纯透传互斥)。
struct CalFrame { bool active=false; int32_t cx=0, cy=0; };

CalFrame hid_cal_tick(int cam_fps, uint16_t btns, std::chrono::steady_clock::time_point now) {
    const CalStep st=cal_step(CAL_MODE_HID,btns,cam_fps,now);
    CalFrame c; c.active=st.active; c.cx=st.cx; c.cy=st.cy;
    return c;
}

// 律一拍。hid 的报文位移与 g_counts 记账在本函数尾部 (标定激励期同样走这里 →
//   激励的位移也逐拍记入 g_counts, 与瞄准注入同一账本)。
void law_tick(const LawIn& in, LawOut& out) {
    static auto last_press=std::chrono::steady_clock::now()-std::chrono::hours(1);
    static float rem_x=0,rem_y=0;
    static float int_x=0,int_y=0;

    auto now=std::chrono::steady_clock::now();
    bool left=in.btns&LEFT_KEY, right=in.btns&RIGHT_KEY, side=in.btns&SIDE_KEY;
    g_left_down.store(left);
    // ADS 键 = 右键位: 本拍的有效增益换算与逐帧消费者 (估计器自身运动补偿) 取
    //   同一状态, ADS 期间的补偿不漂。ADS 倍率在按住的那一拍就生效。
    const bool ads=right;
    g_ads_down.store(ads);
    // 账本→像素比例 (逐轴有效增益) 与账本来源都随输出模式 (io/pad_output.h):
    //   hid 的 px/count = s_hid_now, pad 的是有效满偏屏速换算
    const LedgerPxScale sc=own_motion_scale();

    // hid 标定: 状态机 (io/calib_run.cu) 双侧键长按触发, 激励期独占注入
    CalFrame cal;
    if (!in.pad) cal=hid_cal_tick(in.cam_fps, in.btns, now);

    int32_t fx=in.real_x, fy=in.real_y;

    if (!g_aim_enabled.load()) {
        // 接管关闭: 纯透传 — 不注入任何 counts; 瞄准状态复位
        // (标定激励是程序注入的移动, 与透传互斥), 重新开启后从干净状态起步
        rem_x=rem_y=0; int_x=int_y=0;
    } else if (cal.active) {
        fx=cal.cx; fy=cal.cy;                        // 激励期: 注入激励命令
        rem_x=rem_y=0; int_x=int_y=0;
    } else {
        int aim_mode=g_aim_mode.load();
        bool trig=(aim_mode==2)?(left||right):(aim_mode==1)?right:left;
        if(trig&&!side)last_press=now;
        bool aiming=std::chrono::duration_cast<std::chrono::milliseconds>(
                        now-last_press).count()<=KEEP_ALIVE_MS;
        if (in.pad) out.gate=aiming;                 // 注入门 = 触发保持窗 (接管开关已在上方分流)
        if (aiming) {
            float px,py,vx,vy,le,cs;bool valid;
            std::chrono::steady_clock::time_point tp;
            float ax_e,ay_e,last_dt,last_alpha,last_beta;
            { std::lock_guard<std::mutex> lk(g_target.mtx);
              px=g_target.px;py=g_target.py;vx=g_target.vx;vy=g_target.vy;
              le=g_target.l_est_ms;cs=g_target.cs;
              valid=g_target.valid;tp=g_target.t_pub;
              ax_e=g_target.ax_e;ay_e=g_target.ay_e;
              last_dt=g_target.last_dt;last_alpha=g_target.last_alpha;
              last_beta=g_target.last_beta; }
            double age=elapsed_ms(now,tp);
            if (valid&&age<TARGET_STALE_MS) {
                const float max_v=g_max_v.load(), fov_r=g_fov_radius.load();
                // pad: 逐轴速度帽 = 该轴有效满偏屏速 (注入打满即该轴满偏行程);
                //   spd 调大抬帽, 封顶在热参 -x。hid 两轴同为 -x
                //   (max_vx==max_vy → 下方算式与单帽逐位相同)
                float max_vx=max_v, max_vy=max_v;
                if (in.pad) {
                    max_vx=std::min(max_v,gain_pad_eff(spd_axis(ads,0))/1000.0f);
                    max_vy=std::min(max_v,gain_pad_eff(spd_axis(ads,1))/1000.0f);
                }
                float Lc=le*PRED_L_COMP;
                auto cp=own_motion_ledger().at(shift_ms(tp,-(double)Lc));
                auto cn=own_motion_ledger().cum();
                float ifx=sc.x*(float)(cn.first-cp.first);
                float ify=sc.y*(float)(cn.second-cp.second);
                // 加速度偏差补偿: ε = â·T·(α/β−½) 修 α-β 速度结构滞后,
                //   位置外推加 ½â·W²; 前馈用 v̂+ε — 对匀加速目标, 当前真实
                //   速度才是 type-2 零拖尾的精确开环指令
                float b=std::max(last_beta,1e-9f);
                float eps_x=ax_e*last_dt*(last_alpha/b-0.5f);
                float eps_y=ay_e*last_dt*(last_alpha/b-0.5f);
                float vffx=vx+eps_x, vffy=vy+eps_y;
                float W=(float)age+Lc;
                float ex=px+vffx*W+0.5f*ax_e*W*W-ifx;
                float ey=py+vffy*W+0.5f*ay_e*W*W-ify;
                float r=std::hypot(ex,ey);
                float L=std::max(1.0f,le);
                float wn=(90.0f-FF_PM_DEG)*3.14159265358979f/180.0f/L;
                float kp=2.0f*FF_ZETA*wn;
                float ki=wn*wn;
                float gate=FF_I_GATE/(FF_I_GATE+r);
                float i_lim_x=FF_I_FRAC*max_vx/std::max(ki,1e-9f);
                float i_lim_y=FF_I_FRAC*max_vy/std::max(ki,1e-9f);
                float vx_u=kp*ex+ki*int_x;
                float vy_u=kp*ey+ki*int_y;
                if (ex*ex+ey*ey>fov_r*fov_r) { int_x=int_y=0; }
                else {
                    bool wx=(vx_u>max_vx&&ex>0)||(vx_u<-max_vx&&ex<0);
                    bool wy=(vy_u>max_vy&&ey>0)||(vy_u<-max_vy&&ey<0);
                    if(!wx)int_x=std::clamp(int_x+ex*TICK_MS*gate,-i_lim_x,i_lim_x);
                    if(!wy)int_y=std::clamp(int_y+ey*TICK_MS*gate,-i_lim_y,i_lim_y);
                }
                // FF 门控 = 信任度插值: 信任满格 (稳态追击) → 无门控全力
                // 前馈 (sharp); CUSUM 告警 (模型破缺, 该轴 v̂ 已归零重拉)
                // → 回到距离门控保守形态 (重拉期防二次过冲), 信任按标定
                // L 尺度渐恢复 (无踢脚)。丢帧期按 L 时间尺度额外衰减。
                static float w_state=0;
                float w_inst=cs;
                float rate=(w_inst>w_state)?(1.0f-std::exp(-TICK_MS/(2.0f*PRED_DT0)))
                                           :(1.0f-std::exp(-TICK_MS/std::max(1.0f,L)));
                w_state+=rate*(w_inst-w_state);
                float frame_dt=1000.0f/(float)in.cam_fps;
                float gap_scale=1.0f-std::clamp((float)(age-frame_dt)/std::max(1.0f,L),
                                                0.0f,1.0f);
                float ff_gate=gate+(1.0f-gate)*(1.0f-w_state);
                float ff_eff=FF_GAIN_VAL*ff_gate*gap_scale;
                vx_u+=ff_eff*vffx;
                vy_u+=ff_eff*vffy;
                float vcx=std::clamp(vx_u,-max_vx,max_vx);
                float vcy=std::clamp(vy_u,-max_vy,max_vy);
                if (in.pad) { out.vx=vcx; out.vy=vcy; }   // 缝合: pad 交付期望速度
                else {
                    // hid 落点: 每轴有效 s = 基数×100/spd_axis — 调大 spd = s 变小
                    //   = 发更多 counts = 更快; 与账本→像素换算同一口径
                    rem_x+=vcx*TICK_MS/sc.x; rem_y+=vcy*TICK_MS/sc.y;
                    int sx=std::clamp((int)std::trunc(rem_x),-120,120);
                    int sy=std::clamp((int)std::trunc(rem_y),-120,120);
                    rem_x-=sx;rem_y-=sy; fx+=sx;fy+=sy;
                }
            } else { rem_x=rem_y=0; int_x=int_y=0; }
        } else { rem_x=rem_y=0; int_x=int_y=0; }
    }

    if (!in.pad) {
        // hid 尾巴: 位移钳制 → 报文位移字节 → 账本 (标定期记的是激励的位移)
        fx=std::clamp(fx,-32768,32767); fy=std::clamp(fy,-32768,32767);
        in.rpt[3]=fx&0xFF; in.rpt[4]=fx>>8; in.rpt[5]=fy&0xFF; in.rpt[6]=fy>>8;
        g_counts.add(now,(int)fx,(int)fy);
    }
    // pad: 无报文/counts 尾巴 — 注入偏转与摇杆账本由 io/pad_output.cu 的 pad_merge
    //   按 合并偏转×实际拍时长 入账 (g_counts 不变式 3 的 pad 对应物)
}

} // namespace

void control_apply(int cam_fps, uint8_t* rpt, int16_t real_x, int16_t real_y) {
    LawIn in;
    in.cam_fps=cam_fps; in.real_x=real_x; in.real_y=real_y;
    in.btns=(uint16_t)(rpt[1]|(rpt[2]<<8)); in.rpt=rpt;
    LawOut out;
    law_tick(in,out);
}

bool control_apply_pad(int cam_fps, uint16_t btns, float& out_vx, float& out_vy) {
    LawIn in;
    in.cam_fps=cam_fps; in.btns=btns; in.pad=true;
    LawOut out;
    law_tick(in,out);
    out_vx=out.vx; out_vy=out.vy; return out.gate;
}
