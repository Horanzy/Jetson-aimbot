// ============================================================================
//  control.cu — ff_pi_acc 的控制拍执行 (拍率 = DEFAULT_FREQ): Smith ê 组装 (含 â 的 ε 修正与 ½â·W²
//    外推) → 极点配置 PI (条件积分 + 距离门控) → type-2 速度前馈 (信任度插值
//    门控 + 检测间隙衰减); 双侧键触发的标定状态机 (cal=0..6, 激励轨迹表见
//    core/calib.h) 也在此驱动。跨帧控制状态 (积分器/状态机相位/量化余量) 为
//    law_tick 内 static。律输出在执行器缝合点分流: hid = counts 量化进报文位
//    移字节并记 g_counts; pad = 交付期望速度 (px/ms), 量化/报文/counts 尾巴
//    是 hid 专属。自身运动补偿账本来源与账本→像素比例随模式路由 (io/pad_output.h:
//    hid = g_counts + 标定 s, pad = 摇杆账本 + 逐轴满偏屏速换算), 律数学两模式
//    逐句一致 — hid 的 max_vx==max_vy 使逐轴帽退化为单帽, 算术逐位不变。
// ============================================================================

#include "core/control.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "core/calib.h"
#include "core/state.h"
#include "io/pad_output.h"     // own_motion_ledger/own_motion_scale + g_pad_stick_gain_x/y:
                               //   自身运动账本来源与 pad 速度帽随输出模式

namespace {

// 统一控制拍: hid 经 HID 执行 (量化 + 报文位移 + g_counts 记账), pad 交付期
//   望速度并跳过 hid 专属尾巴 (鼠标侧键抑制/双侧键标定/报文写盘 — pad 键位
//   字恒无 SIDE_KEY/BOTH_SIDE_KEYS 位, 侧键抑制恒过、标定分支自然不可达)。
void law_tick(int cam_fps, int16_t real_x, int16_t real_y, uint16_t btns, bool pad,
              uint8_t* rpt, float* out_vx, float* out_vy, bool* out_gate) {
    static auto last_press=std::chrono::steady_clock::now()-std::chrono::hours(1);
    static float rem_x=0,rem_y=0;
    static float int_x=0,int_y=0;
    static int cal=0,hold=0;
    static const CalibSeg* seq=nullptr;
    static int slen=0,si=0,st=0,wt=0;
    static std::vector<CalibSeg> excite;

    auto now=std::chrono::steady_clock::now();
    bool left=btns&LEFT_KEY, right=btns&RIGHT_KEY, side=btns&SIDE_KEY;
    g_left_down.store(left);

    int32_t fx=real_x, fy=real_y;

    if (!g_aim_enabled.load()) {
        // 接管关闭: 纯透传 — 不注入任何 counts; 标定/瞄准状态机复位
        // (标定激励是程序注入的移动, 与透传互斥), 重新开启后从干净状态起步
        if (cal!=0) { cal=0; g_calib_collect=false;
                      seq=nullptr; slen=si=st=wt=0; }
        hold=0; rem_x=rem_y=0; int_x=int_y=0;
    } else if (cal==3) {
        fx=fy=0;
        int done=g_calib_done.load();
        if (done!=0||++wt>CALIB_WAIT_TIMEOUT) {
            seq=done==1?CAL_END_OK_SEQ:CAL_END_FAIL_SEQ;
            slen=done==1?(int)(sizeof(CAL_END_OK_SEQ)/sizeof(CalibSeg))
                       :(int)(sizeof(CAL_END_FAIL_SEQ)/sizeof(CalibSeg));
            cal=done==1?4:5; si=st=0; }
    } else if (cal!=0) {
        if (si<slen) { auto& sg=seq[si]; fx=sg.dx;fy=sg.dy;
            if(++st>=sg.ticks){st=0;++si;} }
        if (si>=slen) {
            if (cal==1) { excite.clear();
                constexpr int neseg=(int)(sizeof(CAL_EXCITE_SEQ)/sizeof(CalibSeg));
                for(int i=0;i<5;++i) for(int j=0;j<neseg;++j) excite.push_back(CAL_EXCITE_SEQ[j]);
                seq=excite.data();slen=(int)excite.size();si=st=0;
                g_calib_collect=true;cal=2;
            } else if (cal==2) { seq=CAL_SETTLE_SEQ;slen=1;si=st=0;cal=6;fx=fy=0;
            } else if (cal==6) { g_calib_collect=false;g_calib_done=0;
                g_calib_request=true;wt=0;cal=3;fx=fy=0;
            } else { cal=0;fx=real_x;fy=real_y; } }
        rem_x=rem_y=0;
    } else {
        int aim_mode=g_aim_mode.load();
        bool trig=(aim_mode==2)?(left||right):(aim_mode==1)?right:left;
        if(trig&&!side)last_press=now;
        bool aiming=std::chrono::duration_cast<std::chrono::milliseconds>(
                        now-last_press).count()<=KEEP_ALIVE_MS;
        if (pad) *out_gate=aiming;              // 注入门 = 触发保持窗 (接管开关已在上方分流)
        if (aiming) {
            float px,py,vx,vy,se,le,cs;bool valid;
            std::chrono::steady_clock::time_point tp;
            float ax_e,ay_e,last_dt,last_alpha,last_beta;
            { std::lock_guard<std::mutex> lk(g_target.mtx);
              px=g_target.px;py=g_target.py;vx=g_target.vx;vy=g_target.vy;
              se=g_target.s_est;le=g_target.l_est_ms;cs=g_target.cs;
              valid=g_target.valid;tp=g_target.t_pub;
              ax_e=g_target.ax_e;ay_e=g_target.ay_e;
              last_dt=g_target.last_dt;last_alpha=g_target.last_alpha;
              last_beta=g_target.last_beta; }
            double age=elapsed_ms(now,tp);
            if (valid&&age<TARGET_STALE_MS) {
                float max_v=g_max_v.load(); const float fov_r=g_fov_radius.load();
                // pad: 逐轴物理速度帽 = 该轴满偏转屏速 (注入通道打满即该轴满偏行程);
                //   hid 两轴同为 -x (max_vx==max_vy → 下方算式与单帽逐位相同)
                float max_vx=max_v, max_vy=max_v;
                if (pad) { max_vx=std::min(max_v,g_pad_stick_gain_x.load()/1000.0f);
                           max_vy=std::min(max_v,g_pad_stick_gain_y.load()/1000.0f); }
                const LedgerPxScale sc=own_motion_scale(se);
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
                float frame_dt=1000.0f/(float)cam_fps;
                float gap_scale=1.0f-std::clamp((float)(age-frame_dt)/std::max(1.0f,L),
                                                0.0f,1.0f);
                float ff_gate=gate+(1.0f-gate)*(1.0f-w_state);
                float ff_eff=FF_GAIN_VAL*ff_gate*gap_scale;
                vx_u+=ff_eff*vffx;
                vy_u+=ff_eff*vffy;
                float vcx=std::clamp(vx_u,-max_vx,max_vx);
                float vcy=std::clamp(vy_u,-max_vy,max_vy);
                if (pad) { *out_vx=vcx; *out_vy=vcy; }    // 缝合: pad 交付期望速度 (px/ms)
                else {
                    float s=std::clamp(se,S_MIN,S_MAX);
                    rem_x+=vcx*TICK_MS/s; rem_y+=vcy*TICK_MS/s;
                    int sx=std::clamp((int)std::trunc(rem_x),-120,120);
                    int sy=std::clamp((int)std::trunc(rem_y),-120,120);
                    rem_x-=sx;rem_y-=sy; fx+=sx;fy+=sy;
                }
            } else { rem_x=rem_y=0; int_x=int_y=0; }
        } else { rem_x=rem_y=0; int_x=int_y=0; }

        if ((btns&BOTH_SIDE_KEYS)==BOTH_SIDE_KEYS) {
            if(++hold>=CALIB_TRIGGER_TICKS){hold=0;cal=1;
                seq=CAL_START_SEQ;slen=(int)(sizeof(CAL_START_SEQ)/sizeof(CalibSeg));
                si=st=0;std::cout<<"[标定] 触发\n";}
        } else hold=0;
    }

    if (pad) {
        // pad: 无报文/counts 尾巴 — 注入偏转与摇杆账本由 io/pad_output.cu 的
        //   pad_merge 按 合并偏转×实际拍时长 入账 (g_counts 不变式 3 的 pad 对应物)
    } else {
        fx=std::clamp(fx,-32768,32767); fy=std::clamp(fy,-32768,32767);
        rpt[3]=fx&0xFF;rpt[4]=fx>>8; rpt[5]=fy&0xFF;rpt[6]=fy>>8;
        g_counts.add(now,(int)fx,(int)fy);
    }
}

} // namespace

void control_apply(int cam_fps, uint8_t* rpt, int16_t real_x, int16_t real_y) {
    law_tick(cam_fps,real_x,real_y,(uint16_t)(rpt[1]|(rpt[2]<<8)),false,rpt,nullptr,nullptr,nullptr);
}

bool control_apply_pad(int cam_fps, uint16_t btns, float& out_vx, float& out_vy) {
    float vx=0,vy=0; bool gate=false;
    law_tick(cam_fps,0,0,btns,true,nullptr,&vx,&vy,&gate);
    out_vx=vx; out_vy=vy; return gate;
}
