// ============================================================================
//  control.cu — ff_pi_acc 的 500Hz 执行: Smith ê 组装 (含 â 的 ε 修正与 ½â·W²
//    外推) → 极点配置 PI (条件积分 + 距离门控) → type-2 速度前馈 (信任度插值
//    门控 + 检测间隙衰减); 双侧键触发的标定状态机 (cal=0..6, 激励轨迹表见
//    core/calib.h) 也在此驱动。跨帧控制状态 (积分器/状态机相位) 为函数内
//    static; counts 量化结果直接写入报文位移字节并记入 g_counts。
// ============================================================================

#include "core/control.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "core/calib.h"
#include "core/state.h"

void control_apply(int cam_fps, uint8_t* rpt, int16_t real_x, int16_t real_y) {
    static auto last_press=std::chrono::steady_clock::now()-std::chrono::hours(1);
    static float rem_x=0,rem_y=0;
    static float int_x=0,int_y=0;
    static int cal=0,hold=0;
    static const CalibSeg* seq=nullptr;
    static int slen=0,si=0,st=0,wt=0;
    static std::vector<CalibSeg> excite;

    auto now=std::chrono::steady_clock::now();
    uint16_t btns=rpt[1]|(rpt[2]<<8);
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
                for(int i=0;i<5;++i){excite.push_back({4,0,125});excite.push_back({0,4,125});
                    excite.push_back({-4,0,125});excite.push_back({0,-4,125});}
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
                const float max_v=g_max_v.load(), fov_r=g_fov_radius.load();
                float Lc=le*PRED_L_COMP;
                auto cp=g_counts.at(shift_ms(tp,-(double)Lc));
                auto cn=g_counts.cum();
                float ifx=se*(float)(cn.first-cp.first);
                float ify=se*(float)(cn.second-cp.second);
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
                float i_lim=FF_I_FRAC*max_v/std::max(ki,1e-9f);
                float vx_u=kp*ex+ki*int_x;
                float vy_u=kp*ey+ki*int_y;
                if (ex*ex+ey*ey>fov_r*fov_r) { int_x=int_y=0; }
                else {
                    bool wx=(vx_u>max_v&&ex>0)||(vx_u<-max_v&&ex<0);
                    bool wy=(vy_u>max_v&&ey>0)||(vy_u<-max_v&&ey<0);
                    if(!wx)int_x=std::clamp(int_x+ex*TICK_MS*gate,-i_lim,i_lim);
                    if(!wy)int_y=std::clamp(int_y+ey*TICK_MS*gate,-i_lim,i_lim);
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
                float vcx=std::clamp(vx_u,-max_v,max_v);
                float vcy=std::clamp(vy_u,-max_v,max_v);
                float s=std::clamp(se,S_MIN,S_MAX);
                rem_x+=vcx*TICK_MS/s; rem_y+=vcy*TICK_MS/s;
                int sx=std::clamp((int)std::trunc(rem_x),-120,120);
                int sy=std::clamp((int)std::trunc(rem_y),-120,120);
                rem_x-=sx;rem_y-=sy; fx+=sx;fy+=sy;
            } else { rem_x=rem_y=0; int_x=int_y=0; }
        } else { rem_x=rem_y=0; int_x=int_y=0; }

        if ((btns&BOTH_SIDE_KEYS)==BOTH_SIDE_KEYS) {
            if(++hold>=CALIB_TRIGGER_TICKS){hold=0;cal=1;
                seq=CAL_START_SEQ;slen=(int)(sizeof(CAL_START_SEQ)/sizeof(CalibSeg));
                si=st=0;std::cout<<"[标定] 触发\n";}
        } else hold=0;
    }

    fx=std::clamp(fx,-32768,32767); fy=std::clamp(fy,-32768,32767);
    rpt[3]=fx&0xFF;rpt[4]=fx>>8; rpt[5]=fy&0xFF;rpt[6]=fy>>8;
    g_counts.add(now,(int)fx,(int)fy);
}
