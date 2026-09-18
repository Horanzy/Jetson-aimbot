// ============================================================================
//  estimator.cu — estimator_step 的实现: 延迟补偿预测 → 清洗创新 → σ 自标定
//    CUSUM/â 传感器更新 → 滤波位置/速度推进 → TargetState 发布 (无检测帧只
//    发布 valid=false)。
// ============================================================================

#include "core/estimator.h"

#include <algorithm>
#include <cmath>
#include <mutex>

#include "core/state.h"

// â = ȳ·β/T² (创新均值自洽反演加速度, 任何帧率下都精确); ȳ 不过白噪声显著性地板
//   (ACC_SNR·σ_noise·√(ρ/(2−ρ)), σ_noise² = m₂−ȳ² 精确分解) → â = 0 (硬门限)
static inline float accel_est(float ybar, float m2, float beta, float dt,
                              float rho, float lim_a) {
    float sig_n = std::sqrt(std::max(1e-6f, m2 - ybar * ybar));
    float floor = ACC_SNR * sig_n * std::sqrt(rho / (2.0f - rho));
    float eff = (std::fabs(ybar) > floor) ? ybar : 0.0f;
    return std::clamp(eff * beta / (dt * dt), -lim_a, lim_a);
}

float estimator_step(EstimatorState& st, std::chrono::steady_clock::time_point now,
                     bool found, float best_dx, float best_dy,
                     float s_est, float l_est, float max_v) {
    float dt=(float)elapsed_ms(now,st.t_prev); st.t_prev=now;
    dt=std::clamp(dt,1.0f,100.0f);

    if (found) {
        if (!st.filt_init) { st.fx=best_dx;st.fy=best_dy;st.fvx=0;st.fvy=0;st.filt_init=true;
                              st.sig2x=st.sig2y=1;st.csx=st.csy=0;
                              st.sig2rx=st.sig2ry=1;st.ybar_x=st.ybar_y=0;st.ax_e=st.ay_e=0; }
        else {
            float Lc=l_est*PRED_L_COMP;
            auto c0=g_counts.at(shift_ms(now,-(double)Lc-dt));
            auto c1=g_counts.at(shift_ms(now,-(double)Lc));
            float cax=(float)(c1.first-c0.first), cay=(float)(c1.second-c0.second);
            float px_pred=st.fx+st.fvx*dt-s_est*cax, py_pred=st.fy+st.fvy*dt-s_est*cay;
            float inx=best_dx-px_pred, iny=best_dy-py_pred;
            if (std::hypot(inx,iny)>TRACK_JUMP_GATE) { st.fx=best_dx;st.fy=best_dy;st.fvx=0;st.fvy=0;
                st.csx=st.csy=0;
                // 跳变 = 目标模型破缺: v̂ 从 0 重建, â 传感器一并清零抑制
                st.sig2rx=st.sig2ry=1;st.ybar_x=st.ybar_y=0;st.ax_e=st.ay_e=0;
                st.reb_x=st.reb_y=true;
                float rb_hold=ACC_RB_HOLD_N*dt
                    /std::max(1e-6f,std::min(0.60f,PRED_BETA0*dt/PRED_DT0));
                st.reb_until_x=st.reb_until_y=shift_ms(now,rb_hold); }
            else { float rr=dt/PRED_DT0;
                   float alpha=std::min(0.90f,PRED_ALPHA0*rr);
                   float beta=std::min(0.60f,PRED_BETA0*rr);
                   st.sig2x+=beta*(inx*inx-st.sig2x);
                   st.sig2y+=beta*(iny*iny-st.sig2y);
                   // 方向矛盾 CUSUM (σ 自标定): 持续矛盾创新 → 告警后该轴 v̂ 归零重拉
                   float sx=std::max(std::sqrt(st.sig2x),1e-6f);
                   float sy=std::max(std::sqrt(st.sig2y),1e-6f);
                   // â 传感器稳健尺度 (Huber 截断二阶矩): 只喂 â 显著性与重建判定
                   float srx=std::max(std::sqrt(st.sig2rx),1e-6f);
                   float sry=std::max(std::sqrt(st.sig2ry),1e-6f);
                   // 清洗创新: 减掉自身已知的 Lc 过补偿伪迹 (matched 下恰好还原
                   //   真实目标创新; 失配残留 ∝ Δ·a_own, 瞬态成对, 由活动门吸收)
                   auto c0n=g_counts.at(shift_ms(now,-(double)l_est-dt));
                   auto c1n=g_counts.at(shift_ms(now,-(double)l_est));
                   float inx_c=inx-s_est*((c1.first-c0.first)-(float)(c1n.first-c0n.first));
                   float iny_c=iny-s_est*((c1.second-c0.second)-(float)(c1n.second-c0n.second));
                   float clx=std::clamp(inx_c,-ACC_SIG_CLIP_K*srx,ACC_SIG_CLIP_K*srx);
                   float cly=std::clamp(iny_c,-ACC_SIG_CLIP_K*sry,ACC_SIG_CLIP_K*sry);
                   st.sig2rx+=beta*(clx*clx-st.sig2rx);
                   st.sig2ry+=beta*(cly*cly-st.sig2ry);
                   // 自身加速度活动门: 失配伪创新 ∝ a_own·Δ 与真签名 (∝a_t·T²/β)
                   //   物理可分 — 自身剧烈加减速期间 â 不采信
                   float w_own=std::max(1.0f,l_est);
                   auto s0=g_counts.at(now);
                   auto s1=g_counts.at(shift_ms(now,-(double)w_own));
                   auto s2=g_counts.at(shift_ms(now,-(double)dt));
                   auto s3=g_counts.at(shift_ms(now,-(double)dt-(double)w_own));
                   float th_a=max_v/(ACC_OW_ACTIV_K*std::max(1.0f,l_est));
                   float aown_x=(s_est*((float)(s0.first-s1.first)
                                       -(float)(s2.first-s3.first))/w_own)/dt;
                   float aown_y=(s_est*((float)(s0.second-s1.second)
                                       -(float)(s2.second-s3.second))/w_own)/dt;
                   float tx=aown_x/th_a, ty=aown_y/th_a;
                   float gx_own=1.0f/(1.0f+tx*tx*tx*tx*tx*tx);
                   float gy_own=1.0f/(1.0f+ty*ty*ty*ty*ty*ty);
                   // 重建旗标解除 = 时间常数下限 + 创新回典型水平
                   if (st.reb_x && now>=st.reb_until_x && std::fabs(inx)<=ACC_SIG_CLIP_K*srx)
                       st.reb_x=false;
                   if (st.reb_y && now>=st.reb_until_y && std::fabs(iny)<=ACC_SIG_CLIP_K*sry)
                       st.reb_y=false;
                   // 创新均值 EMA ȳ — 无新鲜证据 → 以自然速率向零衰减;
                   //   有证据 → 按 (自身活动门 × CUSUM 矛盾门) 缩放进入
                   float rho=1.0f-std::exp(-dt/(ACC_TAU_L*std::max(1.0f,l_est)));
                   if (st.reb_x) st.ybar_x-=rho*st.ybar_x;
                   else { float g=gx_own*(1.0f-std::min(1.0f,st.csx/CUSUM_H));
                          st.ybar_x+=rho*(g*clx-st.ybar_x); }
                   if (st.reb_y) st.ybar_y-=rho*st.ybar_y;
                   else { float g=gy_own*(1.0f-std::min(1.0f,st.csy/CUSUM_H));
                          st.ybar_y+=rho*(g*cly-st.ybar_y); }
                   float accx=(st.fvx>0)?-inx/sx:(st.fvx<0)?inx/sx:-CUSUM_K;
                   float accy=(st.fvy>0)?-iny/sy:(st.fvy<0)?iny/sy:-CUSUM_K;
                   st.csx=std::max(0.0f,st.csx+std::min(std::max(accx,0.0f),CUSUM_C)-CUSUM_K);
                   st.csy=std::max(0.0f,st.csy+std::min(std::max(accy,0.0f),CUSUM_C)-CUSUM_K);
                   if (st.csx>=CUSUM_H && st.fvx!=0) { st.fvx=0; st.csx=0;
                       st.reb_x=true; st.reb_until_x=shift_ms(now,ACC_RB_HOLD_N*dt/beta); }
                   if (st.csy>=CUSUM_H && st.fvy!=0) { st.fvy=0; st.csy=0;
                       st.reb_y=true; st.reb_until_y=shift_ms(now,ACC_RB_HOLD_N*dt/beta); }
                   st.fx=px_pred+alpha*inx; st.fy=py_pred+alpha*iny;
                   st.fvx+=(beta/dt)*inx; st.fvy+=(beta/dt)*iny;
                   float lim_a=max_v/(ACC_TAU_L*std::max(1.0f,l_est));
                   st.ax_e=accel_est(st.ybar_x,st.sig2rx,beta,dt,rho,lim_a);
                   st.ay_e=accel_est(st.ybar_y,st.sig2ry,beta,dt,rho,lim_a);
                   if (st.reb_x) st.ax_e=0;
                   if (st.reb_y) st.ay_e=0;
                   st.last_dt=dt; st.last_alpha=alpha; st.last_beta=beta; }
        }
        { std::lock_guard<std::mutex> lk(g_target.mtx);
          g_target.px=st.fx;g_target.py=st.fy;g_target.vx=st.fvx;g_target.vy=st.fvy;
          g_target.ax_e=st.ax_e;g_target.ay_e=st.ay_e;
          g_target.last_dt=st.last_dt;g_target.last_alpha=st.last_alpha;
          g_target.last_beta=st.last_beta;
          g_target.cs=std::max(st.csx,st.csy)/CUSUM_H;
          g_target.s_est=s_est;g_target.l_est_ms=l_est;
          g_target.t_pub=now;g_target.valid=true; }
    } else {
        std::lock_guard<std::mutex> lk(g_target.mtx); g_target.valid=false;
    }
    return dt;
}
