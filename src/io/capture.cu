// ============================================================================
//  capture.cu — ai_thread 的实现: 引擎反序列化与张量绑定, GStreamer 管道搭建
//    (整幅预览 / 居中裁剪两种管道), 逐帧采集 → 预处理 → 推理 → 输出解析 →
//    NMS → FOV 目标筛选 → estimator_step; 标定采样 (半分辨率 3×3 块相位
//    相关) 与 g_calib_request 驱动的标定计算/回写 (hid: S_EST/L_EST, pad:
//    PAD_STICK_GAIN/L_EST_PAD — 单位制与 VAR 名随输出模式), 三源截图入队与预览叠加。
// ============================================================================

#include "io/capture.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <thread>
#include <vector>

#include <cuda_fp16.h>

#include "core/calib.h"
#include "core/estimator.h"
#include "core/state.h"
#include "core/trt.h"
#include "io/pad_calib.h"      // pad 标定: 钳制带/换算/回写 VAR 名与设计带判定

// conf / y_off / fov 每帧从热参数原子取快照 (帧内一致), 未收热参时值与 CLI 一致
void ai_thread(std::string model_path, int target_cls,
               std::string cam_dev, int cam_fps, bool preview,
               float init_s, float init_l, std::string persist_path,
               std::string out_dir, int fire_ms, double auto_s,
               int cooldown_ms, int jpeg_quality, bool pad_mode) {
    const int cam_w=1920, cam_h=1080;
    const float nms_iou_thr=0.45f;
    const bool collecting_enabled = !out_dir.empty();

    std::ifstream ef(model_path, std::ios::binary);
    if (!ef.good()) { std::cerr<<"AI: 无法打开模型\n"; global_running=false; return; }
    ef.seekg(0,std::ios::end); size_t esz=ef.tellg(); ef.seekg(0,std::ios::beg);
    std::vector<char> edata(esz); ef.read(edata.data(),esz); ef.close();

    std::unique_ptr<nvinfer1::IRuntime,TRTDestroy> rt(nvinfer1::createInferRuntime(gLogger));
    std::unique_ptr<nvinfer1::ICudaEngine,TRTDestroy> eng(rt->deserializeCudaEngine(edata.data(),esz));
    std::unique_ptr<nvinfer1::IExecutionContext,TRTDestroy> ctx(eng->createExecutionContext());

    int nio=eng->getNbIOTensors(); std::string in_name,out_name;
    for (int i=0;i<nio;++i) { const char* nm=eng->getIOTensorName(i);
        auto m=eng->getTensorIOMode(nm);
        if (m==nvinfer1::TensorIOMode::kINPUT&&in_name.empty()) in_name=nm;
        if (m==nvinfer1::TensorIOMode::kOUTPUT&&out_name.empty()) out_name=nm; }

    auto in_dims=eng->getTensorShape(in_name.c_str());
    auto in_dt=eng->getTensorDataType(in_name.c_str());
    auto out_dt=eng->getTensorDataType(out_name.c_str());
    if (hasDynamicDim(in_dims)) {
        auto opt=eng->getProfileShape(in_name.c_str(),0,nvinfer1::OptProfileSelector::kOPT);
        ctx->setInputShape(in_name.c_str(),opt); }
    auto real_in=ctx->getTensorShape(in_name.c_str());
    int iw=real_in.d[3], ih=real_in.d[2];
    int cap_w=std::max(CAP_SIZE,iw), cap_h=std::max(CAP_SIZE,ih);
    float cx0=cap_w/2.0f, cy0=cap_h/2.0f;

    auto out_dims=ctx->getTensorShape(out_name.c_str());
    if (volume(out_dims)==0) out_dims=eng->getTensorShape(out_name.c_str());
    OutputLayout ol; parseOutputLayout(out_dims,ol);
    long G=(long)(iw/8)*(ih/8)+(long)(iw/16)*(ih/16)+(long)(iw/32)*(ih/32);
    bool end2end=(ol.attrs==6&&!ol.attrs_first&&ol.num==300);
    bool yolov5=(!end2end&&ol.num==3*G);
    int nclass=ol.attrs-(yolov5?5:4);
    std::cout<<"模型: "<<(end2end?"端到端":yolov5?"YOLOv5":"YOLOv8/11")
             <<" "<<iw<<"x"<<ih;
    if (!end2end) std::cout<<" "<<nclass<<"类";
    std::cout<<"\n";

    size_t ivol=volume(real_in),ovol=volume(out_dims);
    size_t ibytes=ivol*elemSize(in_dt),obytes=ovol*elemSize(out_dt);
    size_t bgr_bytes=(size_t)cap_w*cap_h*3;
    void *d_in=nullptr,*d_out=nullptr,*d_bgr=nullptr,*d_model_bgr=nullptr,*h_out=nullptr;
    CHECK_CUDA(cudaMalloc(&d_in,ibytes)); CHECK_CUDA(cudaMalloc(&d_out,obytes));
    CHECK_CUDA(cudaMalloc(&d_bgr,bgr_bytes));
    bool need_crop=(iw!=cap_w||ih!=cap_h);
    if (need_crop) CHECK_CUDA(cudaMalloc(&d_model_bgr,(size_t)iw*ih*3));
    else d_model_bgr=d_bgr;
    int m_off_x=(cap_w-iw)/2, m_off_y=(cap_h-ih)/2;
    CHECK_CUDA(cudaHostAlloc(&h_out,obytes,cudaHostAllocDefault));
    std::vector<float> ofbuf; if (out_dt==nvinfer1::DataType::kHALF) ofbuf.resize(ovol);
    cudaStream_t stream; CHECK_CUDA(cudaStreamCreate(&stream));
    ctx->setTensorAddress(in_name.c_str(),d_in);
    ctx->setTensorAddress(out_name.c_str(),d_out);

    int crop_x=(cam_w-cap_w)/2, crop_y=(cam_h-cap_h)/2;
    std::string base="v4l2src device="+cam_dev+" ! video/x-raw,format=NV12,width="
        +std::to_string(cam_w)+",height="+std::to_string(cam_h)+",framerate="
        +std::to_string(cam_fps)+"/1 ! ";
    std::string full_pipe=base+"nvvidconv ! video/x-raw,format=BGRx ! videoconvert "
        "! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false";
    std::string crop_pipe=base+"nvvidconv left="+std::to_string(crop_x)
        +" right="+std::to_string(crop_x+cap_w)+" top="+std::to_string(crop_y)
        +" bottom="+std::to_string(crop_y+cap_h)+" ! video/x-raw,width="+std::to_string(cap_w)
        +",height="+std::to_string(cap_h)+",format=BGRx ! videoconvert "
        "! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false";

    cv::VideoCapture cap;
    cap.open(preview?full_pipe:crop_pipe, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) { std::cerr<<"无法打开摄像头\n"; global_running=false; return; }
    if (preview) cv::namedWindow("Aimbot",cv::WINDOW_AUTOSIZE);
    std::cout<<"✅ AI 线程已启动 ("<<cam_fps<<" fps, "<<cam_dev<<")\n";

    EstimatorState est;

    float s_est=init_s, l_est=init_l;
    // 标定的灵敏度钳制带与回写 VAR 名随输出模式 (单位制不同, 机制同一); 账本
    //   来源经 own_motion_ledger 路由, 与模式无关地走 run_calibration 同一段数学
    const CalibBand calib_band = pad_mode ? CALIB_BAND_PAD : CALIB_BAND_COUNTS;
    std::deque<CalibSample> hist; bool was_collecting=false;
    int collect_frames=0;
    int bs_w=cap_w/6, bs_h=cap_h/6;
    cv::Mat hann; cv::createHanningWindow(hann,cv::Size(bs_w,bs_h),CV_32F);
    cv::Mat prev_gray_f;

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> jitter(0.5,1.5);
    auto roll_auto=[&](){ return std::chrono::milliseconds((long)(auto_s*1000.0*(jitter(rng)))); };
    auto t_start=std::chrono::steady_clock::now();
    auto next_auto=t_start+roll_auto();
    auto last_save=t_start-std::chrono::hours(1);
    auto last_fire=t_start-std::chrono::hours(1);
    long n_fire=0, n_det=0, n_auto=0;

    cv::Mat frame, cap_img;
    int read_fails=0;
    long fps_cnt=0; auto fps_t0=std::chrono::steady_clock::now();

    while (global_running) {
        if (!cap.read(frame)) {
            if (++read_fails>30) { std::cerr<<"AI: 采集卡断开\n"; global_running=false; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); continue; }
        read_fails=0;
        const float conf_thr=g_conf_thr.load(), y_off_pct=g_y_off_pct.load(),
                    fov_r=g_fov_radius.load(), max_v=g_max_v.load();

        ++fps_cnt;
        auto fps_now=std::chrono::steady_clock::now();
        double fps_dt=std::chrono::duration<double>(fps_now-fps_t0).count();
        if (fps_dt>=60.0) { std::cout<<"[AI FPS] "<<(int)(fps_cnt/fps_dt)<<" fps (上限 "
                            <<cam_fps<<")\n"; fps_cnt=0; fps_t0=fps_now; }

        cap_img=(preview&&frame.cols>cap_w)?frame(cv::Rect(crop_x,crop_y,cap_w,cap_h)):frame;

        CHECK_CUDA(cudaMemcpy2DAsync(d_bgr,(size_t)cap_w*3,
                     cap_img.data,cap_img.step,
                     (size_t)cap_w*3,cap_h,cudaMemcpyHostToDevice,stream));
        if (need_crop) {
            const uint8_t* src=(const uint8_t*)d_bgr+(size_t)m_off_y*cap_w*3+m_off_x*3;
            CHECK_CUDA(cudaMemcpy2DAsync(d_model_bgr,(size_t)iw*3,
                         src,(size_t)cap_w*3,
                         (size_t)iw*3,ih,cudaMemcpyDeviceToDevice,stream));
        }
        launchPreprocess((const uint8_t*)d_model_bgr,d_in,iw,ih,
                         in_dt==nvinfer1::DataType::kFLOAT,stream);
        ctx->enqueueV3(stream);
        CHECK_CUDA(cudaMemcpyAsync(h_out,d_out,obytes,cudaMemcpyDeviceToHost,stream));
        CHECK_CUDA(cudaStreamSynchronize(stream));

        const float* od=nullptr;
        if (out_dt==nvinfer1::DataType::kFLOAT) od=(const float*)h_out;
        else { auto* ho=(const __half*)h_out;
               for (size_t i=0;i<ovol;++i) ofbuf[i]=__half2float(ho[i]); od=ofbuf.data(); }

        std::vector<Detection> raw;
        for (int i=0;i<ol.num;++i) {
            Detection d;
            if (end2end) {
                d.conf=outVal(od,ol,4,i);
                d.class_id=(int)std::round(outVal(od,ol,5,i));
                if (d.conf<conf_thr||d.class_id!=target_cls) continue;
                float x1=outVal(od,ol,0,i),y1=outVal(od,ol,1,i),
                      x2=outVal(od,ol,2,i),y2=outVal(od,ol,3,i);
                d.cx=(x1+x2)*.5f;d.cy=(y1+y2)*.5f;d.w=x2-x1;d.h=y2-y1;
            } else if (yolov5) {
                float obj=outVal(od,ol,4,i),mc=0;int ci=-1;
                for(int c=0;c<nclass;++c){float s=obj*outVal(od,ol,5+c,i);if(s>mc){mc=s;ci=c;}}
                if(mc<conf_thr||ci!=target_cls)continue;
                d.cx=outVal(od,ol,0,i);d.cy=outVal(od,ol,1,i);
                d.w=outVal(od,ol,2,i);d.h=outVal(od,ol,3,i);d.conf=mc;d.class_id=ci;
            } else {
                float mc=0;int ci=-1;
                for(int c=0;c<nclass;++c){float s=outVal(od,ol,4+c,i);if(s>mc){mc=s;ci=c;}}
                if(mc<conf_thr||ci!=target_cls)continue;
                d.cx=outVal(od,ol,0,i);d.cy=outVal(od,ol,1,i);
                d.w=outVal(od,ol,2,i);d.h=outVal(od,ol,3,i);d.conf=mc;d.class_id=ci;
            }
            raw.push_back(d);
        }
        auto filtered=nms(raw,nms_iou_thr);
        if (need_crop) for (auto& d:filtered) {
            d.cx+=m_off_x; d.cy+=m_off_y; }

        bool cal_collecting=g_calib_collect.load();
        if (cal_collecting&&!was_collecting) { hist.clear(); prev_gray_f.release();
            collect_frames=0; }
        was_collecting=cal_collecting;

        float best_dist=1e9f,best_dx=0,best_dy=0; bool found=false;
        for (auto& d:filtered) {
            float ty=d.cy+d.h*(0.5f-y_off_pct/100.0f);
            float dx=d.cx-cx0, dy=ty-cy0, dist=std::sqrt(dx*dx+dy*dy);
            if (dist<best_dist&&dist<fov_r) { best_dist=dist;best_dx=dx;best_dy=dy;found=true; } }

        auto now=std::chrono::steady_clock::now();
        float dt=estimator_step(est,now,found,best_dx,best_dy,s_est,l_est,max_v);

        if (cal_collecting) {
            cv::Mat gray,small,sf;
            cv::cvtColor(cap_img,gray,cv::COLOR_BGR2GRAY);
            cv::resize(gray,small,cv::Size(cap_w/2,cap_h/2),0,0,cv::INTER_AREA);
            small.convertTo(sf,CV_32F);
            if (!prev_gray_f.empty()) {
                float shx[9],shy[9];int nv=0;
                for(int by=0;by<3;++by)for(int bx=0;bx<3;++bx){
                    cv::Rect r(bx*bs_w,by*bs_h,bs_w,bs_h); double resp=0;
                    cv::Point2d sh=cv::phaseCorrelate(prev_gray_f(r),sf(r),hann,&resp);
                    if(resp>0.01){shx[nv]=(float)sh.x;shy[nv]=(float)sh.y;++nv;} }
                ++collect_frames;
                if (nv>=4) { std::nth_element(shx,shx+nv/2,shx+nv);
                             std::nth_element(shy,shy+nv/2,shy+nv);
                             hist.push_back({now,dt,-2.0f*shx[nv/2],-2.0f*shy[nv/2]});
                             if((int)hist.size()>300)hist.pop_front(); }
            }
            prev_gray_f=sf;
        }

        if (g_calib_request.exchange(false)) {
            bool ok=false;
            for(int it=0;it<8;++it) ok|=run_calibration(hist,s_est,l_est,calib_band);
            float gain=0; bool oob=false;
            if (ok && pad_mode) {
                // pad: 拟合出的是 px per (偏转·ms), 换算成满偏转屏速 px/s
                //   (×32767×1000)。带外 = 激励/背景不可信 (钳制把拟合停在带边),
                //   按失败收尾 — 带边垃圾值不回写。
                gain=pad_gain_from_s_rp(s_est);
                oob=!pad_calib_accept(gain);
                if (oob) { ok=false;
                    std::cout<<"[标定] 失败: 摇杆增益越界 (拟合钳到 "<<gain<<" px/s, 设计带 ["
                             <<PAD_GAIN_MIN<<","<<PAD_GAIN_MAX<<"])\n"; }
                else s_est=pad_s_rp_from_gain(gain);   // 与运行期增益同一来源
            }
            g_calib_done=ok?1:2;
            if (ok) {
                if (pad_mode) {
                    g_pad_stick_gain.store(gain);       // 立即对 pad 拍生效
                    std::cout<<"[标定] stick_gain="<<gain<<" px/s, L="<<l_est<<" ms\n";
                    if (!persist_path.empty()) {
                        if (persist_calibration(persist_path,PAD_CAL_VAR_GAIN,gain,
                                                PAD_CAL_VAR_L,l_est))
                            std::cout<<"[标定] 已回写 "<<persist_path<<"\n";
                        else std::cerr<<"[标定] 回写失败\n"; }
                } else {
                    std::cout<<"[标定] s="<<s_est<<" px/count, L="<<l_est<<" ms\n";
                    if (!persist_path.empty()) {
                        if (persist_calibration(persist_path,"S_EST",s_est,"L_EST",l_est))
                            std::cout<<"[标定] 已回写 "<<persist_path<<"\n";
                        else std::cerr<<"[标定] 回写失败\n"; }
                }
            } else if (!oob)
                std::cout<<"[标定] 失败: 样本 "<<hist.size()<<"/"<<collect_frames<<"\n";
        }

        if (collecting_enabled) {
            bool cd_ok = elapsed_ms(now,last_save) >= cooldown_ms;
            std::string mode;

            if (g_left_down.load()) {
                if (g_cap_fire.load() && elapsed_ms(now,last_fire) >= fire_ms) { mode="fire"; last_fire=now; }
                if (g_cap_auto.load() && now>=next_auto) next_auto=now+roll_auto();
            } else {
                if (g_cap_det.load() && found && cd_ok) mode="det";
                if (mode.empty() && g_cap_auto.load() && now>=next_auto && cd_ok) mode="auto";
            }

            if (!mode.empty()) {
                enqueue_save(make_filepath(out_dir+"/"+mode), cap_img);
                last_save=now; next_auto=now+roll_auto();
                if (mode=="fire") ++n_fire;
                else if (mode=="det") ++n_det;
                else ++n_auto;
                std::cout<<"[SAVE] "<<mode<<"  (fire="<<n_fire<<" det="<<n_det
                         <<" auto="<<n_auto<<" drop="<<g_dropped.load()<<")\n";
            }
        }

        if (preview) {
            for (auto& d:filtered) {
                cv::Rect box((int)(d.cx-d.w*.5f+crop_x),(int)(d.cy-d.h*.5f+crop_y),(int)d.w,(int)d.h);
                box&=cv::Rect(0,0,cam_w,cam_h);
                cv::rectangle(frame,box,cv::Scalar(0,255,0),2);
                float ty=d.cy+d.h*(0.5f-y_off_pct/100.0f)+crop_y;
                cv::drawMarker(frame,cv::Point((int)(d.cx+crop_x),(int)ty),
                               cv::Scalar(0,0,255),cv::MARKER_CROSS,10,2);
            }
            if (collecting_enabled) {
                char st[128];
                snprintf(st,sizeof(st),"fire:%ld det:%ld auto:%ld drop:%ld%s",
                         n_fire,n_det,n_auto,g_dropped.load(),g_left_down.load()?" [FIRE]":"");
                cv::putText(frame,st,cv::Point(10,30),cv::FONT_HERSHEY_SIMPLEX,0.6,
                            g_left_down.load()?cv::Scalar(0,0,255):cv::Scalar(0,255,0),2);
            }
            cv::imshow("Aimbot",frame);
            if (cv::waitKey(1)==27) break;
        }
    }

    cap.release();
    if (preview) cv::destroyAllWindows();
    CHECK_CUDA(cudaStreamDestroy(stream));
    CHECK_CUDA(cudaFreeHost(h_out));
    CHECK_CUDA(cudaFree(d_in)); CHECK_CUDA(cudaFree(d_out)); CHECK_CUDA(cudaFree(d_bgr));
    if (d_model_bgr!=d_bgr) CHECK_CUDA(cudaFree(d_model_bgr));
    if (collecting_enabled)
        std::cout<<"采集统计: fire="<<n_fire<<" det="<<n_det<<" auto="<<n_auto
                 <<" dropped="<<g_dropped.load()<<"\n";
    std::cout<<"AI 线程已退出\n";
}
