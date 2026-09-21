// ============================================================================
//  capture.cu — ai_thread 的实现: 引擎反序列化与张量绑定, GStreamer 管道搭建
//    (整幅预览 / 居中裁剪两种管道), 逐帧采集 → 预处理 → 推理 → 输出解析 →
//    NMS → FOV 目标筛选 → estimator_step; 标定采样 (640 居中裁切 → 320 相关域 →
//    3×3 块一维投影相位相关 → 静止簇剔除取中位, 见 core/calib.h) 与
//    g_calib_request 驱动的拟合/回写, 三源截图入队与预览叠加。
//    标定期间**整条推理链跳过** (标定既不需要检测也不需要注入, 省下的 GPU/CPU 付
//    相关的账; 发布的目标自然过期, 跑完首次检测经既有 TRACK_JUMP_GATE 重锁)。
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
#include "io/calib_run.h"

// conf / y_off / fov 每帧从热参数原子取快照 (帧内一致), 未收热参时值与 CLI 一致
void ai_thread(std::string model_path, int target_cls,
               std::string cam_dev, int cam_fps, bool preview,
               float init_l, std::string persist_path, CalMode cal_mode,
               std::string out_dir, int fire_ms, double auto_s,
               int cooldown_ms, int jpeg_quality) {
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

    // 标定采样状态: l_est 是运行态里唯一的标定量 (回写只有延迟), 拟合在
    //   g_calib_request 时一次跑完 (计划/窗口/拟合/诊断/回写都在 io/calib_run.h)
    float l_est=init_l;
    std::deque<CalibSample> hist; bool was_collecting=false;
    CalibSampler sampler;
    cv::Mat prev_dom;                      // 上一帧的相关域图 (CV_32F 320×320)
    long collect_frames=0, collect_samples=0;
    double collect_ms=0.0;                 // 采样本身的累计耗时 (每帧成本实测)
    auto cal_t0=std::chrono::steady_clock::now();
    const size_t hist_max=(size_t)std::max(60, cal_hist_frames(cal_mode,cam_fps));

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

        // 标定期不跑预处理/推理/NMS: 标定既不需要检测也不需要注入, 省下的 GPU/CPU
        //   正好付相关的账。检测状态自然过期 (g_target 走 TARGET_STALE_MS 超时路径),
        //   标定结束后第一帧检测重新入锁 — 既有 TRACK_JUMP_GATE 路径, 不引入新分支。
        const bool cal_collecting=g_calib_collect.load();
        if (cal_collecting&&!was_collecting) {
            hist.clear(); prev_dom.release();
            collect_frames=collect_samples=0; collect_ms=0.0;
            cal_t0=std::chrono::steady_clock::now(); }
        was_collecting=cal_collecting;

        std::vector<Detection> filtered;
        if (!cal_collecting) {
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
        filtered=nms(raw,nms_iou_thr);
        if (need_crop) for (auto& d:filtered) {
            d.cx+=m_off_x; d.cy+=m_off_y; }
        }   // !cal_collecting

        float best_dist=1e9f,best_dx=0,best_dy=0; bool found=false;
        if (!cal_collecting)
        for (auto& d:filtered) {
            float ty=d.cy+d.h*(0.5f-y_off_pct/100.0f);
            float dx=d.cx-cx0, dy=ty-cy0, dist=std::sqrt(dx*dx+dy*dy);
            if (dist<best_dist&&dist<fov_r) { best_dist=dist;best_dx=dx;best_dy=dy;found=true; } }

        auto now=std::chrono::steady_clock::now();
        // 自身运动换算的账本来源与逐轴比例在 estimator_step 内部按输出模式取
        //   (一次快照, 见 core/estimator.cu)
        float dt=estimator_step(est,now,found,best_dx,best_dy,l_est,max_v);

        if (cal_collecting) {
            // 640 居中裁切 → 灰度 → 半分辨率 320 相关域 → 3×3 块一维投影相位相关
            //   (先裁切后压缩: 裁切保证标定场居中且与模型解耦, 压缩决定算力; 采样
            //   几何与两方案对照见 core/calib.h 文件头)
            const auto t_smp0=std::chrono::steady_clock::now();
            const CalCropRect cr=calib_crop_rect(cap_img.cols,cap_img.rows);
            cv::Mat gray,dm;
            cv::cvtColor(cap_img(cv::Rect(cr.x,cr.y,cr.w,cr.h)),gray,cv::COLOR_BGR2GRAY);
            cv::resize(gray,gray,cv::Size(CALIB_SAMPLE_PX,CALIB_SAMPLE_PX),0,0,cv::INTER_AREA);
            gray.convertTo(dm,CV_32F);
            ++collect_frames;
            if (!prev_dom.empty()&&prev_dom.size()==dm.size()) {
                const CalibSampler::Frame f=sampler.measure(prev_dom,dm);
                if (f.ok[0]||f.ok[1]) {
                    CalibSample smp;
                    smp.t=now; smp.dt_ms=dt;
                    smp.sx=f.shift[0]; smp.sy=f.shift[1];
                    smp.sx_all=f.shift_all[0]; smp.sy_all=f.shift_all[1];
                    for (int a=0;a<2;++a) { smp.ok[a]=f.ok[a]; smp.resp[a]=f.resp[a];
                                            smp.spread[a]=f.spread[a];
                                            smp.n_static[a]=f.n_static[a]; }
                    smp.slot=cal_note_sample(smp);       // 在线累计 + 打槽位
                    hist.push_back(smp);
                    while (hist.size()>hist_max) hist.pop_front();
                    ++collect_samples;
                }
            }
            prev_dom=dm;
            collect_ms+=elapsed_ms(std::chrono::steady_clock::now(),t_smp0);
        }

        if (g_calib_request.exchange(false)) {
            // 拟合 (停顿 + 三读数; 账本不参与 — 测量完全来自屏幕位移), 诊断与回写
            //   都在 io/calib_run.cu 里 (两模式同一份口径)
            const CalResult cr=cal_fit(cal_mode,hist,g_cal_win.snapshot());
            const int done=cal_done_code(cr);
            cal_print_diag(cal_mode,cr,hist.size());
            // 标定期采样率实测 (要求不假设 120fps: 读数分辨率 = ±dt/2, 段跨 = T/dt 帧 —
            //   实测值连同每帧采样耗时一起进日志, 640→320 保帧率的验收点)
            if (collect_frames>0) {
                const double wall=elapsed_ms(std::chrono::steady_clock::now(),cal_t0);
                const double ms=wall/(double)collect_frames;
                const double fps=cam_fps;
                printf("[标定] 采样: 有效样本 %ld / 采集帧 %ld, 均值 %.2fms ≈ %.0ffps "
                       "(采集率 %d fps, 无样本帧 %.1f%%) | 采样耗时 %.2fms/帧 (几何 640→%d, "
                       "9 块×2 轴一维投影)\n",
                       collect_samples, collect_frames, ms, ms>0?1000.0/ms:0.0, cam_fps,
                       100.0*(1.0-(double)collect_samples/std::max(1.0,(double)collect_frames)),
                       collect_ms/std::max(1.0,(double)collect_frames), CALIB_SAMPLE_PX);
                fflush(stdout);
            }
            if (done==1) {
                if (!persist_path.empty()) {
                    if (cal_writeback(cal_mode,cr,persist_path))
                        std::cout<<"[标定] 已回写 "<<persist_path
                                 <<(cal_mode==CAL_MODE_HID?" (L_EST)":" (L_EST_PAD)")<<"\n";
                    else std::cerr<<"[标定] 回写失败\n"; }
                l_est=cr.l_est;      // 只标延迟: 唯一进运行态的标定量
            } else std::cout<<"[标定] 失败, 未回写 (无编造的值)\n";
            g_calib_done=done;
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
