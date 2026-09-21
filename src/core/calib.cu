// ============================================================================
//  calib.cu — calib.h 的实现: 一维投影相位相关 (calib_pc1d), 单帧块统计
//    (calib_axis_stats — 静止簇剔除 + 中位 + 两个质量量), 逐帧采样器 (CalibSampler),
//    标定值脚本原子回写 (persist_calibration), /dev/v4l/by-id 采集卡名解析
//    (resolve_cam_device)。纯 C++/OpenCV, 不含相机与模型依赖 — 单测直接喂合成块与
//    合成图 (方案对照与几何不变量都在 core/calib_test.cu)。
// ============================================================================

#include "core/calib.h"

#include <algorithm>
#include <cctype>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {

// 小数组取中位 (块数 ≤ 9): 就地 nth_element, 偶数个取上中位 (采样端一贯口径)
float median_inplace(float* v, int n) {
    if (n <= 0) return 0.0f;
    std::nth_element(v, v + n / 2, v + n);
    return v[n / 2];
}

// 一维汉宁窗 (cv::createHanningWindow 要求两维均 >1): rowvec = 1×n / 否 = n×1
cv::Mat hann1d(int n, bool rowvec) {
    cv::Mat w = cv::Mat::zeros(rowvec ? cv::Size(n, 1) : cv::Size(1, n), CV_32F);
    for (int i = 0; i < n; ++i) {
        const float v = 0.5f - 0.5f * (float)std::cos(2.0 * (double)CV_PI * (double)i
                                                      / (double)(n - 1));
        if (rowvec) w.at<float>(0, i) = v; else w.at<float>(i, 0) = v;
    }
    return w;
}

} // namespace

double calib_pc1d(const cv::Mat& a, const cv::Mat& b, const cv::Mat& win, double* resp) {
    const int n = a.cols;
    cv::Mat fa(n, 1, CV_32F), fb(n, 1, CV_32F);
    for (int i = 0; i < n; ++i) {
        fa.at<float>(i, 0) = a.at<float>(0, i) * win.at<float>(0, i);
        fb.at<float>(i, 0) = b.at<float>(0, i) * win.at<float>(0, i);
    }
    cv::Mat A, B;
    cv::dft(fa, A, cv::DFT_COMPLEX_OUTPUT);
    cv::dft(fb, B, cv::DFT_COMPLEX_OUTPUT);
    // 白化互谱: 只留相位 (相位相关的定义), 峰因此与幅度无关
    cv::Mat C(n, 1, CV_32FC2);
    for (int i = 0; i < n; ++i) {
        const cv::Vec2f x = A.at<cv::Vec2f>(i, 0), y = B.at<cv::Vec2f>(i, 0);
        const float re = x[0] * y[0] + x[1] * y[1];
        const float im = x[1] * y[0] - x[0] * y[1];
        const float m = std::sqrt(re * re + im * im) + 1e-12f;
        C.at<cv::Vec2f>(i, 0) = cv::Vec2f(re / m, im / m);
    }
    cv::Mat c;
    cv::dft(C, c, cv::DFT_INVERSE | cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
    int best = 0; float bv = c.at<float>(0, 0);
    for (int i = 1; i < n; ++i) { const float v = c.at<float>(i, 0); if (v > bv) { bv = v; best = i; } }
    if (resp) *resp = bv;
    // 三点抛物线插值 (循环邻域): 亚像素位移
    const double y0 = c.at<float>((best - 1 + n) % n, 0), y1 = bv,
                 y2 = c.at<float>((best + 1) % n, 0);
    const double den = y0 - 2.0 * y1 + y2;
    double d = (std::fabs(den) > 1e-12) ? (0.5 * (y0 - y2) / den) : 0.0;
    d = std::clamp(d, -1.0, 1.0);
    double s = (double)best + d;
    if (s > n / 2.0) s -= n;                 // 循环: 折到 ±n/2
    return -s;                               // 正 = 内容朝 +轴 移动 (与 cv::phaseCorrelate 同口径)
}

CalAxisStats calib_axis_stats(int n, const float* shift, const float* resp) {
    CalAxisStats o;
    if (n > CALIB_BLOCKS_N) n = CALIB_BLOCKS_N;
    if (n < CALIB_BLOCK_MIN) return o;
    o.ok = true; o.n_used = n;

    // 全体中位 (静止簇剔除前的对照, 也是兜底值)
    float tmp[CALIB_BLOCKS_N];
    for (int i = 0; i < n; ++i) tmp[i] = shift[i];
    o.shift_all = -CALIB_SAMPLE_SCALE * median_inplace(tmp, n);

    // 静止簇剔除: 门限 = CALIB_STATIC_FRAC × 最大块位移 (出处见 calib.h 文件头)。
    //   最大位移取绝对值 — 一维投影下每块只有一个轴分量。
    float mx = 0.0f;
    for (int i = 0; i < n; ++i) mx = std::max(mx, std::fabs(shift[i]));
    const float thr = CALIB_STATIC_FRAC * mx;
    int sel[CALIB_BLOCKS_N], ns = 0;
    for (int i = 0; i < n; ++i) if (std::fabs(shift[i]) >= thr) sel[ns++] = i;

    int m = 0;
    if (ns >= CALIB_CLUSTER_MIN) {
        o.n_static = n - ns;
        for (int k = 0; k < ns; ++k) tmp[k] = shift[sel[k]];
        m = ns;
    } else {                                  // 兜底: 运动簇不足 → 全体中位
        o.fallback = true;
        for (int i = 0; i < n; ++i) sel[i] = i;
        m = n;
    }
    for (int k = 0; k < m; ++k) tmp[k] = resp[sel[k]];
    o.resp = median_inplace(tmp, m);
    for (int k = 0; k < m; ++k) tmp[k] = shift[sel[k]];
    const float med = median_inplace(tmp, m);
    o.shift = -CALIB_SAMPLE_SCALE * med;
    float sp = 0.0f;
    for (int k = 0; k < m; ++k) sp = std::max(sp, std::fabs(shift[sel[k]] - med));
    o.spread = CALIB_SAMPLE_SCALE * sp;
    return o;
}

CalibSampler::CalibSampler() {
    win_row_ = hann1d(CALIB_BLOCK_PX, true);
    win_col_ = hann1d(CALIB_BLOCK_PX, false);
}

CalibSampler::Frame CalibSampler::measure(const cv::Mat& prev_f32, const cv::Mat& cur_f32) {
    Frame f;
    if (prev_f32.empty() || cur_f32.empty()
        || prev_f32.size() != cur_f32.size()
        || prev_f32.size().width != CALIB_SAMPLE_PX) return f;
    for (int axis = 0; axis < 2; ++axis) {
        float sh[CALIB_BLOCKS_N], rq[CALIB_BLOCKS_N];
        int nv = 0;
        for (int by = 0; by < CALIB_GRID_N; ++by)
            for (int bx = 0; bx < CALIB_GRID_N; ++bx) {
                const cv::Rect r(bx * CALIB_BLOCK_PX, by * CALIB_BLOCK_PX,
                                 CALIB_BLOCK_PX, CALIB_BLOCK_PX);
                // 沿轴投影成一维 (x: 块内各行求和 → 行向量; y: 各列求和 → 列向量转置),
                //   再相关: 投影把 106 个样本平均进每一条谱线, 峰因此比二维块相关更陡
                cv::Mat pa, pb;
                if (axis == 0) {
                    cv::reduce(prev_f32(r), pa, 0, cv::REDUCE_SUM, CV_32F);
                    cv::reduce(cur_f32(r), pb, 0, cv::REDUCE_SUM, CV_32F);
                } else {
                    cv::reduce(prev_f32(r), pa, 1, cv::REDUCE_SUM, CV_32F);
                    cv::reduce(cur_f32(r), pb, 1, cv::REDUCE_SUM, CV_32F);
                    pa = pa.t(); pb = pb.t();
                }
                double resp = 0;
                const double s = calib_pc1d(pa, pb, axis == 0 ? win_row_ : win_col_, &resp);
                if (resp > CALIB_BLOCK_RESP) {
                    sh[nv] = (float)s; rq[nv] = (float)resp; ++nv;
                }
            }
        const CalAxisStats st = calib_axis_stats(nv, sh, rq);
        f.ok[axis] = st.ok;
        f.shift[axis] = st.shift;
        f.shift_all[axis] = st.shift_all;
        f.resp[axis] = st.resp;
        f.spread[axis] = st.spread;
        f.n_static[axis] = st.n_static;
    }
    return f;
}

bool persist_calibration(const std::string& path, const std::string& var, float l) {
    std::ifstream in(path); if (!in.good()) return false;
    std::vector<std::string> lines; std::string line;
    while (std::getline(in,line)) lines.push_back(line); in.close();
    char buf[64];
    snprintf(buf,sizeof(buf),"%s=%.1f",var.c_str(),(double)l);
    const std::string key=var+"=";
    bool found=false;
    for (auto& ln:lines) if (ln.rfind(key,0)==0) { ln=buf; found=true; }
    if (!found) lines.push_back(buf);
    struct stat st{}; bool have=(stat(path.c_str(),&st)==0);
    std::string tmp=path+".tmp."+std::to_string((long)getpid());
    { std::ofstream o(tmp,std::ios::trunc); if (!o.good()) return false;
      for (auto& ln:lines) o<<ln<<"\n"; }
    if (have) { chmod(tmp.c_str(),st.st_mode); chown(tmp.c_str(),st.st_uid,st.st_gid); }
    if (rename(tmp.c_str(),path.c_str())!=0) { unlink(tmp.c_str()); return false; }
    return true;
}

std::string resolve_cam_device(const std::string& spec) {
    if (spec.rfind("/dev/",0)==0) return spec;
    std::string key=spec;
    std::transform(key.begin(),key.end(),key.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    std::vector<std::string> avail,hits;
    if (DIR* dp=opendir("/dev/v4l/by-id")) {
        while (dirent* e=readdir(dp)) {
            std::string nm=e->d_name;
            const std::string suf="-video-index0";
            if (nm.size()<=suf.size()
                || nm.compare(nm.size()-suf.size(),suf.size(),suf)!=0) continue;
            avail.push_back(nm);
            std::string low=nm;
            std::transform(low.begin(),low.end(),low.begin(),
                           [](unsigned char c){ return std::tolower(c); });
            if (low.find(key)!=std::string::npos) hits.push_back(nm);
        }
        closedir(dp);
    }
    if (hits.size()!=1) {
        std::cerr<<"❌ 采集卡 \""<<spec<<"\" "
                 <<(hits.empty()?"没有匹配":"匹配到多个")<<", 可选:\n";
        for (auto& a:avail) std::cerr<<"   "<<a<<"\n";
        return "";
    }
    std::string link="/dev/v4l/by-id/"+hits[0];
    char resolved[PATH_MAX];
    if (realpath(link.c_str(),resolved)) return resolved;
    return link;
}
