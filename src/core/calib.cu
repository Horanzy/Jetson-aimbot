// ============================================================================
//  calib.cu — calib.h 的实现: counts 历史上的最小二乘 s 估计与延迟粗/细双扫
//    (run_calibration), S_EST/L_EST 写入脚本的临时文件原子替换
//    (persist_calibration), /dev/v4l/by-id 采集卡名解析 (resolve_cam_device)。
// ============================================================================

#include "core/calib.h"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include "core/state.h"

bool run_calibration(const std::deque<CalibSample>& hist, float& s_est, float& l_est) {
    const int n=(int)hist.size(); if (n<CALIB_WINDOW) return false;
    auto scan=[&](float lo,float hi,float step,float& out_s,float& out_dl)->float {
        float best=FLT_MAX;
        for (float dl=lo;dl<=hi;dl+=step) {
            double lag=l_est+dl, sum_cc=0, sum_sc=0;
            std::vector<std::pair<float,float>> cs(n);
            for (int i=0;i<n;++i) { auto&r=hist[i];
                auto c0=g_counts.at(shift_ms(r.t,-lag-r.dt_ms));
                auto c1=g_counts.at(shift_ms(r.t,-lag));
                float Cx=(float)(c1.first-c0.first), Cy=(float)(c1.second-c0.second);
                cs[i]={Cx,Cy}; sum_cc+=(double)Cx*Cx+(double)Cy*Cy;
                sum_sc+=(double)r.sx*Cx+(double)r.sy*Cy; }
            if (sum_cc<CALIB_MIN_EXCITE) continue;
            double s_hat=sum_sc/sum_cc, score=0;
            for (int i=0;i<n;++i) { double ex=hist[i].sx-s_hat*cs[i].first,
                                         ey=hist[i].sy-s_hat*cs[i].second; score+=ex*ex+ey*ey; }
            score/=sum_cc;
            if (score<best) { best=(float)score; out_s=(float)s_hat; out_dl=dl; } }
        return best;
    };
    float s1=0,dl1=0;
    if (scan(-40.0f,96.0f,8.0f,s1,dl1)==FLT_MAX) return false;
    float s2=s1,dl2=dl1;
    if (scan(dl1-8.0f,dl1+8.0f,2.0f,s2,dl2)==FLT_MAX) { s2=s1; dl2=dl1; }
    s_est=std::clamp(s2,S_MIN,S_MAX); l_est=std::clamp(l_est+dl2,L_MIN,L_MAX);
    return true;
}
bool persist_calibration(const std::string& path, float s, float l) {
    std::ifstream in(path); if (!in.good()) return false;
    std::vector<std::string> lines; std::string line;
    while (std::getline(in,line)) lines.push_back(line); in.close();
    char sbuf[64],lbuf[64];
    snprintf(sbuf,sizeof(sbuf),"S_EST=%.4f",s); snprintf(lbuf,sizeof(lbuf),"L_EST=%.1f",l);
    bool fs=false,fl=false;
    for (auto& ln:lines) { if (ln.rfind("S_EST=",0)==0){ln=sbuf;fs=true;}
                           else if (ln.rfind("L_EST=",0)==0){ln=lbuf;fl=true;} }
    if (!fs) lines.push_back(sbuf); if (!fl) lines.push_back(lbuf);
    struct stat st{}; bool have=(stat(path.c_str(),&st)==0);
    std::string tmp=path+".tmp";
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
