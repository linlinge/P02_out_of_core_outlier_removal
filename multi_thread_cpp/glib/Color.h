#pragma once
#include "V3.hpp"
#include <vector>
using namespace std;
V3 get_color(float min,float max,float confidence);
template<class T>
vector<V3> ToColor(vector<T>& dat)
{
    vector<V3> out;
    out.resize(dat.size());
    auto vmin=*std::min_element(dat.begin(),dat.end());
    auto vmax=*std::max_element(dat.begin(),dat.end());
    #pragma omp prallel for
    for(int i=0;i<dat.size();i++){
        out[i]=get_color(vmin,vmax,dat[i]);
    }
    return out;
}