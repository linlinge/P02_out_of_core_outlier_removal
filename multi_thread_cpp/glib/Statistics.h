#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include<limits.h>
using namespace std;
class Statistics
{
	public:
		vector<float> dat_;
		float min_,max_;
		float sum_;
		float mean_;
		float stdev_;
		float stdevp_;
		
		Statistics(vector<float> dat);
};

template<class T1>
double Quantile(vector<T1>& dat,double ratio)
{	
	vector<T1> dat2;
	for(int i=0;i<dat.size();i++) dat2.push_back(dat[i]);
	sort(dat2.begin(),dat2.end());

	double Q_idx=(dat.size()+1)*ratio-1;
    int Q_idx_integer=(int)Q_idx;
    double Q_idx_decimal=Q_idx-Q_idx_integer;
    double Q=dat2[Q_idx_integer]+(dat2[Q_idx_integer+1]-dat2[Q_idx_integer])*Q_idx_decimal;
    return Q;
}

template<class T2>
double TukeyFence(vector<T2>&dat, double alpha)
{
	double IQR=Quantile(dat,0.75)-Quantile(dat,0.25);
	double thresh=0;
	if(alpha>0)
		thresh=Quantile(dat,0.75)+IQR*alpha;	
	else
		thresh=Quantile(dat,0.25)+IQR*alpha;
	return thresh;	
}