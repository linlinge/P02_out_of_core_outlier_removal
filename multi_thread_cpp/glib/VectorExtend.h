#pragma once
#include<fstream>
#include <iostream>
#include <vector>
#include <limits.h>
#include <algorithm>
#include <set>
using namespace std;
double VectorMean(vector<double>& dat);

template<class T1>
double VectorMaximum(vector<T1>& dat)
{
    auto vmax=std::max_element(dat.begin(),dat.end());
    return *vmax;
}

template<class T2>
void VectorMaximum(vector<T2>& dat,T2& vmax, int& idx){
    vmax=dat[0];
    idx=0;
    for(int i=1;i<dat.size();i++){
        if(vmax<dat[i]){
            vmax=dat[i];
            idx=i;
        }
    }
}

double VectorMinimum(vector<double>& dat);
double VectorStd(vector<double>& dat);
double VectorQuantile(vector<double>& dat,double p);
double VectorSum(vector<double>& dat);
void VectorDelete(vector<int>&rdat,int index);
void VectorDelete(vector<int>&raw_dat, vector<int>& delete_dat);
int VectorIQR(vector<double>& dat);
void VectorNormalize(vector<double>& dat);
void VectorInterval(vector<double>& dat,vector<double>& out);
void VectorInterval(vector<float>& dat,vector<double>& out);
// find the first element among dat which is not belong to dictionary
int VectorFindFirstNot(vector<int>& dictionary, vector<int>& dat);

/*
    Logical Operation
*/
void VectorDifference(vector<int>& dat1,vector<int>& dat2,vector<int>& out);
void VectorDifference(string range,vector<int>& dat2,vector<int>& out);
void VectorDifference(int start_idx, int end_idx, vector<int>& dat2, vector<int>& out);
void VectorUnion(vector<int>& dat1,vector<int>& dat2,vector<int>& out);
void VectorIntersection(vector<int>& dat1,vector<int>& dat2,vector<int>& out);

/* Find */
int VecFindPos(vector<int>& buf,int dat);
/* Get the indices of all elements of buf2 in buf1*/
void VecFindPos(vector<int>& buf1,vector<int>& buf2, vector<int>& out);
void VecPrint(vector<int>& dat);
void VecUnique(vector<int>& dat);
/* write */
void VectorWrite(string filename,vector<double>& dat,string mode="append,column");
void VectorWrite(string filename,vector<float>& dat,string mode="append,column");
void VectorWrite(string filename,vector<int>& dat,string mode="append,column");

template<class T3>
void VectorWrite(string filename,vector<vector<T3>>& dat)
{
    cout<<dat.size()<<endl;
    int count_endl=0;
    ofstream file(filename, ios::out | std::ios::app);
    for(int i=0;i<dat.size();i++){        
        if(dat[i].size()!=0){
            int j=0;
            for(;j<dat[i].size()-1;j++){
                file<<dat[i][j]<<" ";
            }
            file<<dat[i][j]<<endl;
            count_endl++;
        }
        else{
            file<<endl;
            count_endl++;
        }
    }
    file.close();
    cout<<"count_endl:"<<count_endl<<endl;
}