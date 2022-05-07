#pragma once
#include <vector>
#include <fstream>
#include <string>
#include <iostream>
#include <StringExtend.h>
#include <Statistics.h>
using namespace std;
class HashTable
{
    public:
        int n_,m_;
        vector<vector<int>> dat_;
        vector<vector<int>> offset_len_;
        string path_;

        double GetThresholdWithTukeyFence(double kIQR);

        /* Operation */
        void Resize(int n);
        void PushBack(int idx, int val);
        int GetSize();

        /* Write Section */
        void SaveBinary(string path);
        void SaveAscii(string path);

        /* Read Section */
        void Init(string path);
        void ReadLine(int i,vector<int>& dat);     
        void ReadLines(string path_to_ht, string mode="binary");  
};