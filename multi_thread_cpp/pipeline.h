#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include "PCLExtend.h"
#include "FileExtend.h"
#include <random>
#include <algorithm>
#include <stdlib.h>
#include <time.h> 
#include <Eigen/Dense>
#include "global.h"
#include "HybridMethod.h"
#include "omp.h"
#include "PlyParsing.h"
#include "PartManager.h"
using namespace std;

class Pipeline
{
    private:
        pcl::PointCloud<PointType>::Ptr LPC_;
        string RPC_path_;
        vector<vector<int>> hash_table_;
        PlyParsing ppr_;

    public:
        Pipeline(){
            LPC_=pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
        }
        void DealWithPlaneLikeSegments(int cluster_id, vector<int>& indices_in_cluster, string path_to_raw, HashTable& ht);

        void Downsampling(string load_path, double num_or_ratio, pcl::PointCloud<PointType>::Ptr cloud);        
        void Segmentation_condition(pcl::PointCloud<PointType>::Ptr cloud_LPC, pcl::PointCloud<PointType>::Ptr cloud_out,string save_path);
        void Segmentation_has_normal(string path_to_raw, pcl::PointCloud<PointType>::Ptr cloud_LPC,string path_to_save_parts);

        // extract colored LPC
        void Segmentation_has_normal(pcl::PointCloud<PointType>::Ptr cloud_LPC,string path_to_save);        

     
        void Indexing(pcl::PointCloud<PointType>::Ptr cloud, string filepath);
        void Merging(string read_files_path, string write_file_path);
};