#pragma once
#include "PCLExtend.h"
// #include "FileExtend.h"
#include "Statistics.h"
#include "StringExtend.h"
#include "VectorExtend.h"
#include "PartManager.h"
#include "GridClustering.h"
#include <iostream>
using namespace std;

class HybridMethod
{
    public:        
        pcl::PointCloud<PointType>::Ptr cloud_;
        pcl::PointCloud<PointType>::Ptr cloud_out_regular_;
        pcl::PointCloud<PointType>::Ptr cloud_out_irregular_;
        pcl::search::KdTree<PointType>::Ptr kdtree_;
        int flag_count_;
        vector<int> status_;
        int cnt_;     
        
        HybridMethod(pcl::PointCloud<PointType>::Ptr cloud){
            flag_count_=0;
            cnt_=0;
            cloud_=cloud;
            status_.resize(cloud_->points.size());            
            kdtree_=pcl::search::KdTree<PointType>::Ptr(new pcl::search::KdTree<PointType>);
            kdtree_->setInputCloud(cloud_);
            cloud_out_regular_=pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
            cloud_out_irregular_=pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);

        }

        // input: patch        
        void GenerateNoise();        
        void RegionGrowth_kIQR(string domain="0", double level=8, double thresh_kIQR=3.0);
        void RegionGrowth_Max(string domain="0", double level=8);

        /* OR2 */
        void OR2(string domain="0", int level=8, double thresh_kIQR=3.0);
        void PlaneRefine(string domain="0", double level=8, double thresh_kIQR=3.0);

        void FM_MEval(string domain,int K, double alpha);

        /* remove by bounding box */
        void RemoveByBoundingBox(string domain="0",string path_to_low="default");

        void LoOP(string domain="0", int K=40, double thresh=0.8);
        void STING_kIQR(string domain="0", int cell_level=10, int cluster_level=8,double threshold=15.0);
        void STING_Max(string domain="0", int cell_level=10, int cluster_level=8);
        void SOR(string domain="0");
        void ExtractResult();

        void LNGD(string domain="0",int K=40, double kIQR=3);

        void Clear(){
            cloud_out_regular_->points.clear();
            cloud_out_irregular_->points.clear();
            status_.clear();
        }
    
    private:
        void GetScopeIndices(string str,vector<int>& cIdx);
};