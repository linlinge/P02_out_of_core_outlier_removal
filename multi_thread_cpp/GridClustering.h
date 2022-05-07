#pragma once
#include "PCLExtend.h"
// #include "FileExtend.h"
#include "StringExtend.h"
#include "VectorExtend.h"
// #include "Patch.h"
#include <iostream>
#include "global.h"
#include <set>
using namespace std;

class GridClustering
{
    public:
        pcl::PointCloud<PointType>::Ptr cloud_raw_,cloud_out_;
		vector<vector<int>> indices_;
        pcl::search::KdTree<PointType>::Ptr kdtree_;
        
        GridClustering(pcl::PointCloud<PointType>::Ptr cloud);
        void OctreeProperties(int octree_level=10);
        void RegionGrowth_kIQR(double level=8, double thresh_kIQR=3.0);
        void RegionGrowth_Max(double level=8);
        void ExtractResult(string save_path);
};