#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <HashTable.h>
#include <PlyParsing.h>
#include "PCLExtend.h"
#include "global.h"
#include "VectorExtend.h"
#include "Color.h"
using namespace std;
class PartManager
{
    public:
        Eigen::Vector3f centre_;
        Eigen::Vector3f normal_;        
        Eigen::Vector3f corners_[4];
        HashTable ht_;
        PlyParsing ppr_;       
        float a_,b_,c_,d_;

        void AdjustOrder(Eigen::Vector3f& A,Eigen::Vector3f& B,Eigen::Vector3f& C,Eigen::Vector3f& D);
        bool Intersection(Eigen::Vector3f& A, Eigen::Vector3f& B, Eigen::Vector3f& C, Eigen::Vector3f& D,Eigen::Vector3f& itsc);

        /* whether it is plane */
        bool is_plane_with_AABB(pcl::PointCloud<PointType>::Ptr cloud);
        bool is_plane_with_OBB(pcl::PointCloud<PointType>::Ptr cloud);
        void RANSAC(pcl::PointCloud<PointType>::Ptr cloud_segment);

        /* */
        bool IsInside(PointType& pt);
        void GetFoot(PointType& pt, Eigen::Vector3f& foot);

        /* Get distance between specify point and segment*/
        // void Init(pcl::PointCloud<PointType>::Ptr cloud_segment);
        // void Init2(pcl::PointCloud<PointType>::Ptr cloud_segment);
        double GetDistance(PointType& pt);

        /* Write low patch indices */
        void WriteIndices(string path,vector<int>& dat, string mode="binary");
        void ReadIndices(string path,vector<int>& dat, string mode="binary");

        /* extract raw patches  */
        void get_raw_patch_from_low_patch();
        void get_raw_patch_from_low_indices(string path_to_low_indices,string path_to_RPC,string path_to_save_row_patch);
};