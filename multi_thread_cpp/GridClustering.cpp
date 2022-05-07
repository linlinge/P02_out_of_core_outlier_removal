#include "GridClustering.h"
#include "Statistics.h"
#define MAX2(D1,D2) ((D1)>(D2) ? (D1):(D2))
#define MAX3(D1,D2,D3) MAX2(MAX2(D1,D2),(D3))

bool customRegionGrowing2(const PointType& point_a, const PointType& point_b, float squared_distance)
{
    return true;
}

GridClustering::GridClustering(pcl::PointCloud<PointType>::Ptr cloud)
{
    cloud_raw_=cloud;
    cloud_out_=pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    kdtree_=pcl::search::KdTree<PointType>::Ptr(new pcl::search::KdTree<PointType>);
    kdtree_->setInputCloud(cloud_raw_);
}

void GridClustering::OctreeProperties(int octree_level)
{
	/* establish octree */
	// 最小体素的边长
	float resolution = GetCellSize(cloud_raw_,octree_level);
	pcl::octree::OctreePointCloudSearch<PointType> octree(resolution);
	octree.setInputCloud(cloud_raw_);

	// 从输入点云构建八叉树
	octree.addPointsFromInputCloud();

	// 求出体素边界
	int depth = octree.getTreeDepth();
	for (auto it = octree.begin(depth);it != octree.end();++it){
		if (it.isLeafNode()){
			// Get minimum and maximum boundary of each voxel
			Eigen::Vector3f  voxel_min, voxel_max;
			octree.getVoxelBounds(it, voxel_min, voxel_max);
			Eigen::Vector3f voxel_cen;
			voxel_cen=(voxel_min+voxel_max)/2.0;
			PointType ptmp;
			ptmp.x=voxel_cen[0];
			ptmp.y=voxel_cen[1];
			ptmp.z=voxel_cen[2];
			cloud_out_->points.push_back(ptmp);

			// Get points indices in each voxel
			auto leaf = it.getLeafContainer();
			std::vector<int> itmp; 
			leaf.getPointIndices(itmp);
			indices_.push_back(itmp);
		}
	}
}

void GridClustering::RegionGrowth_kIQR(double level, double kIQR)
{
    /* Step 03: */
    pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters);
    pcl::search::KdTree<PointType>::Ptr kdtree_tmp(new pcl::search::KdTree<PointType>);

    PointType vmin,vmax;
    pcl::getMinMax3D(*cloud_raw_, vmin, vmax);
    double box_max=MAX3(vmax.x-vmin.x,vmax.y-vmin.y,vmax.z-vmin.z);
    double tolerance=box_max/pow(2,level);

    // Set up a Conditional Euclidean Clustering class
    pcl::ConditionalEuclideanClustering<PointType> cec(true);
    cec.setInputCloud(cloud_out_);
    cec.setConditionFunction(&customRegionGrowing2);
    cec.setClusterTolerance(tolerance);
    cec.segment(*clusters);

    /* Step 04: Threshold */
    #if GRID_CLUSTERING_MODE
        pcl::copyPointCloud(*cloud_raw_,*cloud_out_);
        for(int i=0;i<clusters->size();i++){
            int current_cluster_size=(*clusters)[i].indices.size();
            int rtmp=rand()%256;
            int gtmp=rand()%256;
            int btmp=rand()%256;
            for(int j=0;j<(*clusters)[i].indices.size();j++){
                int itmp=(*clusters)[i].indices[j];
                for(int k=0;k<indices[itmp].size();k++){
                    int itt=indices[itmp][k];
                    cloud_out_->points[itt].r=rtmp;
                    cloud_out_->points[itt].g=gtmp;
                    cloud_out_->points[itt].b=btmp; 
                }
            }
        }
    #else   
        // get true cluster: clusters (cell id) -> clusters (point indices)
        vector<vector<int>> cls;
        cls.resize(clusters->size());
        for(int i=0;i<clusters->size();i++){
            for(int j=0;j<(*clusters)[i].indices.size();j++){
                int itmp=(*clusters)[i].indices[j];
                cls[i].insert(cls[i].end(),indices_[itmp].begin(),indices_[itmp].end());
            }
        }

        // get cluster size
        vector<int> cluster_size;
        for(int i=0;i<cls.size();i++){
             cluster_size.push_back(cls[i].size());
        }
       
        double thresh=TukeyFence(cluster_size,kIQR);

        // threshold
        vector<int> idx_out;
        for(int i=0;i<cls.size();i++){
           if(cls[i].size()>thresh){    
               idx_out.insert(idx_out.end(),cls[i].begin(),cls[i].end());
           }
        }
        pcl::copyPointCloud(*cloud_raw_,idx_out,*cloud_out_);
    #endif
}

void GridClustering::RegionGrowth_Max(double level)
{
    /* Step 03: */
    pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters);
    pcl::search::KdTree<PointType>::Ptr kdtree_tmp(new pcl::search::KdTree<PointType>);

    PointType vmin,vmax;
    pcl::getMinMax3D(*cloud_raw_, vmin, vmax);
    double box_max=MAX3(vmax.x-vmin.x,vmax.y-vmin.y,vmax.z-vmin.z);
    double tolerance=box_max/pow(2,level);

    // Set up a Conditional Euclidean Clustering class
    pcl::ConditionalEuclideanClustering<PointType> cec(true);
    cec.setInputCloud(cloud_out_);
    cec.setConditionFunction(&customRegionGrowing2);
    cec.setClusterTolerance(tolerance);
    cec.segment(*clusters);

    /* Step 04: Threshold */
    // get true cluster: clusters (cell id) -> clusters (point indices)
    vector<vector<int>> cls;
    cls.resize(clusters->size());
    for(int i=0;i<clusters->size();i++){
        for(int j=0;j<(*clusters)[i].indices.size();j++){
            int itmp=(*clusters)[i].indices[j];
            cls[i].insert(cls[i].end(),indices_[itmp].begin(),indices_[itmp].end());
        }
    }

    // get cluster size
    vector<int> cluster_size;
    for(int i=0;i<cls.size();i++){
            cluster_size.push_back(cls[i].size());
    }
    
    vector<int>::iterator it=std::max_element(cluster_size.begin(),cluster_size.end());
    int idx=it-cluster_size.begin();

    // extract idx
    vector<int> idx_out;
    idx_out.insert(idx_out.end(),cls[idx].begin(),cls[idx].end());
    pcl::copyPointCloud(*cloud_raw_,idx_out,*cloud_out_);
}

void GridClustering::ExtractResult(string save_path)
{
    pcl::io::savePLYFileBinary(save_path,*cloud_out_);
}