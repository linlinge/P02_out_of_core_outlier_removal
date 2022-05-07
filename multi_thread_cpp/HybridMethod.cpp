#include "HybridMethod.h"
#include "Statistics.h"
#include <pcl/filters/statistical_outlier_removal.h>
#define MAX2(D1,D2) ((D1)>(D2) ? (D1):(D2))
#define MAX3(D1,D2,D3) MAX2(MAX2(D1,D2),(D3))

bool customRegionGrowing(const PointType& point_a, const PointType& point_b, float squared_distance)
{
    return true;
}

void  HybridMethod::RemoveByBoundingBox(string domain,string path_to_low)
{
    if(path_to_low=="default"){
        cout<<"Please specify a path to low parts"<<endl;
        return;
    }

    /* Init */
    cnt_++;
    vector<int> scope;
    GetScopeIndices(domain, scope);     
    pcl::PointCloud<PointType>::Ptr cloud_active(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*cloud_,scope,*cloud_active);

    /*  */
    pcl::PointCloud<PointType>::Ptr part_low(new pcl::PointCloud<PointType>);
    pcl::io::loadPLYFile(path_to_low,*part_low);
    PointType minPt,maxPt;
    pcl::getMinMax3D(*part_low,minPt,maxPt);

    for(int i=0;i<cloud_active->points.size();i++){
        PointType& pt=cloud_active->points[i];
        if(pt.x>minPt.x && pt.y>minPt.y && pt.z>minPt.z && pt.x<maxPt.x && pt.y<maxPt.y && pt.z < maxPt.z){
            status_[scope[i]]=cnt_;          
        }
        else{
            status_[scope[i]]=-cnt_;
            #if COLOR_ON
            cloud_->points[scope[i]].r=255;
            cloud_->points[scope[i]].g=0;
            cloud_->points[scope[i]].b=0;
            #endif
        }
    }
}

void HybridMethod::LNGD(string domain,int K, double kIQR)
{
    cout<<"LNGD"<<endl;
    /* Init */
    cnt_++;
    vector<int> scope;
    GetScopeIndices(domain, scope);     
    pcl::PointCloud<PointType>::Ptr cloud_active(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*cloud_,scope,*cloud_active);


    vector<double> rst_slope_;
    rst_slope_.resize(scope.size());    
    pcl::search::KdTree<PointType>::Ptr kdtree_tmp(new pcl::search::KdTree<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_tmp(new pcl::PointCloud<PointType>);
    for(int i=0;i<scope.size();i++)
        cloud_tmp->points.push_back(cloud_->points[scope[i]]);
    kdtree_tmp->setInputCloud(cloud_tmp);

    Eigen::MatrixXd xtmp(K,1);
    for(int i=0;i<K;i++) xtmp(i,0)=i;
    // calculate y
    #pragma omp parallel for
    for(int i=0;i<scope.size();i++){
        vector<int> idx(K);
        vector<float> dist(K);
        Eigen::MatrixXd ytmp(K,1);
        int itmp=scope[i];
        kdtree_tmp->nearestKSearch(cloud_->points[itmp], K, idx, dist);

        for(int j=0;j<dist.size();j++) ytmp(j,0)=dist[j];
        Eigen::MatrixXd atmp=(xtmp.transpose()*xtmp).inverse()*xtmp.transpose()*ytmp;        
        rst_slope_[i]=atmp(0,0);
    }
    
    double thresh=TukeyFence(rst_slope_,kIQR);
    #pragma omp parallel for
    for(int i=0;i<scope.size();i++){
        if(rst_slope_[i]>thresh)
            status_[scope[i]]=-cnt_;            
        else
            status_[scope[i]]=cnt_;
    }
}

void HybridMethod::SOR(string domain)
{    
    cnt_++;
    vector<int> scope;
    GetScopeIndices(domain, scope);
    pcl::PointCloud<PointType>::Ptr cloud_active(new pcl::PointCloud<PointType>);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    pcl::copyPointCloud(*cloud_,scope,*cloud_active);
    pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);

    kdtree->setInputCloud(cloud_active);

    vector<double> distance;
    distance.resize(cloud_active->points.size());

    float mean_dist=0;
    int K=30;
    for(int i=0;i<cloud_active->points.size();i++){
        vector<int> idx;
        vector<float> dist;
        kdtree->nearestKSearch(i,K+1,idx,dist);
        for(int j=1;j<K+1;j++)
            mean_dist+=sqrt(dist[j]);
        
        mean_dist=mean_dist/K;
        distance[i]=mean_dist;    
    }

    double sum=std::accumulate(distance.begin(),distance.end(),0);
    double mean=sum/distance.size();
    double variance=0;
    for(int i=0;i<cloud_active->points.size();i++){
        variance+=pow(distance[i]-mean,2);
    }
    variance=variance/(cloud_active->points.size()-1);
    double stddev=sqrt(variance);

    vector<int> oidx;
    double thresh=mean_dist+3*stddev;
    for(int i=0;i<cloud_active->points.size();i++){
        if(distance[i]<thresh){
            status_[scope[i]]=cnt_;
        }
        else
            status_[scope[i]]=-cnt_;
    }
}

void HybridMethod::OR2(string domain, int level, double thresh_kIQR)
{
    // Step 01: init scope
    vector<int> scope;
    pcl::PointCloud<PointType>::Ptr cloud_active(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_part_0(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_part_1(new pcl::PointCloud<PointType>);
    GetScopeIndices(domain, scope);
    pcl::copyPointCloud(*cloud_,scope,*cloud_active);
    
    cnt_++;
    vector<double> dist;
    dist.resize(cloud_active->points.size());

    // get patch info
    PartManager sm;
    sm.RANSAC(cloud_active);

    /* Step 02: calculate distance between points and plane */
    // #pragma omp parallel for
    for(int j=0;j<cloud_active->points.size();j++)
        dist[j]=sm.GetDistance(cloud_active->points[j]);

    // /* Step 03: determine the threshold */
    double dist_thresh=TukeyFence(dist,3);

    vector<int> neg_scope;
    // #pragma omp parallel for
    for(int j=0;j<dist.size();j++){
        if(dist[j]>dist_thresh){
            // #pragma omp critical
            // {
                cloud_part_0->points.push_back(cloud_active->points[j]);           
                status_[j]=-cnt_;
                neg_scope.push_back(j);
            // }
        }
        else{
            // #pragma omp critical
            // {
                cloud_part_1->points.push_back(cloud_active->points[j]);             
            // }
            status_[j]=cnt_;
        }
    }

    // pcl::io::savePLYFileBinary("result2/cloud_part_0.ply",*cloud_part_0);
    // pcl::io::savePLYFileBinary("result2/cloud_part_1.ply",*cloud_part_1);
}

void HybridMethod::FM_MEval(string domain,int K, double kIQR)
{
    vector<float> rst_meval;
    cnt_++;
    vector<int> scope;
    GetScopeIndices(domain,scope);

    if(scope.size()!=0){        
        rst_meval.resize(scope.size());
        pcl::search::KdTree<PointType>::Ptr kdtree_active(new pcl::search::KdTree<PointType>);
        pcl::PointCloud<PointType>::Ptr cloud_active(new pcl::PointCloud<PointType>);
        pcl::copyPointCloud(*cloud_,scope,*cloud_active);

        kdtree_active->setInputCloud(cloud_active);
    
        // #pragma omp parallel for
        for(int i=0;i<cloud_active->points.size();i++){
            pcl::PointCloud<PointType>::Ptr ctmp(new pcl::PointCloud<PointType>);
            vector<int> idx;
            vector<float> dist;
            kdtree_active->nearestKSearch(i,K,idx,dist);
            pcl::copyPointCloud(*cloud_active,idx,*ctmp);
            double eval_tmp=GetMEval(ctmp);                 
            rst_meval[i]=eval_tmp;
        }        
        double thresh=TukeyFence(rst_meval,kIQR);

        // #pragma omp parallel for
        for(int i=0;i<scope.size();i++){
            if(rst_meval[i]>thresh)
                status_[scope[i]]=-cnt_;
            else
                status_[scope[i]]=cnt_;
        }
        
        vector<V3> cl=ToColor(rst_meval);
        for(int i=0;i<cl.size();i++){
            cloud_active->points[i].r=cl[i].r;
            cloud_active->points[i].g=cl[i].g;
            cloud_active->points[i].b=cl[i].b;
        }
    }
    else{
        cout<<"Meval no input cloud"<<endl;
    } 
}

void HybridMethod::RegionGrowth_kIQR(string domain, double level, double thresh_kIQR)
{
    /* Step 01: Init */
    cnt_++;
    vector<int> scope;
    GetScopeIndices(domain, scope);    

    /* Step 02: copy point cloud */
    pcl::PointCloud<PointType>::Ptr cloud_active(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*cloud_,scope,*cloud_active);

    /* Step 03: */
    pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters);
    pcl::IndicesClustersPtr small_clusters (new pcl::IndicesClusters);
    pcl::IndicesClustersPtr large_clusters (new pcl::IndicesClusters);
    pcl::search::KdTree<PointType>::Ptr kdtree_tmp(new pcl::search::KdTree<PointType>);

    PointType vmin,vmax;
    pcl::getMinMax3D(*cloud_active, vmin, vmax);
    double box_max=MAX3(vmax.x-vmin.x,vmax.y-vmin.y,vmax.z-vmin.z);
    double tolerance=box_max/pow(2,level);

    // Set up a Conditional Euclidean Clustering class
    pcl::ConditionalEuclideanClustering<PointType> cec(true);
    cec.setInputCloud(cloud_active);
    cec.setConditionFunction(&customRegionGrowing);
    cec.setClusterTolerance(tolerance);
    cec.segment(*clusters);
    vector<int> cluster_size_set;
    for(int i=0;i<clusters->size();i++){
       cluster_size_set.push_back((*clusters)[i].indices.size());
    }

    /* Step 04: Threshold */
    double IQR=Quantile(cluster_size_set,0.75)-Quantile(cluster_size_set,0.25);
    double thresh=Quantile(cluster_size_set,0.75)+IQR*thresh_kIQR;
    cout<<"quantity of cluster:"<<cluster_size_set.size()<<endl;
    cout<<"cluster thresh:"<<thresh<<endl;

    for(int i=0;i<clusters->size();i++){
        int current_cluster_size=(*clusters)[i].indices.size();
        if(current_cluster_size<=thresh){
            for(int j=0;j<(*clusters)[i].indices.size();j++){
                int itmp=(*clusters)[i].indices[j];
                vector<int> idx;
                vector<float> dist;
                kdtree_->nearestKSearch(cloud_active->points[itmp],1,idx,dist);          
                status_[idx[0]]=-cnt_;
            }
        }
        else{
            for(int j=0;j<(*clusters)[i].indices.size();j++){
                int itmp=(*clusters)[i].indices[j];
                vector<int> idx;
                vector<float> dist;
                kdtree_->nearestKSearch(cloud_active->points[itmp],1,idx,dist);
                status_[idx[0]]=cnt_;
                cloud_->points[idx[0]].r=0;
                cloud_->points[idx[0]].g=255;
                cloud_->points[idx[0]].b=0; 
            }
        }
    }
}
void HybridMethod::RegionGrowth_Max(string domain, double level)
{
    /* Step 01: Init */
    cnt_++;
    vector<int> scope;
    GetScopeIndices(domain, scope);    

    /* Step 02: copy point cloud */
    pcl::PointCloud<PointType>::Ptr cloud_active(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*cloud_,scope,*cloud_active);

    /* Step 03: */
    pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters);
    pcl::IndicesClustersPtr small_clusters (new pcl::IndicesClusters);
    pcl::IndicesClustersPtr large_clusters (new pcl::IndicesClusters);
    pcl::search::KdTree<PointType>::Ptr kdtree_tmp(new pcl::search::KdTree<PointType>);

    PointType vmin,vmax;
    pcl::getMinMax3D(*cloud_active, vmin, vmax);
    double box_max=MAX3(vmax.x-vmin.x,vmax.y-vmin.y,vmax.z-vmin.z);
    double tolerance=box_max/pow(2,level);

    // Set up a Conditional Euclidean Clustering class
    pcl::ConditionalEuclideanClustering<PointType> cec(true);
    cec.setInputCloud(cloud_active);
    cec.setConditionFunction(&customRegionGrowing);
    cec.setClusterTolerance(tolerance);
    cec.segment(*clusters);
    vector<int> cluster_size_set;
    for(int i=0;i<clusters->size();i++){
       cluster_size_set.push_back((*clusters)[i].indices.size());
    }
    vector<int>::iterator it=std::max(cluster_size_set.begin(),cluster_size_set.end());
    int max_cluster_size=*it;

    /* Step 04: Threshold */
    for(int i=0;i<clusters->size();i++){
        int current_cluster_size=(*clusters)[i].indices.size();
        if(current_cluster_size==max_cluster_size){
            for(int j=0;j<(*clusters)[i].indices.size();j++){
                int itmp=(*clusters)[i].indices[j];         
                status_[scope[itmp]]=-cnt_;
            }
        }
        else{
            for(int j=0;j<(*clusters)[i].indices.size();j++){
                int itmp=(*clusters)[i].indices[j];
                status_[scope[itmp]]=cnt_;
            }
        }
    }
}

void HybridMethod::ExtractResult()
{
    int number_of_regular,number_of_irregular;
    number_of_regular=number_of_irregular=0;
    for(int i=0;i<status_.size();i++){
        if(status_[i]>0){
            number_of_regular++;
        }
        else if(status_[i]<0){            
            number_of_irregular++;
        }
        // else{
        //     cout<<"error: "<<i<<"   "<<status_[i]<<endl;
        // }
    }
    vector<int> idx_regular,idx_irregular;
    int i_regular,i_irregular;
    i_regular=i_irregular=0;
    idx_regular.resize(number_of_regular);
    idx_irregular.resize(number_of_irregular);
    for(int i=0;i<status_.size();i++){
        if(status_[i]>0){
            idx_regular[i_regular++]=i;
        }
        else{
            idx_irregular[i_irregular++]=i;
        }
    }

    if(idx_regular.size()>0)
        pcl::copyPointCloud(*cloud_,idx_regular,*cloud_out_regular_);
    
    if(idx_irregular.size()>0)
        pcl::copyPointCloud(*cloud_,idx_irregular,*cloud_out_irregular_);
}

void HybridMethod::LoOP(string domain,int K, double thresh)
{
    // cout<<"LoOP"<<endl;
    /* Step 01: Init */
    cnt_++;
    vector<int> scope;
    GetScopeIndices(domain, scope);   

    if(scope.size()>0){
        /* Step 02: copy point cloud */
        pcl::PointCloud<PointType>::Ptr cloud_active(new pcl::PointCloud<PointType>);
        pcl::copyPointCloud(*cloud_,scope,*cloud_active);


        pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
        kdtree->setInputCloud(cloud_active);
        vector<double> sigma;
        vector<double> plof;
        vector<double> rst_LoOP_;
        double nplof=0;
        // Resize Scores	
        sigma.resize(cloud_active->points.size());
        plof.resize(cloud_active->points.size());
        rst_LoOP_.resize(cloud_active->points.size());

        // Step 01: Calculate sigma
        // #pragma omp parallel for
        for(int i=0;i<cloud_active->points.size();i++){
            // find k-nearest neighours
            vector<int> idx(K+1);
            vector<float> dist(K+1);
            kdtree->nearestKSearch (i, K+1, idx, dist);
            // cout<<cloud->points[i]<<endl;
            double sum=0;
            for(int j=1;j<K+1;j++){
                sum+=dist[j];
            }
            sum=sum/K;
            sigma[i]=sqrt(sum);
        }
        
        // Step 02: calculate mean
        double mean=0;
        // #pragma omp parallel for
        for (int i = 0; i < cloud_active->points.size(); i++){        
            vector<int> idx(K+1);
            vector<float> dist(K+1);
            kdtree->nearestKSearch (cloud_active->points[i], K+1, idx, dist);
            double sum = 0;
            for (int j = 1; j < K+1; j++)
            sum += sigma[idx[j]];
            sum /= K;
            plof[i] = sigma[i] / sum  - 1.0f;				
            mean += plof[i] * plof[i];
        }
        nplof=sqrt(mean/cloud_active->points.size());	

        // Step 03: caculate score
        // #pragma omp parallel for
        for(int i=0;i<cloud_active->points.size();i++){
            double value = plof[i] / (nplof * sqrt(2.0f));
            // rst_.records_[i].item1_=value;

            double dem = 1.0 + 0.278393 * value;
            dem += 0.230389 * value * value;
            dem += 0.000972 * value * value * value;
            dem += 0.078108 * value * value * value * value;
            double op = std::max(0.0, 1.0 - 1.0 / dem);
            rst_LoOP_[i] = op;
        }

        vector<int> idx;
        for(int i=0;i<rst_LoOP_.size();i++){
            vector<int> idx;
            vector<float> dist;
            kdtree_->nearestKSearch(cloud_active->points[i],1,idx,dist);
            if(rst_LoOP_[i]<thresh){
                status_[idx[0]]=cnt_;
            }
            else{
                status_[idx[0]]=-cnt_;
            }        
        }
    }
    else
        cout<<"LoOP no input cloud"<<endl;
  
}

void HybridMethod::GetScopeIndices(string str, vector<int>& cIdx)
{
    vector<int> emnt; // elements
    Str2Vec(str,",",emnt);
    VecFindPos(status_,emnt,cIdx);
}

void HybridMethod::GenerateNoise()
{
    std::cout << "读取文件" << std::endl;

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>); // 创建点云（指针）

    pcl::io::loadPLYFile<PointType>("E:\\data\\12.ply", *cloud);
    std::cout << "正确查找文件" << std::endl;


    //添加高斯噪声
    pcl::PointCloud<PointType>::Ptr cloudfiltered(new pcl::PointCloud<PointType>());
    cloudfiltered->points.resize(cloud->points.size());//将点云的cloud的size赋值给噪声
    cloudfiltered->header = cloud->header;
    cloudfiltered->width = cloud->width;
    cloudfiltered->height = cloud->height;

    boost::mt19937 rng;
    rng.seed(static_cast<unsigned int>(time(0)));
    boost::normal_distribution<> nd(0, 2);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> var_nor(rng, nd);

    //添加噪声
    for (size_t point_i = 0; point_i < cloud->points.size(); ++point_i)
    {
        cloudfiltered->points[point_i].x = cloud->points[point_i].x + static_cast<float> (var_nor());
        cloudfiltered->points[point_i].y = cloud->points[point_i].y + static_cast<float> (var_nor());
        cloudfiltered->points[point_i].z = cloud->points[point_i].z + static_cast<float> (var_nor());
    }

    pcl::io::savePLYFileBinary("/home/llg/dataset/",*cloudfiltered);
}

void HybridMethod::STING_kIQR(string domain, int cell_level, int cluster_level, double threshold)
{
    // cout<<"STING_kIQR"<<endl;
    /* get active cloud */    
    vector<int> scope;
    GetScopeIndices(domain, scope);   
    pcl::PointCloud<PointType>::Ptr cloud_active(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*cloud_,scope,*cloud_active);

    /* run STING on active point cloud */    
    GridClustering gc(cloud_active);
    gc.OctreeProperties(cell_level);
    gc.RegionGrowth_kIQR(cluster_level,threshold);
    // gc.ExtractResult("/home/llg/Result/1.ply");

    /* indices (active cloud) -> indices (global cloud) */
    vector<int> nidx,pidx;
    FindCorrespondingIndices(kdtree_,gc.cloud_out_,pidx);
    VectorDifference(scope,pidx,nidx);

    /* extract indices */
    cnt_++;
    for(int i=0;i<pidx.size();i++)
        status_[pidx[i]]=cnt_;
    
    for(int i=0;i<nidx.size();i++)
        status_[nidx[i]]=-cnt_;    
}

void HybridMethod::STING_Max(string domain, int cell_level, int cluster_level)
{
    /* get active cloud */    
    vector<int> scope;
    GetScopeIndices(domain, scope);   
    pcl::PointCloud<PointType>::Ptr cloud_active(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*cloud_,scope,*cloud_active);

    /* run STING on active point cloud */    
    GridClustering gc(cloud_active);
    gc.OctreeProperties(cell_level);
    gc.RegionGrowth_Max(cluster_level);

    /* indices (active cloud) -> indices (global cloud) */
    vector<int> nidx,pidx;
    FindCorrespondingIndices(kdtree_,gc.cloud_out_,pidx);
    VectorDifference(scope,pidx,nidx);

    /* extract indices */
    cnt_++;
    for(int i=0;i<pidx.size();i++)
        status_[pidx[i]]=cnt_;
    
    for(int i=0;i<nidx.size();i++)
        status_[nidx[i]]=-cnt_;    
}

void HybridMethod::PlaneRefine(string domain, double level, double thresh_kIQR)
{
   
}