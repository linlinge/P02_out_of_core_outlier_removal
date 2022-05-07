#include "pipeline.h"
#include <pcl/surface/mls.h>

void Pipeline::Downsampling(string load_path, double num_or_ratio, pcl::PointCloud<PointType>::Ptr cloud_out)
{
	PlyParsing ppr;	
    ppr.ReadInit(load_path);
    ppr.PrintHeader(load_path);
	ppr.LoadPly(num_or_ratio,cloud_out);
    ppr.ReadClose();
    // cloud_out=LPC_;
}

bool custom_euclidean_and_normal(const PointType& point_a, const PointType& point_b, float squared_distance)
{
    Eigen::Vector3f v1(point_a.normal_x,point_a.normal_y,point_a.normal_z);
    Eigen::Vector3f v2(point_b.normal_x,point_b.normal_y,point_b.normal_z);
    double arc=acos(v1.dot(v2));
    if(arc< 0.0349 || (M_PI-arc)<0.0349)  // M_PI*2.0 / 180.0 
        return true;
    else 
        return false;
}

void Pipeline::Segmentation_condition(pcl::PointCloud<PointType>::Ptr cloud_LPC, pcl::PointCloud<PointType>::Ptr cloud_out,string save_path)
{
    double tolerance=GetCellSize(cloud_LPC,8);
    pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters);
    pcl::ConditionalEuclideanClustering<PointType> cec(true);
    cec.setInputCloud(cloud_LPC);
    cec.setConditionFunction(&custom_euclidean_and_normal);
    cec.setClusterTolerance(tolerance);
    cec.segment(*clusters);

    #ifdef SEGMENTATION_EXTRACT_PATCH 
    for(int i=0;i<clusters->size();i++){
        string patch_name=save_path+"/"+to_string(i)+".txt";
        FILE* fp=fopen(patch_name.c_str(),"wb");        
        int tmp_n=(*clusters)[i].indices.size();
        fwrite(&tmp_n,sizeof(int),1,fp);
        fwrite((*clusters)[i].indices.data(),sizeof(int),tmp_n,fp);

        // for(int j=0;j<(*clusters)[i].indices.size();j++){
        //     // cloud_out->points[(*clusters)[i].indices[j]].r=rtmp;
        //     // cloud_out->points[(*clusters)[i].indices[j]].g=gtmp;
        //     // cloud_out->points[(*clusters)[i].indices[j]].b=btmp; 
        //     int tmp=(*clusters)[i].indices[j];
        //     fwrite(&tmp,sizeof(int))
        // }
        fclose(fp);
        break;
    }
    #else /* extract color cloud */
    pcl::copyPointCloud(*cloud_LPC,*cloud_out);    
    srand((int)time(0));
    for(int i=0;i<clusters->size();i++){
        int rtmp=rand()%255;
        int gtmp=rand()%255;
        int btmp=rand()%255;
        for(int j=0;j<(*clusters)[i].indices.size();j++){
            cloud_out->points[(*clusters)[i].indices[j]].r=rtmp;
            cloud_out->points[(*clusters)[i].indices[j]].g=gtmp;
            cloud_out->points[(*clusters)[i].indices[j]].b=btmp;
        }
    }
    #endif
}

void Pipeline::Segmentation_has_normal(pcl::PointCloud<PointType>::Ptr cloud_LPC,string path_to_save)
{
     pcl::search::Search<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    normals->points.resize(cloud_LPC->points.size());
    for(int i=0;i<normals->points.size();i++){
        normals->points[i].normal_x=cloud_LPC->points[i].normal_x;
        normals->points[i].normal_y=cloud_LPC->points[i].normal_y;
        normals->points[i].normal_z=cloud_LPC->points[i].normal_z;
    }

    pcl::RegionGrowing<PointType, pcl::Normal> reg;
    // reg.setMinClusterSize(20);
    // reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(20);
    reg.setInputCloud(cloud_LPC);
    //reg.setIndices (indices);
    reg.setInputNormals(normals);
    // reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);
    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);

    /* extract result */
    pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*cloud_LPC,*cloud_out);
    srand((int)time(0));
    for(int i=0;i<clusters.size();i++){
        int rtmp=rand()%255;
        int gtmp=rand()%255;
        int btmp=rand()%255;
        for(int j=0;j<clusters[i].indices.size();j++){
            cloud_out->points[clusters[i].indices[j]].r=rtmp;
            cloud_out->points[clusters[i].indices[j]].g=gtmp;
            cloud_out->points[clusters[i].indices[j]].b=btmp;
            cloud_out->points[clusters[i].indices[j]].normal_x=normals->points[clusters[i].indices[j]].normal_x;
            cloud_out->points[clusters[i].indices[j]].normal_y=normals->points[clusters[i].indices[j]].normal_y;
            cloud_out->points[clusters[i].indices[j]].normal_z=normals->points[clusters[i].indices[j]].normal_z;
        }
    }
    pcl::io::savePLYFileBinary(path_to_save,*cloud_out);
}
void Pipeline::Segmentation_has_normal(string path_to_raw, pcl::PointCloud<PointType>::Ptr cloud_LPC,string path_to_save_parts)
{
    pcl::search::Search<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    normals->points.resize(cloud_LPC->points.size());
    for(int i=0;i<normals->points.size();i++){
        normals->points[i].normal_x=cloud_LPC->points[i].normal_x;
        normals->points[i].normal_y=cloud_LPC->points[i].normal_y;
        normals->points[i].normal_z=cloud_LPC->points[i].normal_z;
    }

    pcl::RegionGrowing<PointType, pcl::Normal> reg;
    // reg.setMinClusterSize(0);
    // reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(50);
    reg.setInputCloud(cloud_LPC);
    //reg.setIndices (indices);
    reg.setInputNormals(normals);
    // reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setSmoothnessThreshold(3 / 180.0 * M_PI);
    reg.setCurvatureThreshold(10.0);
    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);

    /* extrat raw patches */
    #if SEGMENTATION_GET_RAW_PART_ON
        HashTable ht;            
        ht.Init(path_to_ht);

        // ht.Init();
        PlyParsing pps;
        pps.Init(path_to_raw);

        #pragma omp parallel for
        for(int i=0;i<clusters.size();i++){     
            // if(clusters[i].indices.size()>100){
                // cout<<i<<endl;

                vector<int> ridx;
                for(int j=0;j<clusters[i].indices.size();j++){
                    vector<int> itmp;
                    // cout<<"line:"<<clusters[i].indices[j]<<endl;
                    ht.ReadElements(clusters[i].indices[j],itmp);
                    ridx.insert(ridx.end(),itmp.begin(),itmp.end());
                }

                pcl::PointCloud<PointType>::Ptr ctmp(new pcl::PointCloud<PointType>);
                ctmp->points.resize(ridx.size());
                for(int j=0;j<ridx.size();j++){
                    PointType ptmp;
                    pps.ReadLine(ptmp,ridx[j]);
                    ctmp->points[j].x=ptmp.x;
                    ctmp->points[j].y=ptmp.y;
                    ctmp->points[j].z=ptmp.z;
                    ctmp->points[j].r=ptmp.r;
                    ctmp->points[j].g=ptmp.g;
                    ctmp->points[j].b=ptmp.b;
                }
                // cout<<ctmp->points.size()<<endl;
                // cout<<path_to_save_parts+"/"+to_string(i)+".ply"<<endl;
                // pcl::io::savePLYFileBinary(path_to_save_parts+"/"+to_string(i)+".ply",*ctmp);

                #if SEGMENTATION_GET_LOW_PART_ON
                pcl::PointCloud<PointType>::Ptr ctmp_low(new pcl::PointCloud<PointType>);
                ctmp_low->points.resize(clusters[i].indices.size());
                for(int j=0;j<clusters[i].indices.size();j++){
                    ctmp_low->points[j].x=cloud_LPC->points[clusters[i].indices[j]].x;
                    ctmp_low->points[j].y=cloud_LPC->points[clusters[i].indices[j]].y;
                    ctmp_low->points[j].z=cloud_LPC->points[clusters[i].indices[j]].z;
                    ctmp_low->points[j].r=cloud_LPC->points[clusters[i].indices[j]].r;
                    ctmp_low->points[j].g=cloud_LPC->points[clusters[i].indices[j]].g;
                    ctmp_low->points[j].b=cloud_LPC->points[clusters[i].indices[j]].b;
                }            
                cout<<"/home/llg/dataset_upholstery/04_Segmentation/low_patch/"+to_string(i)+".ply"<<endl;
                pcl::io::savePLYFileBinary("/home/llg/dataset_upholstery/04_Segmentation/low_patch/"+to_string(i)+".ply",*ctmp_low);
                #endif
            // }                          
        }
    #elif SEGMENTATION_GET_LOW_INDICES_ON 
         for(int i=0;i<clusters.size();i++){
            string patch_binary_name=path+"/"+to_string(i)+".bin";
            PartManager pm;
            if(clusters[i].indices.size()>100)
                pm.WriteIndices(patch_binary_name,clusters[i].indices);
        }
    #endif
}

void Pipeline::DealWithPlaneLikeSegments(int cluster_id, vector<int>& indices_in_cluster, string path_to_raw, HashTable& ht)
{
    vector<int> cluster;
    for(int j=0;j<indices_in_cluster.size();j++){
        int itmp=indices_in_cluster[j];	
        cluster.insert(cluster.end(),ht.dat_[itmp].begin(),ht.dat_[itmp].end());
    }

    pcl::PointCloud<PointType>::Ptr ctmp(new pcl::PointCloud<PointType>);
    PlyParsing pps;
    pps.ReadLines(path_to_raw,cluster,ctmp);

    HybridMethod hm(ctmp);
    // hm.STING("0",9,8,10);
    // hm.LNGD("0",40,40);	
    hm.LoOP("0");
    hm.STING_Max("1",9,8);
    hm.ExtractResult();
    if(hm.cloud_out_regular_->points.size()>0)
        pcl::io::savePLYFileBinary("result/"+to_string(cluster_id)+".ply",*hm.cloud_out_regular_);
    else
        cout<<"error!"<<endl;
}

void Pipeline::Indexing(pcl::PointCloud<PointType>::Ptr cloud,string filepath)
{
    hash_table_.resize(cloud->points.size());
    pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
    kdtree->setInputCloud(cloud);
    ppr_.ReadInit(filepath);
    // #pragma omp parallel for
    for(long i=0;i<ppr_.vertex_num_;i++){
        PointType pts;
        ppr_.ReadLine(i,pts);
        vector<int> idx;
        vector<float> dist;
        kdtree->nearestKSearch(pts,1,idx,dist);
        hash_table_[idx[0]].push_back(i);
    }
    ppr_.ReadClose();
    cout<<hash_table_.size()<<endl;
}

void Pipeline::Merging(string read_files_path, string write_file_path)
{
    vector<string> filepath,filename;
    list_all_files(read_files_path,filepath,filename);
    for(int i=0;i<filepath.size();i++){
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
        pcl::io::loadPLYFile(filepath[i],*cloud);        
    }
}