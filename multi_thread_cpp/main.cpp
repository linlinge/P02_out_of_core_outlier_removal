#include <iostream>	
#include <fstream>
#include "PCLExtend.h"
#include "FileExtend.h"
#include <Eigen/Dense>
#include "V3.hpp"
#include <vector>
#include "pipeline.h"
#include "global.h"
#include "HybridMethod.h"
#include "GridClustering.h"
#include <HashTable.h>
#include "PartManager.h"
using namespace std;

#include <ctime>
void Delay(int   time) //time*1000为秒数 
{
	clock_t now = clock();
	while(clock()-now<time);
}

int main(int argc,char** argv)
{
	Pipeline ppl;
	string mode=argv[1];
	if("in_core"==mode){
		string path_to_raw=argv[2];
		pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
		pcl::io::loadPLYFile(path_to_raw,*cloud);
	}
	else if("loop"==mode){
		string path_to_raw=argv[2];
		pcl::PointCloud<PointType>::Ptr cloud_raw(new pcl::PointCloud<PointType>);
		PlyParsing ppw,ppr;		
		
		ppr.ReadInit(path_to_raw);
		cloud_raw->resize(ppr.vertex_num_);
		for(int i=0;i<ppr.vertex_num_;i++){
			PointType ptmp;
			ppr.ReadLine(i,cloud_raw,i);				
		}
		ppr.ReadClose();

		HybridMethod hm(cloud_raw);
		hm.LoOP();
		hm.ExtractResult();
		ppw.WriteInit(argv[3],POSITION_COLOR,hm.cloud_out_regular_->points.size());
		for(int i=0;i<hm.cloud_out_regular_->points.size();i++){
			ppw.AppendLine(cloud_raw->points[i]);
		}
		ppw.WriteClose();
	}
	else if("resampling"==mode){
		int num=10000000;

		PlyParsing ppr,ppw;
		ppr.ReadInit("/home/llg/workspace/experiment_massive/T02/ply/luming_crop.ply");
		ppw.WriteInit("/home/llg/workspace/experiment_massive/T02/ply/luming_crop_10^7.ply",POSITION_COLOR,num);				
		for(int i=0;i<num;i++){
			int idx=rand()%ppr.vertex_num_;
			PointType pts;
			ppr.ReadLine(idx,pts);
			ppw.AppendLine(pts);	
		}
		ppr.ReadClose();
		ppw.WriteClose();
	}
	else if("grid_sampling"==mode){
		// 创建点云对象
		pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
		pcl::PointCloud<PointType>::Ptr filteredCloud(new pcl::PointCloud<PointType>);
		pcl::io::loadPLYFile<PointType>(argv[2], *cloud);
		// 创建滤波对象
		pcl::VoxelGrid<PointType> filter;
		filter.setInputCloud(cloud);
		// 设置体素栅格的大小为 1x1x1cm
		double cell_size=atof(argv[3]);
		filter.setLeafSize(cell_size,cell_size,cell_size);
		filter.filter(*filteredCloud);

		ofstream fout(argv[4],ios::out);
		fout<<filteredCloud->points.size()<<endl;	
		cout<<endl<<"size of output: "<<filteredCloud->points.size()<<endl;	
		fout.close();
	}
	else if("01_Downsampling"==mode){ // mproc test_sampling path_to_raw 1000 path_to_save
		string path_to_raw=argv[2];
		double num_or_ratio=strtod(argv[3],NULL);
		// string path_to_save=argv[4];
		pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>);

		ppl.Downsampling(path_to_raw,num_or_ratio,cloud_out);
		// pcl::io::savePLYFileBinary(path_to_save,*cloud_out);
	}
	else if("02_OR1"==mode){
		pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
		pcl::io::loadPLYFile(argv[2],*cloud);
		
		HybridMethod hm(cloud);
		hm.LoOP("0");
		hm.STING_kIQR("1",9,8,40);
		hm.ExtractResult();
	}
	else if("03_Indexing"==mode){ // mproc 03_Indexing_threshold_binary path_to_raw path_to_low 
		pcl::PointCloud<PointType>::Ptr LPC(new pcl::PointCloud<PointType>);
		pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
		pcl::io::loadPLYFile<PointType>(argv[3],*LPC);
		kdtree->setInputCloud(LPC);

		// PLY read Init		
		PlyParsing pps;
		pps.ReadInit(argv[2]);
		HashTable ht;	
		ht.Resize(LPC->points.size());
		int count=0;

		// #pragma omp parallel for
		for(int i=0;i<pps.vertex_num_;i++){	
			PointType pts;
			pps.ReadLine(i,pts);
			vector<int> idx;
			vector<float> dist;
			kdtree->nearestKSearch(pts,1,idx,dist);
			dist[0]=sqrt(dist[0]);
			if(dist[0]<0.3)
				ht.PushBack(idx[0],i);
		}
		
		ht.SaveBinary(path_to_ht);
	}
	else if("04_Segmentation"==mode){ // mproc 04_Segmentation path_to_raw path_to_low path_to_save_parts
		pcl::PointCloud<PointType>::Ptr cloud_LPC(new pcl::PointCloud<PointType>);
		pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>);
		pcl::io::loadPLYFile(argv[3],*cloud_LPC);
		// ppl.Segmentation_condition(cloud_LPC,cloud_out,"/home/llg/dataset_upholstery/3_Segmentation/patches");
		
		// ppl.Segmentation_has_normal(argv[2],cloud_LPC,argv[4]);
		ppl.Segmentation_has_normal(cloud_LPC,"/home/llg/dataset_upholstery/04_Segmentation/low_color.ply");
		// ppl.Segmentation_has_normal(cloud_LPC,"/home/llg/dataset_upholstery/3_Segmentation/low_color.ply","get_color_low_cloud");
		// ppl.Segmentation_estimate_normal(cloud_LPC,cloud_out,"/home/llg/dataset_upholstery/3_Segmentation/patches");
		// pcl::io::savePLYFileBinary(argv[3],*cloud_out);
	}
	else if("05_OR2"==mode){
		int pid;
		vector<string> filepath,filename;
        list_all_files(argv[2],filepath,filename);

		/* read all ply iteratively */
        // #pragma omp parallel for
		for(int i=0;i<filepath.size();i++){				
				pid=get_num_in_string(filename[i]);				
				if(pid==89){ //10 30  // 47 105	
						cout<<"pid: "<<pid<<endl;											
						pcl::PointCloud<PointType>::Ptr cloud_segment(new pcl::PointCloud<PointType>);
						pcl::io::loadPLYFile<PointType>(filepath[i],*cloud_segment);						
						HybridMethod hm(cloud_segment);
						hm.OR2();
						hm.PlaneRefine("-1",8,40.0);						
						string path_to_save=argv[3];
						path_to_save=path_to_save+"/"+to_string(pid)+".ply";						
						hm.ExtractResult();
						pcl::io::savePLYFileBinary(path_to_save,*hm.cloud_out_regular_);
				}
		}
	}
	else if("06_Merging"==mode){
		vector<string> filepath,filename;
		list_all_files(argv[2],filepath,filename);		
		int vnum=0;		
		PlyParsing ppw;
		/* write head */
		for(int i=0;i<filepath.size();i++){							
			vnum+=ppw.ReadVertexNumber(filepath[i]);					
		}
		ppw.WriteInit(argv[3],POSITION_COLOR,vnum);		
		// ppw.WriteInit(argv[3],POSITION,10);
		cout<<"vnum:"<<vnum<<endl;
		
		/* write entry */
		int cnt=0;
		for(int i=0;i<filepath.size();i++){
			pcl::PointCloud<PointType>::Ptr ctmp(new pcl::PointCloud<PointType>);
			pcl::io::loadPLYFile(filepath[i],*ctmp);
			for(int j=0;j<ctmp->points.size();j++){
				ppw.AppendLine(ctmp->points[j]);
				cnt++;
			}				
		}
		ppw.WriteClose();
		cout<<"cnt:"<<cnt<<endl;
	}
	else if("print_header"==mode){
		PlyParsing pps;
		pps.PrintHeader(argv[2]);
	}
	else if("remove_by_aabb"==mode){ // mproc remove_by_aabb path_to_raw_part path_to_low_part
		string path_to_raw_part=argv[2];
		string path_to_low_part=argv[3];
		pcl::PointCloud<PointType>::Ptr part_raw(new pcl::PointCloud<PointType>);
		pcl::io::loadPLYFile(path_to_raw_part,*part_raw);
		HybridMethod hm(part_raw);
		hm.RemoveByBoundingBox("0",path_to_low_part);
		// hm.ExtractResult();
		pcl::io::savePLYFileBinary("/home/llg/dataset_upholstery/1.ply",*hm.cloud_);
	}
	else if("coarse_outlier_removal"==mode){ 
		/* arguments */
		// string path_to_raw=argv[2];
		// string path_to_low=argv[3];
		// string path_to_low_cleaned=argv[4];
		// string path_to_hash_table=argv[5];
		string path_to_raw="/home/llg/workspace/experiment_massive/T02/ply/sg27_station2_intensity_rgb.ply";
		string path_to_low="/home/llg/workspace/experiment_massive/T02/ply/sg27_station4_intensity_rgb_dwn.ply";
		string path_to_low_cleaned="/home/llg/workspace/experiment_massive/T02/ply/sg27_station4_intensity_rgb_cleaned.ply";
		string path_to_hash_table="/home/llg/workspace/experiment_massive/T02/ply/sg27_station4_intensity_rgb_ht.txt";


		/* Downsampling */	
		cout<<"Downsampling"<<endl;	
		pcl::PointCloud<PointType>::Ptr cloud_low(new pcl::PointCloud<PointType>);
		ppl.Downsampling(path_to_raw,6000000,cloud_low);
		pcl::io::savePLYFileBinary(path_to_low,*cloud_low);

		/* OR1 */
		cout<<"OR1"<<endl;
		HybridMethod hm(cloud_low);
		hm.LNGD("0",20,50);
		hm.STING_kIQR("1",9,8,800);
		hm.LoOP("2",5,0.9);
		hm.ExtractResult();
		cloud_low=hm.cloud_out_regular_;
		pcl::io::savePLYFileBinary(path_to_low_cleaned,*cloud_low);		

		cout<<cloud_low->points.size()<<endl;
		/* Indexing */
		cout<<"Indexing"<<endl;	
		pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);		
		kdtree->setInputCloud(cloud_low);

		/* Write hash table */
		PlyParsing ppr;
		ppr.ReadInit(path_to_raw);
		HashTable ht;
		ht.Resize(cloud_low->points.size());

		for(int i=0;i<ppr.vertex_num_;i++){
			PointType pts;	
			ppr.ReadLine(i, pts);
			vector<int> idx;
			vector<float> dist;
			kdtree->nearestKSearch(pts,1,idx,dist);
			if(idx[0]>=0){
				dist[0]=sqrt(dist[0]);
				if(dist[0]<0.05){
					ht.PushBack(idx[0],i);
				}
			}
			else{
				cout<<"read point cloud error!"<<endl;
				break;
			}			
		}		
		ht.SaveAscii(path_to_hash_table);
	}
	else if("refinement_outlier_removal"==mode){
		// system("rm -rf result/*");	
		string path_to_raw="/home/i9/experiment_massive/Area_2_noise.ply";
		string path_to_component="/home/i9/experiment_massive/Area_2_noise_dwn_cleaned_components.txt";
		string path_to_ht="/home/i9/experiment_massive/ht.txt";
		
		
		vector<int> small_idx;
		HashTable ht;
		ht.ReadLines(path_to_ht,"ascii");
		cout<<"start or"<<endl;

		// Delay(20000);	
		
		fstream fin(path_to_component,ios::in);
		int k=0;
		while(!fin.eof()){
			string line;			
			vector<string> ss;
			vector<int> cluster;

			fin>>line;
			StrSplit(line," ",ss);
			for(int i=0;i<ss.size();i++){
				 int itmp=atoi(ss[i].c_str());
				 cluster.insert(cluster.end(),ht.dat_[itmp].begin(),ht.dat_[itmp].end());
			}

			// make sure LPC segment has corresponding RPC segment
			if(cluster.size()>2){								
				pcl::PointCloud<PointType>::Ptr ctmp;
				ctmp=std::make_shared<pcl::PointCloud<PointType>>();
				PlyParsing pps;
				pps.ReadLines(path_to_raw,cluster,ctmp);	

			
				HybridMethod hm(ctmp);
				// hm.OR2("0",9,1);					
				hm.FM_MEval("0",40,3);
				// hm.STING_Max("1",10,9);					
				hm.LoOP("1",10,0.6);					
				hm.LoOP("2",10,0.8);
				hm.ExtractResult();					
				// hm.Clear();			

				if(hm.cloud_out_regular_->points.size()>0){
					PlyParsing pwtmp;
					// cout<<"../../result/"+to_string(k++)+".ply"<<endl;
					pwtmp.WriteInit("../../result/"+to_string(k)+".ply",POSITION_COLOR,hm.cloud_out_regular_->points.size());
					for(int j=0;j<hm.cloud_out_regular_->points.size();j++){
						pwtmp.AppendLine(hm.cloud_out_regular_->points[j]);						
					}
					pwtmp.WriteClose();
				}											
			}		
			// else{
			// 	cout<<"LPC segment have not found the corresponding RPC segment"<<endl;
			// }	

			k++;							
		}		

		// // merge
		// cout<<"merge:"<<endl;
		// vector<string> filepath,filename;
		// list_all_files("result",filepath,filename);
		// int vnum=0;		
		// PlyParsing ppsw;
		// /* write head */
		// for(int i=0;i<filepath.size();i++){							
		// 	vnum+=ppsw.ReadVertexNumber(filepath[i]);					
		// }
		// ppsw.WriteInit(path_to_merge,POSITION_COLOR,vnum);		
		// cout<<"vnum:"<<vnum<<endl;
		
		// /* write entry */
		// int cnt=0;
		// for(int i=0;i<filepath.size();i++){
		// 	pcl::PointCloud<PointType>::Ptr ctmp(new pcl::PointCloud<PointType>);
		// 	pcl::io::loadPLYFile(filepath[i],*ctmp);
		// 	for(int j=0;j<ctmp->points.size();j++){
		// 		ppsw.AppendLine(ctmp->points[j]);
		// 		cnt++;
		// 	}
		// }
		// ppsw.WriteClose();
		// cout<<"cnt:"<<cnt<<endl;
	}
	else if("Merge"==mode){
		string path_to_segments="../../result/";
		string path_to_merge="/home/i9/experiment_massive/Area_2_noise_dwn_cleaned_merge.ply";
		// merge
		cout<<"merge:"<<endl;
		vector<string> filepath,filename;
		list_all_files(path_to_segments,filepath,filename);
		int vnum=0;		
		PlyParsing ppsw;
		/* write head */
		for(int i=0;i<filepath.size();i++){			
			// cout<<filepath[i]<<endl;		
			vnum+=ppsw.ReadVertexNumber(filepath[i]);					
		}
		ppsw.WriteInit(path_to_merge,POSITION_COLOR,vnum);		
		cout<<"vnum:"<<vnum<<endl;
		
		/* write entry */
		int cnt=0;
		for(int i=0;i<filepath.size();i++){
			pcl::PointCloud<PointType>::Ptr ctmp(new pcl::PointCloud<PointType>);
			pcl::io::loadPLYFile(filepath[i],*ctmp);
			for(int j=0;j<ctmp->points.size();j++){
				ppsw.AppendLine(ctmp->points[j]);
				cnt++;
			}
		}
		ppsw.WriteClose();
		cout<<"cnt:"<<cnt<<endl;
	}
	else if("crop"==mode){	// mproc crop path_to_low path_to_raw path_to_save
		string path_to_low=argv[2];
		string path_to_raw=argv[3];
		string path_to_save=argv[4];
		PlyParsing ppr,ppw;		
		pcl::PointCloud<PointType>::Ptr cloud_low(new pcl::PointCloud<PointType>);
		pcl::io::loadPLYFile(path_to_low,*cloud_low);
		pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);		
		kdtree->setInputCloud(cloud_low);

		/* Indexing */	
		cout<<"Indexing"<<endl;		
		
		ppr.ReadInit(path_to_raw);
		vector<vector<int>> ht;	
		ht.resize(cloud_low->points.size());

		#pragma omp parallel for			
		for(int i=0;i<ppr.vertex_num_;i++){	
			PointType pts;			
			ppr.ReadLine(i,pts);
			

			vector<int> idx;
			vector<float> dist;
			kdtree->nearestKSearch(pts,1,idx,dist);
			dist[0]=sqrt(dist[0]);	
			if(dist[0]<0.5)		
				ht[idx[0]].push_back(i);
		}		
		

		vector<int> oidx;
		for(int i=0;i<ht.size();i++){
			oidx.insert(oidx.end(),ht[i].begin(),ht[i].end());
		}

		cout<<"extract cloud"<<endl;		
		ppw.WriteInit(path_to_save,POSITION_COLOR,oidx.size());
		for(int i=0;i<oidx.size();i++){
			PointType ptmp;
			ppr.ReadLine(oidx[i],ptmp);			
			ppw.AppendLine(ptmp);
		}
		ppr.ReadClose();
		ppw.WriteClose();
	}
	else if("f1_score"==mode){
		string path_to_gnd=argv[2];
		string path_to_raw=argv[3];
		double thresh=0.001;
		pcl::PointCloud<PointType>::Ptr cloud_raw(new pcl::PointCloud<PointType>);
		pcl::PointCloud<PointType>::Ptr cloud_gnd(new pcl::PointCloud<PointType>);
		pcl::io::loadPLYFile(path_to_raw,*cloud_raw);
		pcl::io::loadPLYFile(path_to_gnd,*cloud_gnd);
		// cout<<cloud_raw->points.size()<<endl;
		// cout<<cloud_gnd->points.size()<<endl;

		pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
		kdtree->setInputCloud(cloud_gnd);
		double precision=0;		
		
		#pragma omp parallel for
		for(int i=0;i<cloud_raw->points.size();i++){
			vector<int> idx;
			vector<float> dist;
			kdtree->nearestKSearch(cloud_raw->points[i],1,idx,dist);
			if(dist[0]<thresh){
				#pragma omp critical
				{
					precision+=1;
				}				
			}				
		}
		precision=precision*100.0/cloud_raw->points.size();
		// cout<<"precision:"<<precision<<endl;

		// recall
		double recall=0;		
		kdtree->setInputCloud(cloud_raw);		
		#pragma omp parallel for
		for(int i=0;i<cloud_gnd->points.size();i++){
			vector<int> idx;
			vector<float> dist;
			kdtree->nearestKSearch(cloud_gnd->points[i],1,idx,dist);
			if(dist[0]<thresh){
				#pragma omp critical
				{
					recall+=1;
				}				
			}				
		}
		recall=recall*100.0/cloud_gnd->points.size();
		// cout<<"recall:"<<recall<<endl;

		double fscore=2*precision*recall/(precision+recall);
		cout<<"fscore:"<<fscore<<endl;
	}
	else if("point_density"==mode){
		string path_to_raw=argv[2];
		string path_to_out=argv[3];
		pcl::PointCloud<PointType>::Ptr cloud_raw(new pcl::PointCloud<PointType>);
		pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
		pcl::io::loadPLYFile(path_to_raw,*cloud_raw);
		kdtree->setInputCloud(cloud_raw);
		float r=0.08;
		vector<int> density;
		density.resize(cloud_raw->points.size());
		#pragma omp parallel for
		for(int i=0;i<cloud_raw->points.size();i++){
			vector<int> idx;
			vector<float> dist;
			// kdtree->nearestKSearch(i,k+1,idx,dist);
			kdtree->radiusSearch(i,r,idx,dist);
			density[i]=idx.size();
		}
		ofstream fout(path_to_out);
		for(int i=0;i<cloud_raw->points.size();i++){
			fout<<density[i]<<endl;
		}
		fout.close();
	}
	else if("correspoinding_normals"==mode){
		string path_to_cloud1=argv[2];
		string path_to_cloud2=argv[3];
		string path_to_out=argv[4];
		pcl::PointCloud<PointType>::Ptr cloud_raw1(new pcl::PointCloud<PointType>);
		pcl::PointCloud<PointType>::Ptr cloud_raw2(new pcl::PointCloud<PointType>);		
		pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
		pcl::io::loadPLYFile(path_to_cloud1,*cloud_raw1);
		pcl::io::loadPLYFile(path_to_cloud2,*cloud_raw2);
		kdtree->setInputCloud(cloud_raw1);
		#pragma omp parallel for
		for(int i=0;i<cloud_raw2->points.size();i++){
			vector<int> idx;
			vector<float> dist;
			kdtree->nearestKSearch(cloud_raw2->points[i],1,idx,dist);						
			cloud_raw2->points[i].normal_x= cloud_raw1->points[idx[0]].normal_x;
			cloud_raw2->points[i].normal_y= cloud_raw1->points[idx[0]].normal_y;
			cloud_raw2->points[i].normal_z= cloud_raw1->points[idx[0]].normal_z;
		}
		pcl::io::savePLYFileBinary(path_to_out,*cloud_raw2);
	}
	else if("convert_to_xyz"==mode){
		PlyParsing ppr,ppw;
		ppr.ReadInit(argv[2]);
		ofstream fout(argv[3]);

		#pragma omp parallel for
		for(int i=0;i<ppr.vertex_num_;i++){
			PointType ptmp;
			ppr.ReadLine(i,ptmp);
			fout<<ptmp.x<<" "<<ptmp.y<<" "<<ptmp.z<<endl;			
		}
		ppr.ReadClose();
		fout.close();
	}
	return 0;
}