#include "PartManager.h"
#include <pcl/features/moment_of_inertia_estimation.h>

void PartManager::WriteIndices(string path,vector<int>& dat, string mode)
{
    if(mode=="binary"){
        FILE* fp=fopen(path.c_str(),"wb");        
        int tmp_n=dat.size();
        fwrite(&tmp_n,sizeof(int),1,fp);
        fwrite(dat.data(),sizeof(int),tmp_n,fp);
        fclose(fp);
    }
    else if(mode=="ascii"){     
        ofstream fout(path);    
        int tmp_n=dat.size();
        fout<<tmp_n<<endl;  
        for(int j=0;j<dat.size();j++)
            fout<<dat[j]<<endl;
        fout.close();
    }
    else{
        cout<<"PartManager error: writeindices mode!"<<endl;
    }
}

void PartManager::ReadIndices(string path,vector<int>& dat,string mode)
{
    if(mode=="binary"){
        FILE* fp=fopen(path.c_str(),"rb");
        int n=0;
        fread(&n,sizeof(int),1,fp);
        dat.resize(n);
        fread(dat.data(),sizeof(int),n,fp);
        fclose(fp);
    }
    else{
       cout<<"PartManager error: readindices mode error!"<<endl; 
    }   
}

void PartManager::get_raw_patch_from_low_indices(string path_to_low_indices,string path_to_RPC,string path_to_save_row_patch)
{
    vector<int> lidx,ridx;
    ReadIndices(path_to_low_indices,lidx);

    pcl::PointCloud<PointType>::Ptr cloud_low(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr ctmp2(new pcl::PointCloud<PointType>);
    pcl::io::loadPLYFile("/home/llg/dataset_upholstery/2_OR1/huizhongli_8000000_unique_OR1_regular_hough.ply",*cloud_low);
    for(int i=0;i<lidx.size();i++){
        ctmp2->points.push_back(cloud_low->points[lidx[i]]);
    }
    pcl::io::savePLYFileBinary("/home/llg/dataset_upholstery/3_Segmentation/low_patch/0.ply",*ctmp2);

    ht_.Init(path_to_ht);
    for(int i=0;i<lidx.size();i++){
        vector<int> tmp;
        ht_.ReadLine(lidx[i],tmp);
        ridx.insert(ridx.end(),tmp.begin(),tmp.end());
    }

    ppr_.ReadInit(path_to_RPC);
    pcl::PointCloud<PointType>::Ptr ctmp(new pcl::PointCloud<PointType>);
    ctmp->points.resize(ridx.size());
    for(int i=0;i<ridx.size();i++){
        ppr_.ReadLine(ridx[i],ctmp->points[i]);
    }
    pcl::io::savePLYFileBinary(path_to_save_row_patch,*ctmp);
}

void PrintPoint(PointType& pt,string delimiter)
{
    cout<<pt.x<<delimiter<<pt.y<<delimiter<<pt.z<<endl;
}
void PrintPoint(Eigen::Vector3f& pt,string delimiter)
{
    cout<<pt[0]<<delimiter<<pt[1]<<delimiter<<pt[2]<<endl;
}

// void PartManager::Init(pcl::PointCloud<PointType>::Ptr cloud_segment)
// {
//     pcl::MomentOfInertiaEstimation<PointType> feature_extractor;
//     feature_extractor.setInputCloud(cloud_segment);
// 	feature_extractor.compute();

//     PointType min_point_AABB,max_point_AABB;	

//     feature_extractor.getAABB(min_point_AABB, max_point_AABB);
//     Eigen::Vector3f major_vector, middle_vector, minor_vector;
//     feature_extractor.getEigenVectors(major_vector,middle_vector, normal_);

//     double xw=max_point_AABB.x-min_point_AABB.x;
//     double yw=max_point_AABB.y-min_point_AABB.y;
//     double zw=max_point_AABB.z-min_point_AABB.z;

//     /* pmin and pmax */
//     Eigen::Vector3f pmin(min_point_AABB.x,min_point_AABB.y,min_point_AABB.z);
//     Eigen::Vector3f pmax(max_point_AABB.x,max_point_AABB.y,max_point_AABB.z);

//     /* get centre_ */
//     centre_=(pmin+pmax)/2.0;

//     /* eight corners*/
//     Eigen::Vector3f A(pmin[0],pmin[1],pmax[2]);
//     Eigen::Vector3f B(pmin[0],pmax[1],pmax[2]);
//     Eigen::Vector3f C(pmax[0],pmax[1],pmax[2]);
//     Eigen::Vector3f D(pmax[0],pmin[1],pmax[2]);
//     Eigen::Vector3f E(pmin[0],pmin[1],pmin[2]);
//     Eigen::Vector3f F(pmin[0],pmax[1],pmin[2]);
//     Eigen::Vector3f G(pmax[0],pmax[1],pmin[2]);
//     Eigen::Vector3f H(pmax[0],pmin[1],pmin[2]);    

//     if(xw<yw && xw<zw){
//         corners_[0]=(A+D)/2.0;
//         corners_[1]=(B+C)/2.0;
//         corners_[2]=(F+G)/2.0;
//         corners_[3]=(E+H)/2.0;      
//     }
//     else if(yw<xw && yw<zw){
//         corners_[0]=(A+B)/2.0;
//         corners_[1]=(E+F)/2.0;
//         corners_[2]=(H+G)/2.0;
//         corners_[3]=(D+C)/2.0;   
//     }
//     else if(zw<yw && zw<xw){
//         corners_[0]=(A+E)/2.0;
//         corners_[1]=(B+F)/2.0;
//         corners_[2]=(C+G)/2.0;
//         corners_[3]=(D+H)/2.0;      
//     }
// }


void PartManager::RANSAC(pcl::PointCloud<PointType>::Ptr cloud_segment)
{
    vector<int> inliers;//用于存放合群点的vector
    pcl::SampleConsensusModelPlane<PointType>::Ptr model(new pcl::SampleConsensusModelPlane<PointType>(cloud_segment));//定义待拟合平面的model，并使用待拟合点云初始化
    pcl::RandomSampleConsensus<PointType> ransac(model);//定义RANSAC算法模型
    ransac.setDistanceThreshold(0.05);//设定阈值
    ransac.computeModel();//拟合
    ransac.getInliers(inliers);//获取合群点
    vector<int> tmp;
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);//获取拟合平面参数，对于平面ax+by_cz_d=0，coeff分别按顺序保存a,b,c,d
    normal_[0]=coeff[0];
    normal_[1]=coeff[1];
    normal_[2]=coeff[2];
    normal_.normalize();

    a_=coeff[0];
    b_=coeff[1];
    c_=coeff[2];
    d_=coeff[3];


    vector<float> dist;
    dist.resize(cloud_segment->points.size());
    // #pragma omp parallel for
    for(int i=0;i<dist.size();i++){
        float denominator=abs(a_*cloud_segment->points[i].x+b_*cloud_segment->points[i].y+c_*cloud_segment->points[i].z+d_);
        float nominator=sqrt(a_*a_+b_*b_+c_*c_);
        dist[i]=denominator/nominator;
    }
    float dist_min=*std::min_element(dist.begin(),dist.end());
    float dist_max=*std::max_element(dist.begin(),dist.end());

    #pragma omp parallel for
    for(int i=0;i<dist.size();i++){        
        V3 ctmp=get_color(dist_min,dist_max,dist[i]);
        cloud_segment->points[i].r=ctmp.r;
        cloud_segment->points[i].g=ctmp.g;
        cloud_segment->points[i].b=ctmp.b;
    }
    pcl::io::savePLYFileBinary("result2/wokaka.ply",*cloud_segment);
}

double PartManager::GetDistance(PointType& pt)
{
    float numerator=abs(a_*pt.x + b_*pt.y + c_*pt.z + d_);
    float denominator=sqrt(a_*a_+b_*b_+c_*c_);
    float dist=numerator/denominator;    
    return dist;
}


bool PartManager::Intersection(Eigen::Vector3f& A, Eigen::Vector3f& B, Eigen::Vector3f& C, Eigen::Vector3f& D,Eigen::Vector3f& itsc)
{
    // cout<<"wokaka"<<endl;
    // cout<<A[0]<<","<<A[1]<<","<<A[2]<<endl;
    // cout<<B[0]<<","<<B[1]<<","<<B[2]<<endl;
    // cout<<C[0]<<","<<C[1]<<","<<C[2]<<endl;
    // cout<<D[0]<<","<<D[1]<<","<<D[2]<<endl;

    double xa=A[0]; double ya=A[1]; double za=A[2];
    double xb=B[0]; double yb=B[1]; double zb=B[2];
    double xc=C[0]; double yc=C[1]; double zc=C[2];
    double xd=D[0]; double yd=D[1]; double zd=D[2];


    Eigen::MatrixXd M(3,2);
    M<<xb-xa,xc-xd,yb-ya,yc-yd,zb-za,zc-zd;
    Eigen::MatrixXd b(3,1);
    b<<xc-xa,yc-ya,zc-za;
    Eigen::MatrixXd t=(M.transpose()*M).inverse()*M.transpose()*b;

    itsc=A+t(0,0)*(B-A);

    double t1=(itsc-A)[0]/(B-A)[0];
    double t2=(itsc-C)[0]/(D-C)[0];

    if(t1>0 && t2<=1 && t2>=0)
        return true;
    else
        return false;
}

void PartManager::AdjustOrder(Eigen::Vector3f& A,Eigen::Vector3f& B,Eigen::Vector3f& C,Eigen::Vector3f& D)
{
    Eigen::Vector3f itsc;
    bool flag1=Intersection(A,B,C,D,itsc);
    bool flag2=Intersection(A,C,B,D,itsc);
    bool flag3=Intersection(A,D,B,C,itsc);
    if(flag1==true && flag2==false && flag3==false){
        Eigen::Vector3f tmp;
        // exchange B and C
        tmp=B;
        B=C;
        C=tmp;
    }
    else if(flag1==false && flag2==true && flag3==false){        
        // nice, do not require to chagne
    }
    else if(flag1==false && flag2==false && flag3==true){
        Eigen::Vector3f tmp;
        // exchange C and D
        tmp=C;
        C=D;
        D=tmp;
    }
    else{
        // cout<<"pid:"<<pid<<endl;
        // cout<<A[0]<<","<<A[1]<<","<<A[2]<<endl;
        // cout<<B[0]<<","<<B[1]<<","<<B[2]<<endl;
        // cout<<C[0]<<","<<C[1]<<","<<C[2]<<endl;
        // cout<<D[0]<<","<<D[1]<<","<<D[2]<<endl;
        // cout<<"Intersection Error!"<<endl;
        bool flag8=Intersection(A,B,C,D,itsc);
    }
}

bool PartManager::IsInside(PointType& pt)
{
    Eigen::Vector3f foot(pt.x,pt.y,pt.z);
    // cout<<endl<<"foot:"<<endl;
    // cout<<foot[0]<<","<<foot[1]<<","<<foot[2]<<endl;

    Eigen::Vector3f A= corners_[0];
    Eigen::Vector3f B= corners_[1];
    Eigen::Vector3f C= corners_[2];
    Eigen::Vector3f D= corners_[3];
    // cout<<endl<<"before:"<<endl;
    // cout<<A[0]<<","<<A[1]<<","<<A[2]<<endl;
    // cout<<B[0]<<","<<B[1]<<","<<B[2]<<endl;
    // cout<<C[0]<<","<<C[1]<<","<<C[2]<<endl;
    // cout<<D[0]<<","<<D[1]<<","<<D[2]<<endl;

    AdjustOrder(A,B,C,D);
    // cout<<endl<<"after:"<<endl;
    // cout<<A[0]<<","<<A[1]<<","<<A[2]<<endl;
    // cout<<B[0]<<","<<B[1]<<","<<B[2]<<endl;
    // cout<<C[0]<<","<<C[1]<<","<<C[2]<<endl;
    // cout<<D[0]<<","<<D[1]<<","<<D[2]<<endl;
    
    Eigen::Vector3f itsc_BC,itsc_CD,itsc_AD;
    Eigen::Vector3f mid_AB=(A+B)/2;
    bool is_itsc1=Intersection(foot,mid_AB,B,C,itsc_BC);
    bool is_itsc2=Intersection(foot,mid_AB,C,D,itsc_CD);
    bool is_itsc3=Intersection(foot,mid_AB,A,D,itsc_AD);
    // cout<<endl<<"itsc"<<endl;
    // cout<<itsc_BC[0]<<","<<itsc_BC[1]<<","<<itsc_BC[2]<<endl;
    // cout<<itsc_CD[0]<<","<<itsc_CD[1]<<","<<itsc_CD[2]<<endl;
    // cout<<itsc_AD[0]<<","<<itsc_AD[1]<<","<<itsc_AD[2]<<endl;    
    
    if(is_itsc1==false && is_itsc2==false && is_itsc3==false)
    {
        return true;
    }
    else 
        return false;
}

void PartManager::GetFoot(PointType& pt, Eigen::Vector3f& foot)
{
    /* Init */
    foot[0]=-(- pt.x*b_*b_ + a_* pt.y *b_ - pt.x *c_*c_ + a_*pt.z*c_ + a_*d_)/(a_*a_ + b_*b_ + c_*c_);
    foot[1]=-(- pt.y*a_*a_ + b_* pt.x *a_ - pt.y *c_*c_ + b_*pt.z*c_ + b_*d_)/(a_*a_ + b_*b_ + c_*c_);
    foot[2]=-(- pt.z*a_*a_ + c_* pt.x *a_ - pt.z *b_*c_ + c_*pt.y*b_ + c_*d_)/(a_*a_ + b_*b_ + c_*c_);
}


bool PartManager::is_plane_with_AABB(pcl::PointCloud<PointType>::Ptr cloud)
{
    pcl::MomentOfInertiaEstimation<PointType> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();

	std::vector <float> moment_of_inertia;
	std::vector <float> eccentricity;

	PointType min_point_AABB;//AABB包围盒
	PointType max_point_AABB;

	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;

	feature_extractor.getMomentOfInertia(moment_of_inertia);
	feature_extractor.getEccentricity(eccentricity);
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);
}

bool PartManager::is_plane_with_OBB(pcl::PointCloud<PointType>::Ptr cloud)
{
    pcl::MomentOfInertiaEstimation <PointType> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();

	std::vector <float> moment_of_inertia;
	std::vector <float> eccentricity;
	PointType min_point_OBB;
	PointType max_point_OBB;
	PointType position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;

	feature_extractor.getMomentOfInertia(moment_of_inertia);
	feature_extractor.getEccentricity(eccentricity);
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	// feature_extractor.getEigenValues(major_value, middle_value, minor_value);
	// feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	// feature_extractor.getMassCenter(mass_center);
}
