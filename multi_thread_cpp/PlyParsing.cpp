#include "PlyParsing.h"
int PlyParsing::ReadVertexNumber(string path)
{
    ifstream fin(path);
    string line;
    getline(fin,line);
    int flag=line.find("element vertex");
    while(flag==string::npos){
        getline(fin,line);
        flag=line.find("element vertex");        
    } 
    string tmp=line.substr(15,line.length()-14);
    int rst=stoi(tmp);
    fin.close();
    return rst;
}

void PlyParsing::ReadInit(string load_path)
{
    load_path_=load_path;
    fpr_=fopen(load_path_.c_str(),"rb");
    length_of_header_=0;
    fread((void*)(&char_header), sizeof(char_header), 1, fpr_);
    type_size_=0;
    pos_x_=pos_red_=pos_nx_=-1;
    string str_tmp=char_header;  

    /* Get number_of_point_cloud and length_of_header*/
    vertex_num_= atoi(str_tmp.substr(str_tmp.find("element vertex")+15, str_tmp.find("property float")-str_tmp.find("element vertex")-15).c_str());
    length_of_header_=str_tmp.find("end_header")+10;

    /* analysis position */
    vector<string> ss;
    StrSplit(str_tmp,"\n",ss);
    for(int i=0;i<ss.size();i++){
        if(ss[i].find("property float")!=string::npos){            
            if(ss[i].find("property float x")!=string::npos){
                pos_x_=type_size_;
            }
            type_size_+=4;
        }
        else if(ss[i].find("property uchar")!=string::npos){
            if(ss[i].find("property uchar red")!=string::npos){
                pos_red_=type_size_;
            }
            type_size_+=1;
        }
    }
    
    if((pos_x_ !=-1) && (pos_red_==-1) && (pos_nx_==-1))      mode=POSITION;
    else if((pos_x_ !=-1) && (pos_red_!=-1) && (pos_nx_==-1)) mode=POSITION_COLOR;
    else if((pos_x_ !=-1) && (pos_red_==-1) && (pos_nx_!=-1)) mode=POSITION_NORMAL;
    else if((pos_x_ !=-1) && (pos_red_!=-1) && (pos_nx_!=-1)) mode=POSITION_NORMAL_COLOR;    
}

void PlyParsing::PrintHeader(string filepath)
{
    FILE* fp=fopen(filepath.c_str(),"rb");
    fseek(fp,0,0);
    fread((void*)(&char_header), sizeof(char_header), 1, fp);
    string str_tmp=char_header;
    length_of_header_=str_tmp.find("end_header")+10;
    str_tmp=str_tmp.substr(0,length_of_header_);
    cout<<str_tmp<<endl;
    fclose(fp);
}

void PlyParsing::ReadLines(string path_to_file, vector<int>& line_id, pcl::PointCloud<PointType>::Ptr cloud)
{
    cloud->points.resize(line_id.size());
    ReadInit(path_to_file);
    // #pragma omp parallel for
    for(int i=0;i<cloud->points.size();i++){        
        ReadLine(line_id[i],cloud,i);
    }
    fclose(fpr_);
}

// void PlyParsing::ReadLines(string path_to_file, vector<int>& line_id, pcl::PointCloud<PointType>& cloud)
// {
//     cloud.points.resize(line_id.size());
//     ReadInit(path_to_file);
//     // #pragma omp parallel for
//     for(int i=0;i<cloud.points.size();i++){        
//         ReadLine(line_id[i],cloud,i);
//     }
//     fclose(fpr_);
// }


/*
    itmp: the index of point that should be added in the point cloud
*/
void PlyParsing::ReadLine(long linenum, pcl::PointCloud<PointType>::Ptr cloud,int itmp)
{ 
    char* line_buf=new char[type_size_];
    long int start_cursor=0;

    // file pointer management    
    if(POSITION==mode){
        Position ptmp;          
        start_cursor=length_of_header_+type_size_*linenum+1;
        fseek(fpr_,start_cursor+pos_x_,0);
        fread((void*)(&ptmp), 12, 1, fpr_);
	   
        cloud->points[itmp].x=ptmp.x;
        cloud->points[itmp].y=ptmp.y;
        cloud->points[itmp].z=ptmp.z;        
    }
    else if(POSITION_COLOR==mode){
        PositionColor ptmp;
        start_cursor=length_of_header_+type_size_*linenum+1;
        fseek(fpr_,start_cursor+pos_x_,0);
        fread((void*)(&ptmp), 12, 1, fpr_);      
        
        start_cursor=length_of_header_+type_size_*linenum+1;
        fseek(fpr_,start_cursor+pos_red_,0);
        fread((void*)(&ptmp.r), 4, 1, fpr_);   

        cloud->points[itmp].x=ptmp.x;
        cloud->points[itmp].y=ptmp.y;
        cloud->points[itmp].z=ptmp.z;     
        cloud->points[itmp].r=ptmp.r;
        cloud->points[itmp].g=ptmp.g;
        cloud->points[itmp].b=ptmp.b;
    }
    else if(POSITION_NORMAL==mode){ // position normal        
        PositionNormal ptmp;          
        start_cursor=length_of_header_+ type_size_*linenum+1;
        fseek(fpr_,start_cursor+pos_x_,0);
        fread((void*)(&ptmp), 12, 1, fpr_);
        start_cursor=length_of_header_+ type_size_*linenum+1;
        fseek(fpr_,start_cursor+pos_nx_,0);
        fread((void*)(&ptmp.nx), 12, 1, fpr_);      

        cloud->points[itmp].x=ptmp.x;
        cloud->points[itmp].y=ptmp.y;
        cloud->points[itmp].z=ptmp.z;
        cloud->points[itmp].normal_x=ptmp.x;
        cloud->points[itmp].normal_y=ptmp.y;
        cloud->points[itmp].normal_z=ptmp.z;
    }
    else if(POSITION_NORMAL_COLOR==mode){
        PositionNormalColor ptmp;          
        start_cursor=length_of_header_+ type_size_*linenum+1;
        fseek(fpr_,start_cursor+pos_x_,0);
        fread((void*)(&ptmp), 12, 1, fpr_);      
        start_cursor=length_of_header_+ type_size_*linenum+1;
        fseek(fpr_,start_cursor+pos_red_,0);
        fread((void*)(&ptmp.r), 4, 1, fpr_);      
        start_cursor=length_of_header_+ type_size_*linenum+1;
        fseek(fpr_,start_cursor+pos_nx_,0);
        fread((void*)(&ptmp.nx), 12, 1, fpr_);      
        
        cloud->points[itmp].x=ptmp.x;
        cloud->points[itmp].y=ptmp.y;
        cloud->points[itmp].z=ptmp.z;
        cloud->points[itmp].x=ptmp.r;
        cloud->points[itmp].y=ptmp.g;
        cloud->points[itmp].z=ptmp.b;  
        cloud->points[itmp].normal_x=ptmp.x;
        cloud->points[itmp].normal_y=ptmp.y;
        cloud->points[itmp].normal_z=ptmp.z;       
    }   
    delete line_buf;     
}

void PlyParsing::ReadLine(long linenum, PointType& pts)
{
    if(POSITION==mode){
        Position ptmp;          
        long start_cursor=length_of_header_+type_size_*linenum+1;
        fseek(fpr_,start_cursor+pos_x_,0);
        fread((void*)(&ptmp.x), 12, 1, fpr_);      	   
        pts.x=ptmp.x;
        pts.y=ptmp.y;
        pts.z=ptmp.z;        
    }
     else if(POSITION_COLOR==mode){
        PositionColor ptmp;
        long int start_cursor=length_of_header_+type_size_*linenum+1;
        fseek(fpr_,start_cursor+pos_x_,0);
        fread((void*)(&ptmp.x), 12, 1, fpr_);  

        fseek(fpr_,start_cursor+pos_red_,0);
        fread((void*)(&ptmp.r), 12, 1, fpr_);      
	   
        pts.x=ptmp.x;
        pts.y=ptmp.y;
        pts.z=ptmp.z;  
        pts.r=ptmp.r;
        pts.g=ptmp.g;
        pts.b=ptmp.b;
    }
    else if(POSITION_NORMAL==mode){ // position normal        
        PositionNormal ptmp;          
        long start_cursor=length_of_header_+type_size_*linenum+1;
        fseek(fpr_,start_cursor+pos_x_,0);
        fread((void*)(&ptmp.x), 12, 1, fpr_);  
        fseek(fpr_,start_cursor+pos_nx_,0);
        fread((void*)(&ptmp.nx), 12, 1, fpr_);      
        pts.x=ptmp.x;
        pts.y=ptmp.y;
        pts.z=ptmp.z;
        pts.normal_x=ptmp.nx;
        pts.normal_y=ptmp.ny;
        pts.normal_z=ptmp.nz;
    }
    else if(POSITION_NORMAL_COLOR==mode){
        PositionNormalColor ptmp;         
        long start_cursor=length_of_header_+type_size_*linenum+1;
        fseek(fpr_,start_cursor+pos_x_,0);
        fread((void*)(&ptmp.x), 12, 1, fpr_);

        fseek(fpr_,start_cursor+pos_nx_,0);
        fread((void*)(&ptmp.nx), 12, 1, fpr_);

        fseek(fpr_,start_cursor+pos_red_,0);
        fread((void*)(&ptmp.r), 4, 1, fpr_);
        pts.x=ptmp.x;
        pts.y=ptmp.y;
        pts.z=ptmp.z;
        pts.r=ptmp.r;
        pts.g=ptmp.g;
        pts.b=ptmp.b;
        pts.normal_x=ptmp.nx;
        pts.normal_y=ptmp.ny;
        pts.normal_z=ptmp.nz;        
    } 
}
void PlyParsing::ReadClose()
{
    if(fpr_!=NULL)
        fclose(fpr_);
}
void PlyParsing::LoadPly(double num_or_ratio, pcl::PointCloud<PointType>::Ptr cloud_out)
{
    /* random sampling */
    set<int> indices_of_samples;

    cout<<"number of vertex:"<<vertex_num_<<endl;
    #if TEST_RUNTIME_DOWNSAMPLING
    double itime, ftime, exec_time;
    itime = omp_get_wtime();
    #endif

    // determine number of ratio
    int LEN;
    if(num_or_ratio>=1){
        LEN=(int)num_or_ratio;
    }
    else if(num_or_ratio<1 && num_or_ratio>0){
        LEN=(int)vertex_num_*num_or_ratio;
    }
    else{
        cout<<"LoadPly parameter error!"<<endl;
    }
    
    while (indices_of_samples.size() < LEN){
         indices_of_samples.insert(rand() % vertex_num_);
    }        
    vector<int> v;
    v.assign(indices_of_samples.begin(),indices_of_samples.end());    
    cloud_out->points.resize(v.size());


    ReadInit(load_path_);
    // #pragma omp parallel for
    for(int i=0;i<v.size();i++){        
        ReadLine(v[i],cloud_out,i);        
    }

    #if TEST_RUNTIME_DOWNSAMPLING
    ftime = omp_get_wtime();
    exec_time = ftime - itime;
    cout<<"Time taken is "<< exec_time<<endl;
    #endif
}

void PlyParsing::WriteInit(string write_path, int mode, int num)
{
    write_path_=write_path;
    write_mode_=mode;
    PlyHeadAssembly pha;
    pha.Init(write_path,num,mode);
    fpw_=fopen(write_path_.c_str(),"a+");
}

void PlyParsing::AppendLine(PointType& pt)
{
    if(POSITION==write_mode_){
        Position ptmp;
        ptmp.x=pt.x;
        ptmp.y=pt.y;
        ptmp.z=pt.z;
        fwrite(&ptmp,SIZE_POSITION,1,fpw_);
    }
    else if(POSITION_COLOR==write_mode_){
        PositionColor ptmp;
        ptmp.x=pt.x;
        ptmp.y=pt.y;
        ptmp.z=pt.z;
        ptmp.r=pt.r;
        ptmp.g=pt.g;
        ptmp.b=pt.b;
        fwrite(&ptmp,SIZE_POSITION_COLOR,1,fpw_);
    }
    else if(POSITION_NORMAL==write_mode_){
        PositionNormal ptmp;
        ptmp.x=pt.x;
        ptmp.y=pt.y;
        ptmp.z=pt.z;
        ptmp.nx=pt.normal_x;
        ptmp.ny=pt.normal_y;
        ptmp.nz=pt.normal_z;
        fwrite(&ptmp,SIZE_POSITION_NORMAL,1,fpw_);
    }
    else if(POSITION_NORMAL_COLOR==write_mode_){
        PositionNormalColor ptmp;
        ptmp.x=pt.x;
        ptmp.y=pt.y;
        ptmp.z=pt.z;
        ptmp.r=pt.r;
        ptmp.g=pt.g;
        ptmp.b=pt.b;
        ptmp.nx=pt.normal_x;
        ptmp.ny=pt.normal_y;
        ptmp.nz=pt.normal_z;
        fwrite(&ptmp,SIZE_POSITION_NORMAL_COLOR,1,fpw_);
    }
}
void PlyParsing::WriteClose()
{
    if(fpw_!=NULL)
        fclose(fpw_);
}