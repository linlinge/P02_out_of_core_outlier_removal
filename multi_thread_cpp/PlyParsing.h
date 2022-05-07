#pragma once
#include "PCLExtend.h"
#include "StringExtend.h"
#include <iostream>
using namespace std;
#define SIZE_POSITION 12
#define SIZE_POSITION_COLOR 15
#define SIZE_POSITION_NORMAL 24
#define SIZE_POSITION_NORMAL_COLOR 27


struct Position
{
    float x,y,z;    
    Position(){}
    Position(PointType& pt){
        x=pt.x;
        y=pt.y;
        z=pt.z;
    }
};
struct PositionColor{
    float x,y,z;
    unsigned char r,g,b;
};
struct PositionNormal{
    float x,y,z;
    float nx,ny,nz;
};
struct PositionNormalColor{
    float x,y,z;
    float nx,ny,nz;
    unsigned char r,g,b;
};


enum MODE{POSITION,POSITION_COLOR,POSITION_NORMAL,POSITION_NORMAL_COLOR};
class PlyHeadAssembly
{
    public:
        string head[3],tail[3];
        float x_,y_,z_,nx_,ny_,nz_;
        unsigned char r_,g_,b_,alpha_;
        string header_;
        int n_;
        void Init(string save_path,int num, int mode=POSITION){
            head[0]="ply\n";
            head[1]="format binary_little_endian 1.0\n";
            head[2]="comment VCGLIB generated\n";            
            tail[0]="element face 0\n";
            tail[1]="property list uchar int vertex_indices\n";
            tail[2]="end_header\n";
            if(POSITION==mode){
                header_+=head[0]+head[1]+head[2];
                header_+="element vertex "+to_string(num)+"\n";
                header_+="property float x\n";
                header_+="property float y\n";
                header_+="property float z\n";                
            }
            else if(POSITION_COLOR==mode){
                header_+=head[0]+head[1]+head[2];
                header_+="element vertex "+to_string(num)+"\n";
                header_+="property float x\n";
                header_+="property float y\n";
                header_+="property float z\n";
                header_+="property uchar red\n";
                header_+="property uchar green\n";
                header_+="property uchar blue\n";                              
            }
            else if(POSITION_NORMAL==mode){
                header_+=head[0]+head[1]+head[2];
                 header_+="element vertex "+to_string(num)+"\n";
                header_+="property float x\n";
                header_+="property float y\n";
                header_+="property float z\n";
                header_+="property float nx\n";
                header_+="property float ny\n";
                header_+="property float nz\n";                              
            }
            else if(POSITION_NORMAL_COLOR==mode){
                header_+=head[0]+head[1]+head[2];
                header_+="element vertex "+to_string(num)+"\n";
                header_+="property float x\n";
                header_+="property float y\n";
                header_+="property float z\n";
                header_+="property float nx\n";
                header_+="property float ny\n";
                header_+="property float nz\n"; 
                header_+="property uchar red\n";
                header_+="property uchar green\n";
                header_+="property uchar blue\n";                
            }
            header_+=tail[0]+tail[1]+tail[2];
            n_=header_.size();
            FILE* fp=fopen(save_path.c_str(),"wb");
            fputs(header_.c_str(),fp);
            fclose(fp);
        }
};

class PlyParsing
{
    private:       
        /* header parsing paramters */
        int length_of_header_;
        char char_header[1000];
        int mode;       
        string load_path_;

    public:           
        long vertex_num_; 
        int pos_x_;
        int pos_red_;
        int pos_nx_;
        int type_size_;
        /* parsing */
        int ReadVertexNumber(string path);

        /* read */
        // Note: file hander should be close when finishing using
        FILE* fpr_;
        void ReadInit(string load_path);
        void ReadLine(long linenum, pcl::PointCloud<PointType>::Ptr cloud, int itmp);
        void ReadLine(long linenum, PointType& pts);
        void ReadLines(string path_to_file, vector<int>& line_id, pcl::PointCloud<PointType>::Ptr cloud);
        void ReadClose();
        void PrintHeader(string filepath); // Do not require to call Init firstly
        void LoadPly(double num_or_ratio, pcl::PointCloud<PointType>::Ptr cloud_out);

        // /* search */
        // int Search(PointType& pt);

        /* write */
        string write_path_;
        int write_mode_;
        FILE* fpw_;
        void WriteInit(string write_path, int mode, int num);
        void AppendLine(PointType& pt);
        void WriteClose();
};