#include "global.h"
int pid;
string path_to_ht="ht_thresh.txt";
void PrintVector3f(Eigen::Vector3f& dat,string seperator)
{
    cout<<dat[0]<<seperator<<dat[1]<<seperator<<dat[2]<<endl;
}