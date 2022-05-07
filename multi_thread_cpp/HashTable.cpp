#include <HashTable.h>
void HashTable::Resize(int n)
{
    n_=n;
    dat_.resize(n);
}
void HashTable::PushBack(int idx, int val)
{
    dat_[idx].push_back(val);
}
void HashTable::SaveBinary(string path)
{
    n_=dat_.size();
    // calculate offset    
    offset_len_.resize(n_);
    // 1-th element
    offset_len_[0].resize(2);
    offset_len_[0][0]=sizeof(int)*(2*n_+1);
    offset_len_[0][1]=dat_[0].size();

    // from 2 to end
    for(int i=1;i<n_;i++){        
        offset_len_[i].resize(2);
        offset_len_[i][0]=offset_len_[i-1][0]+sizeof(int)*dat_[i-1].size();
        offset_len_[i][1]=dat_[i].size();
    }

    if(path=="default"){
        FILE* fp=fopen("ht.txt","wb");
        // write size
        fwrite(&n_,sizeof(int),1,fp);
        // write each offset_len_ element
        for(int i=0;i<n_;i++){
            fwrite(offset_len_[i].data(),sizeof(int)*2,1,fp);
        }
        // write each data element
        for(int i=0;i<n_;i++){
            fwrite(dat_[i].data(),sizeof(int),dat_[i].size(),fp);
        }
        fclose(fp);
    }
    else{
         FILE* fp=fopen(path.c_str(),"wb");
        // write size
        fwrite(&n_,sizeof(int),1,fp);
        // write each offset_len_ element
        for(int i=0;i<n_;i++){
            fwrite(offset_len_[i].data(),sizeof(int)*2,1,fp);
        }
        // write each data element
        for(int i=0;i<n_;i++){
            fwrite(dat_[i].data(),sizeof(int),dat_[i].size(),fp);
        }
        fclose(fp);
    }
}

void HashTable::SaveAscii(string path)
{
   ofstream fout(path);
   for(int i=0;i<dat_.size();i++){
       for(int j=0;j<dat_[i].size()-1;j++){
           fout<<dat_[i][j]<<" ";
       }
       fout<<dat_[i][dat_[i].size()-1]<<endl;
   }
   fout.close();
}

// Read Section
void HashTable::Init(string path)
{
    path_=path;
    if(path.find(".txt")!=string::npos){
        FILE* fp=fopen(path.c_str(),"rb");
        fread(&n_,sizeof(int),1,fp);
        offset_len_.resize(n_);
        for(int i=0;i<n_;i++){
            offset_len_[i].resize(2);
            fread(offset_len_[i].data(),sizeof(int)*2,1,fp);
        }
        fclose(fp);
    }
    else{
        cout<<"Path error: please check the read path!"<<endl;
    }
}

void HashTable::ReadLine(int i,vector<int>& dat)
{   
    if(i>=0){
        FILE* fp=fopen(path_.c_str(),"rb");
        fseek(fp,offset_len_[i][0],0);
        dat_.resize(offset_len_[i][1]);
        fread(dat_.data(),sizeof(int),offset_len_[i][1],fp);        
        fclose(fp);
    }     
    else{
        cout<<"HashTable read line error!"<<endl;
    }
}

void HashTable::ReadLines(string path_to_ht, string mode)
{
    if("binary"==mode){
        Init(path_to_ht);
        dat_.resize(offset_len_.size());
        FILE* fp=fopen(path_.c_str(),"rb");
        for(int i=0;i<offset_len_.size();i++){
            fseek(fp,offset_len_[i][0],0);
            dat_[i].resize(offset_len_[i][1]);
            fread(dat_[i].data(),sizeof(int),offset_len_[i][1],fp);
        }
        fclose(fp);
    }
    else if("ascii"==mode){
        ifstream fin(path_to_ht);
        string line;
        m_=0;

        while(getline(fin, line)){            
            vector<int> dtmp;
            if(line!=""){
                vector<string> ss;
                StrSplit(line," ",ss);
                dtmp.resize(ss.size());
                m_+=ss.size();

                for(int i=0;i<ss.size();i++)
                    dtmp[i]=atoi(ss[i].c_str());
            }
            dat_.push_back(dtmp);
        }
        fin.close();
    }   
}

double HashTable::GetThresholdWithTukeyFence(double kIQR)
{
    vector<int> size_in;
    size_in.resize(dat_.size());
    for(int i=0;i<dat_.size();i++)
        size_in[i]=dat_[i].size();    
    
    double thresh=TukeyFence(size_in,kIQR);
    return thresh;
}

int HashTable::GetSize()
{
    return dat_.size();
}