import Bsc 
path="/home/llg/dataset_paper/Stanford3dDataset_v1.2_Aligned_Version/Area_6"
fs=Bsc.list_all_files(path)
fp2=open(path+"/Area_6.xyz","a+")
count=0
for f in fs:
    fname=f.split("/")[-1]
    farther_folder=f.split("/")[-2]
    # print(fname)
    if fname.find(".txt")!=-1:
        fname_no_suffix=fname.split(".")[0]
        if fname_no_suffix==farther_folder:
            fp1=open(f)            
            lines=fp1.readlines()
            fp2.writelines(lines)
            fp1.close()
            count=count+1
            print(count)
fp2.close()