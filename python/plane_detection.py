import open3d as o3d 
import Bsc 
import numpy as np
import math
import  os
# import matplotlib.pyplot as plt

path_to_load="/home/llg/dataset_upholstery/04_Segmentation/raw_patch"
fs=Bsc.list_all_files(path_to_load)
score=[]
files=[]

for i in range(len(fs)):    
    cloud=o3d.io.read_point_cloud(fs[i])
    cloud_xyz=np.asarray(cloud.points)    
    c=np.cov(cloud_xyz.transpose())
    rk=np.linalg.matrix_rank(c)
    if rk==3:
        eval,evec=np.linalg.eig(c)
        w=np.argsort(eval)
        lambda0=eval[w[0]]
        lambda1=eval[w[1]]
        lambda2=eval[w[2]]            
        fname=fs[i].split("/")[-1]  
        id=int(fname.split(".")[0])
        score.append(np.array([id,lambda0,lambda1,lambda2]))
        files.append(fs[i])

score=np.asarray(score)
files=np.asarray(files)
reorder=np.argsort(score[:,0])
score=score[reorder]
files=files[reorder]

os.system("rm data.csv")
f=open("data.csv","a+")
for i in range(len(score)):
    if score[i,1]<0.001 and score[i,2]>0.001 and score[i,3]>0.01:
        f.write("%d,%f,%f,%f\n" % (score[i,0],score[i,1],score[i,2],score[i,3]))    
        exe_str='cp -rf '+files[i]+" wokaka/"+files[i].split("/")[-1]
        print(exe_str)
        os.system(exe_str)
f.close()