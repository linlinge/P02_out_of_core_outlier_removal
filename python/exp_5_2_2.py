import numpy as np 
import matplotlib.pyplot as plt 
import open3d as o3d 
from sklearn.neighbors import NearestNeighbors
from KDEpy import FFTKDE

def get_local_density(cloud_xyz,K=40):    
    nn = NearestNeighbors(n_neighbors=K+1, algorithm='kd_tree').fit(cloud_xyz)
    distances, neighbors = nn.kneighbors(cloud_xyz)
    # neighbors = neighbors[:, 1:]
    distances = distances[:, -1]
    density=1/distances    
    return density

cloud1=o3d.io.read_point_cloud("/home/llg/dataset_paper/exp_5_2_3/hzl_8000000.ply")
cloud2=o3d.io.read_point_cloud("/home/llg/dataset_paper/exp_5_2_3/hzl_7000000.ply")
cloud1_xyz=np.asarray(cloud1.points)
cloud2_xyz=np.asarray(cloud2.points)
density1=get_local_density(cloud1_xyz)
density2=get_local_density(cloud2_xyz)
x1, y1 = FFTKDE(kernel="gaussian", bw="silverman").fit(density1).evaluate()
x2, y2 = FFTKDE(kernel="gaussian", bw="silverman").fit(density2).evaluate()
plt.plot(x1, y1)
plt.plot(x2, y2)
plt.show()

