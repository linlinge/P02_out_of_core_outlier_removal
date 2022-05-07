import os.path
import sys
import numpy as np
import argparse
from timeit import default_timer as timer
import matplotlib.pyplot as plt
sys.path.append("./cut-pursuit/build/src")
sys.path.append("./ply_c")
sys.path.append("./")
# sys.path.append("./cut-pursuit/build/src")
import libcp

# import libply_c
from graphs import *
from provider import *
import open3d as o3d 

# import solve_curvature.build.solve_curvature as sc
import time
import math


def write_componets(path_to_ht,comp):
    fout=open(path_to_ht,"w")
    for cluster in comp:
        wstr=""
        for e in cluster:
            wstr=wstr+" %d" % e
        wstr=wstr+"\n"
        fout.write(wstr)
    fout.close()

def divide_into_10(indices,dat):
    indices=np.asarray(indices)
    dat=np.asarray(dat)
    vmin=min(dat)
    vmax=max(dat)
    span=(vmax-vmin)/10
    rst=[[] for i in range(10)]
    rst_i=[[] for i in range(10)]
    for i in range(len(dat)):             
        span_i=math.floor((dat[i]-vmin)/span)
        if span_i==10:
            span_i=9
        rst[span_i].append(dat[i])
        rst_i[span_i].append(indices[i])
    
    return rst_i,rst


parser = argparse.ArgumentParser(description='massive')
parser.add_argument('--path_to_low',       default='/home/i9/experiment_massive/Area_2_noise_dwn_cleaned.ply',help="input")
parser.add_argument('--path_to_component', default='/home/i9/experiment_massive/Area_2_noise_dwn_cleaned_components.txt',help="output")
parser.add_argument('--path_to_color',     default='/home/i9/experiment_massive/Area_2_noise_dwn_cleaned_color.ply',help="output")
parser.add_argument('--k_nn_geof', default=45, type=int, help='number of neighbors for the geometric features')
parser.add_argument('--k_nn_adj', default=10, type=int, help='adjacency structure for the minimal partition')
parser.add_argument('--lambda_edge_weight', default=0.5, type=float, help='parameter determine the edge weight for minimal part.')
parser.add_argument('--reg_strength', default=0.1, type=float, help='regularization strength for the minimal partition')
parser.add_argument('--d_se_max', default=0, type=float, help='max length of super edges')
parser.add_argument('--voxel_width', default=0.03, type=float, help='voxel size when subsampling (in m)')
parser.add_argument('--ver_batch', default=0, type=int, help='Batch size for reading large files, 0 do disable batch loading')
parser.add_argument('--overwrite', default=0, type=int, help='Wether to read existing files or overwrite them')
args = parser.parse_args()


Tstart=time.time()

#--- build the geometric feature file h5 file ---
cloud=o3d.io.read_point_cloud(args.path_to_low)
cloud_xyz=np.asarray(cloud.points)
cloud_nrm=np.asarray(cloud.normals)
cloud_rgb=np.ones((cloud_xyz.shape[0],3))

# feature
K=30
nn = NearestNeighbors(n_neighbors=K+1, algorithm='kd_tree').fit(cloud_xyz)
distances, neighbors = nn.kneighbors(cloud_xyz)
neighbors = neighbors[:, 1:]
distances = distances[:, 1:]
# curvature=sc.get_curvature(cloud_xyz,neighbors)

#--- compute 10 nn graph -------
graph_nn, target_fea = compute_graph_nn_2(cloud_xyz, args.k_nn_adj, args.k_nn_geof)

#--- compute geometric features -------
# geof = libply_c.compute_geof(cloud_xyz, target_fea, args.k_nn_geof).astype('float32')
geof=cloud_nrm
del target_fea

#--compute the partition------
print("    computing the superpoint graph...")
#--- build the spg h5 file --
start = timer()
features = np.hstack((geof, cloud_rgb/255.)).astype('float32')#add rgb as a feature for partitioning
# features = np.hstack((geof, curvature)).astype('float32')#add rgb as a feature for partitioning
# features[:,3] = 2. * features[:,3] #increase importance of verticality (heuristic)

# features=cloud_nrm
graph_nn["edge_weight"] = np.array(1. / ( args.lambda_edge_weight + graph_nn["distances"] / np.mean(graph_nn["distances"])), dtype = 'float32')
print("        minimal partition...")
components, in_component = libcp.cutpursuit(features, graph_nn["source"], graph_nn["target"], graph_nn["edge_weight"], args.reg_strength)


# 
for i in range(len(components)):    
    if len(components[i]) > 200000:
        datx=cloud_xyz[components[i],0]
        daty=cloud_xyz[components[i],1]
        datz=cloud_xyz[components[i],2]
        datx_span=np.max(datx)-np.min(datx)
        daty_span=np.max(daty)-np.min(daty)
        datz_span=np.max(datz)-np.min(datz)
        if datx_span> daty_span and datx_span > daty_span:
            tmp_i,tmp=divide_into_10(components[i],datx)
        elif daty_span> datx_span and daty_span > datz_span:
            tmp_i,tmp=divide_into_10(components[i],daty)
        elif datz_span> datx_span and datz_span > daty_span:
            tmp_i,tmp=divide_into_10(components[i],datz)
        components[i]=tmp_i[0]
        for i in [1,2,3,4,5,6,7,8,9]:
           if len(tmp_i[i])!=0:
                components.append(tmp_i[i])



components = np.array(components, dtype = 'object')

# assign color to each cluster
for c in components:
    color_tmp=np.array([[random.randint(0,255)/255,random.randint(0,255)/255,random.randint(0,255)/255]])
    cloud_rgb[c,:]=np.matlib.repmat(color_tmp,len(c),1)


write_componets(args.path_to_component,components)

# cloud_colors=np.asarray(cloud_colors,dtype=int)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(cloud_xyz)
pcd.colors = o3d.utility.Vector3dVector(cloud_rgb)
pcd.normals = o3d.utility.Vector3dVector(cloud_nrm)
o3d.io.write_point_cloud(args.path_to_color, pcd)

Tend=time.time()
print(Tend-Tstart)