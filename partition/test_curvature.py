import solve_curvature.build.solve_curvature as sc
import numpy as np 
import open3d as o3d 
from sklearn.neighbors import NearestNeighbors


ply=o3d.io.read_point_cloud("/home/llg/dataset_paper/4_Mimosa.ply")
ply_xyz=np.asarray(ply.points)
ply_rgb=np.asarray(ply.colors)

K=30
nn = NearestNeighbors(n_neighbors=K+1, algorithm='kd_tree').fit(ply_xyz)
distances, neighbors = nn.kneighbors(ply_xyz)
neighbors = neighbors[:, 1:]
distances = distances[:, 1:]

curvature=sc.get_curvature(ply_xyz,neighbors)
print(curvature.shape)