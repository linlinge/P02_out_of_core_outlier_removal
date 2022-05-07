import hdbscan
from sklearn.datasets import make_blobs
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt


cloud = o3d.io.read_point_cloud("/home/llg/dataset_paper/camp001_l3.ply")
cloud_xyz=np.array(cloud.points)

clusterer = hdbscan.HDBSCAN(min_cluster_size=10,core_dist_n_jobs=16)
cluster_labels = clusterer.fit_predict(cloud_xyz)
print(np.unique(cluster_labels) )

max_label = cluster_labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(cluster_labels / (max_label if max_label > 0 else 1))
colors[cluster_labels < 0] = 0
cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([cloud])