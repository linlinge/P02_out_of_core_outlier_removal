import numpy as np 
import open3d as o3d 
def get_color(dat):
    dmin=np.min(dat)
    dmax=np.max(dat)    
    n=4.0
    step=(dmax-dmin)/n
    c=dat-dmin
    colors=np.zeros([len(dat),3])
    for i in range(len(c)):
        if c[i]<step: # blue        
            colors[i,0]=0
            colors[i,1]=c[i]/step*255
            colors[i,2]=255
        elif c[i]<2*step:
            colors[i,0]=0
            colors[i,1]=255
            colors[i,2]=255-(c[i]-step)/step*255
        elif c[i]<3*step:
            colors[i,0]=(c[i]-2*step)/step*255
            colors[i,1]=255
            colors[i,2]=0        
        else:    # red        
            colors[i,0]=255
            colors[i,1]=255-(c[i]-3*step)/step*255
            colors[i,2]=0
    
    colors=colors/256
    return colors

cloud=o3d.io.read_point_cloud("/home/llg/dataset_paper/new/1508.ply")
cloud_xyz=np.asarray(cloud.points)
dat=np.load("/home/llg/dataset_paper/new/1.npy")
dat_colors=get_color(dat)

# print(dat_colors.shape)
# print(cloud_xyz.shape)
# dat_colors=np.ones([len(dat_colors),3])
# cloud.colors=o3d.utility.Vector3dVector(dat_colors)
# o3d.io.write_point_cloud("/home/llg/dataset_paper/new/2.ply",cloud,write_ascii=True)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(cloud_xyz)
pcd.colors=o3d.utility.Vector3dVector(dat_colors)
o3d.io.write_point_cloud("/home/llg/dataset_paper/new/2.ply", pcd)
# o3d.visualization.draw_geometries([cloud])