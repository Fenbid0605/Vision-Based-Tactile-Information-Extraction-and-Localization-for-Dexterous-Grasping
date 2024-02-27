import open3d as o3d

# Load a .pcd file
pcd = o3d.io.read_point_cloud("D:/0226/0.IROSdianyun/0.B.IROS/pcd/initial_cup0226_1(1).pcd")

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])