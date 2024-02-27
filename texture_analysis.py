import open3d as o3d  # 需要pip安装open3d库
import os
import numpy as np
from scipy.spatial import KDTree

"""
总共需要自定义以下几个变量的值，要想得到想要的效果，需要对以下变量进行调参
    leaf_size,
    kdtree_radius,
    threshold_normal_y,
    threshold_color_var
"""
base_dir = "pcd_2"
file_path = os.path.join(base_dir, "vc0226_1.pcd")  # 点云存放路径
initial_pcd: o3d.geometry.PointCloud = o3d.io.read_point_cloud(file_path)  # 读取点云
# 基于PCA求解特征值和特征向量
def PCA(data):
    mean_data = np.mean(data, axis=0)  # normalize 归一化
    normal_data = data - mean_data
    H = np.dot(normal_data.T, normal_data)  # 计算对称的协方差矩阵
    eigen_vectors, eigen_values, _ = np.linalg.svd(H)  # SVD奇异值分解，得到H矩阵的特征值和特征向量
    sort = eigen_values.argsort()[::-1]  # 建立从大到小的索引
    eigen_values = eigen_values[sort]
    eigen_vectors = eigen_vectors[:, sort]

    return eigen_values, eigen_vectors  # 返回特征值和特征向量


initial_points = np.asarray(initial_pcd.points)  # 获得点云的三维坐标
initial_colors = np.asarray(initial_pcd.colors)  # 获得点云的颜色值（经过分析发现R=G=B，则为灰度值）

leaf_size = 32  # KD树搜索的最小数量
kdtree_radius = 0.05  # 设置邻域半径
kdtree = KDTree(initial_points, leafsize=leaf_size)  # 构建KDTree
neighbor_idx_list = kdtree.query_ball_point(initial_points, kdtree_radius)  # 得到每个点的邻近索引
point_normals = []  # 法向量列表
texture_feature_points = []  # 特征点列表（能够表征纹理的点）
non_texture_feature_points = []  # 非特征点列表
texture_feature_colors = []  # 特征点的RGB值列表
non_texture_feature_colors = []  # 非特征点的RGB值列表
threshold_normal_y = 0.2  # 法向量的Y分量的阈值
threshold_color_var = 0.002# 颜色值（灰度值）方差的阈值
# 遍历每个点的邻近索引
for i in range(len(neighbor_idx_list)):
    neighbor_idx = neighbor_idx_list[i]  # 得到第i个点的邻近点索引，邻近点包括自己
    neighbor_points = initial_points[neighbor_idx]  # 得到邻近点的三维坐标
    neighbor_colors = initial_colors[neighbor_idx]  # 得到邻近点的颜色值
    eigen_values, eigen_vectors = PCA(neighbor_points)  # 对邻近点采用主成分分析法（PCA）得到特征值和特征向量
    normal = eigen_vectors[:, 2]  # 最小特征值对应着法向量
    if abs(normal[1]) >= threshold_normal_y and np.std(neighbor_colors[:, 0]) ** 2 >= threshold_color_var:
        # 纹理特征点的判定条件：法向量的Y分量大于threshold_normal_y，且邻近点的颜色方差大于threshold_color_var
        texture_feature_points.append(initial_points[i])
        texture_feature_colors.append(initial_colors[i])
    else:
        non_texture_feature_points.append(initial_points[i])
        non_texture_feature_colors.append(initial_colors[i])
    point_normals.append(normal)

point_normals = np.array(point_normals, dtype=np.float64)
initial_pcd.normals = o3d.utility.Vector3dVector(point_normals)
texture_feature_points = np.array(texture_feature_points, dtype=np.float64)
texture_feature_colors = np.array(texture_feature_colors, dtype=np.float64)
texture_pcd = o3d.geometry.PointCloud()  # 包含纹理特征的点云
texture_pcd.points = o3d.utility.Vector3dVector(texture_feature_points)
non_texture_pcd = o3d.geometry.PointCloud()  # 不包含纹理特征的点云
non_texture_pcd.points = o3d.utility.Vector3dVector(non_texture_feature_points)
non_texture_pcd.colors = o3d.utility.Vector3dVector(non_texture_feature_colors)
height_min = np.min(texture_feature_colors[:, 1])  # 纹理点云颜色最小值
height_max = np.max(texture_feature_colors[:, 1])  # 纹理点云颜色最大值
delta_c = np.abs(height_max - height_min) / 255
colors = np.zeros((texture_feature_colors.shape[0], 3))
for i in range(texture_feature_colors.shape[0]):
    color_n = (texture_feature_colors[i, 1] - height_min) / delta_c
    if color_n <= 255:
        colors[i, :] = [0, color_n / 255, 1]
    else:
        colors[i, :] = [color_n / 255, 0, 1]

texture_pcd.colors = o3d.utility.Vector3dVector(colors)  # 包含纹理特征的点云的颜色为渐变色
non_texture_pcd.paint_uniform_color([0, 1, 0])  # 不包含纹理特征的点云的颜色为绿色
# 可视化
o3d.visualization.draw_geometries([texture_pcd, non_texture_pcd], point_show_normal=False)
