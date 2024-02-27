import open3d as o3d  # 需要pip安装open3d库
import os
import numpy as np
from scipy.spatial import KDTree

"""
总共需要自定义以下几个变量的值，要想得到想要的效果，需要对以下变量进行调参
    threshold_y,
    threshold_z,
    leaf_size,
    kdtree_radius,
    threshold_normal_y,
    threshold_color_var
"""
base_dir = "pcd_2"
file_path = os.path.join(base_dir, "beverage0226_1.pcd")  # 点云存放路径
original_pcd: o3d.geometry.PointCloud = o3d.io.read_point_cloud(file_path)  # 读取点云
xmin, ymin, zmin = original_pcd.get_min_bound()
translate_pcd: o3d.geometry.PointCloud = original_pcd.translate(
    (-xmin, -ymin, -zmin))  # 获得点云坐标的最小值，将点云平移，使得点云坐标的最小值恰好与原点重合
translate_pcd_points = np.asarray(translate_pcd.points)  # 获取点云坐标
# 以下4行代码用于输出点云在Y方向上的范围（具体原理见论文）
# y = points[:, 1]
# y = sorted(y)
# for i in y:
#     print(i)
threshold_y = 0.66  # 使用二分法确定Y方向上的阈值（具体原理见论文）
selected_points_id = np.argwhere(
    translate_pcd_points[:, 1] >= threshold_y
).reshape(-1)  # 返回满足Y坐标大于threshold_y的点的int型索引
y_filtered_pcd: o3d.geometry.PointCloud = translate_pcd.select_by_index(selected_points_id)  # 根据上面的索引过滤点云，相当于直通滤波
y_filtered_points = np.asarray(y_filtered_pcd.points)
threshold_z = 0.32  # 使用二分法确定Z方向上的阈值（具体原理见论文）
selected_points_id = np.argwhere(
    y_filtered_points[:, 2] >= threshold_z
)  # 返回满足Z坐标大于threshold_z的点的int型索引
final_filtered_pcd: o3d.geometry.PointCloud = y_filtered_pcd.select_by_index(selected_points_id)  # 根据上面的索引过滤点云，相当于直通滤波


# 经过两次直通滤波后，点云的形态良好，不存在离群点和背景点，且点云个数不多，可以不使用统计滤波或半径滤波去除离群点
# 这里，可以视点云的形貌决定是否使用统计滤波或半径滤波，目的是既要剔除离群点，又要尽可能地保留点云信息
# nb_neighbors = 10
# std_ratio = 0.1
# pcd4, idx = pcd3.remove_statistical_outlier(nb_neighbors, std_ratio)  # 统计滤波


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


final_filtered_points = np.asarray(final_filtered_pcd.points)  # 获得点云的三维坐标
final_filtered_colors = np.asarray(final_filtered_pcd.colors)  # 获得点云的颜色值（经过分析发现R=G=B，则为灰度值）

leaf_size = 32  # KD树搜索的最小数量
kdtree_radius = 0.05  # 设置邻域半径
kdtree = KDTree(final_filtered_points, leafsize=leaf_size)  # 构建KDTree
neighbor_idx_list = kdtree.query_ball_point(final_filtered_points, kdtree_radius)  # 得到每个点的邻近索引
point_normals = []  # 法向量列表
texture_feature_points = []  # 特征点列表（能够表征纹理的点）
non_texture_feature_points = []  # 非特征点列表
texture_feature_colors = []  # 特征点的RGB值列表
non_texture_feature_colors = []  # 非特征点的RGB值列表
threshold_normal_y = 0.15  # 法向量的Y分量的阈值
threshold_color_var = 0.0015  # 颜色值（灰度值）方差的阈值
# 遍历每个点的邻近索引
for i in range(len(neighbor_idx_list)):
    neighbor_idx = neighbor_idx_list[i]  # 得到第i个点的邻近点索引，邻近点包括自己
    neighbor_points = final_filtered_points[neighbor_idx]  # 得到邻近点的三维坐标
    neighbor_colors = final_filtered_colors[neighbor_idx]  # 得到邻近点的颜色值
    eigen_values, eigen_vectors = PCA(neighbor_points)  # 对邻近点采用主成分分析法（PCA）得到特征值和特征向量
    normal = eigen_vectors[:, 2]  # 最小特征值对应着法向量
    if normal[1] > threshold_normal_y and np.std(neighbor_colors[:, 0]) ** 2 > threshold_color_var:
        # 纹理特征点的判定条件：法向量的Y分量大于threshold_normal_y，且邻近点的颜色方差大于threshold_color_var
        texture_feature_points.append(final_filtered_points[i])
        texture_feature_colors.append(final_filtered_colors[i])
    else:
        non_texture_feature_points.append(final_filtered_points[i])
        non_texture_feature_colors.append(final_filtered_colors[i])
    point_normals.append(normal)

point_normals = np.array(point_normals, dtype=np.float64)
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
coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
o3d.visualization.draw_geometries([texture_pcd, non_texture_pcd, coord], point_show_normal=False)
