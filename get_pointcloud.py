import time
import numpy as np
import open3d as o3d
import pyrealsense2 as rs

# 获取耗时 0.016852140426635742
# 对齐耗时 0.01042938232421875
# 处理耗时 0.010200977325439453
# 总耗时 0.037482500076293945

# 配置相机
points = rs.points()
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(
    rs.stream.depth, 640, 480, rs.format.z16, 30
)
config.enable_stream(
    rs.stream.color, 640, 480, rs.format.bgr8, 30
)
profile = pipeline.start(config)
# 指定对齐对象
align_to = rs.stream.color
align = rs.align(align_to)
try:
    for frame_id in range(20):
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

    t0 = time.time()

    frames = pipeline.wait_for_frames()
    print('获取耗时', time.time() - t0)
    t1 = time.time()
    aligned_frames = align.process(frames)
    print('对齐耗时', time.time() - t1)
    # 这里开始将realsense的数据转换为open3d的数据结构
    # 相机参数
    profile = aligned_frames.get_profile()
    intrinsics = profile.as_video_stream_profile().get_intrinsics()
    # 转换为open3d中的相机参数
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
        intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
    # 转化数据帧为图像
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    # 转化为open3d中的图像
    t2 = time.time()
    img_depth = o3d.geometry.Image(depth_image)
    img_color = o3d.geometry.Image(color_image)
    # 从彩色图、深度图创建RGBD
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth)
    # 创建pcd
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
    print('处理耗时', time.time() - t2)
    print('总耗时', time.time() - t0)
    # 图像上下翻转
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    # 可视化
    o3d.visualization.draw_geometries([pcd])
    o3d.io.write_point_cloud('pcd/cup0226.pcd', pcd)

finally:
    pipeline.stop()
    print('done')
