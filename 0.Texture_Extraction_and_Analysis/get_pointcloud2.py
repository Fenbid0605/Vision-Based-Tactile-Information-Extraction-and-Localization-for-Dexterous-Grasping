import time
import numpy as np
import open3d as o3d
import pyrealsense2 as rs

def get_aligned_frames(pipeline, align):
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    return aligned_frames.get_depth_frame(), aligned_frames.get_color_frame()

def main():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    try:
        pipeline.start(config)
        align_to = rs.stream.color
        align = rs.align(align_to)

        # Skip some frames to allow auto-exposure to stabilize
        for _ in range(20):
            pipeline.wait_for_frames()

        start_time = time.time()
        depth_frame, color_frame = get_aligned_frames(pipeline, align)
        print('获取耗时', time.time() - start_time)

        start_time = time.time()
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        print('转换耗时', time.time() - start_time)

        start_time = time.time()
        img_depth = o3d.geometry.Image(depth_image)
        img_color = o3d.geometry.Image(color_image)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        print('处理耗时', time.time() - start_time)

        o3d.visualization.draw_geometries([pcd])
        o3d.io.write_point_cloud('cup0224.pcd', pcd)

    except Exception as e:
        print('Error:', e)
    finally:
        pipeline.stop()

if __name__ == '__main__':
    main()
