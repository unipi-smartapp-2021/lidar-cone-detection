import math

import numpy as np
import open3d as o3d
import pandas as pd


def depth_to_pcd(depth_path, rgb_path):
    fovx = math.radians(110)
    fovy = math.radians(70)
    img_height = 720
    img_width = 1280
    fx = img_width / (2 * math.tan(fovx / 2))
    fy = img_height / (2 * math.tan(fovy / 2))
    cx = img_width / 2
    cy = img_height / 2

    colored = o3d.io.read_image(depth_path)
    depth = o3d.io.read_image(rgb_path)
    rgbdimage = o3d.geometry.RGBDImage.create_from_color_and_depth(colored, depth, convert_rgb_to_intensity=False, depth_scale=1e3)

    cam_mat = o3d.camera.PinholeCameraIntrinsic(img_width, img_height, fx, fy, cx, cy)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbdimage, cam_mat)
    pcd.scale(1e5, center=(0,0,0))
    # quaternion = [[1, 0, 0, 0],
    #               [0, -1, 0, 0],
    #               [0, 0, -1, 0],
    #               [0, 0, 0, 1]] #rotation
    # pcd.transform(quaternion)
    R = pcd.get_rotation_matrix_from_xyz((np.pi,
                                          np.pi/2,
                                          np.pi/2))
    pcd.rotate(R, center=(0,0,0))
    raw_pcd = np.array(pcd.points)
    pcd_df = pd.DataFrame(raw_pcd, columns=["x", "y", "z"])  # x horizontal, y vertical, z distance
    print(pcd_df.describe())
    print(pcd_df['z'].value_counts())
    pcd_df = pcd_df[pcd_df['z'] > pcd_df['z'].min()]
    # return pcd_df
    o3d.visualization.draw_geometries([pcd])
    return pcd



pcd = depth_to_pcd("./stereo/rgb_camera.png","./stereo/depth_camera.png")
o3d.io.write_point_cloud("./stereo/converted_rgbd.pcd", pcd)