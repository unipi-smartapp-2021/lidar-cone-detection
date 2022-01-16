import math

import open3d as o3d
import numpy as np
import pandas as pd

from src.lidar.utils import visualize_cartesian


def depth_to_pcd(depth_img):
    fovx = math.radians(110)
    fovy = math.radians(70)
    img_height = 720
    img_width = 1280
    fx = img_width / (2 * math.tan(fovx / 2))
    fy = img_height / (2 * math.tan(fovy / 2))
    cx = img_width / 2
    cy = img_height / 2
    cam_mat = o3d.camera.PinholeCameraIntrinsic(img_width, img_height, fx, fy, cx, cy)
    pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_img.depth, cam_mat)
    quaternion = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]] #rotation
    pcd.transform(quaternion)
    o3d.visualization.draw_geometries([pcd])
    raw_pcd = np.array(pcd.points)
    pcd_df = pd.DataFrame(raw_pcd, columns=["x", "y", "z"])  # x horizontal, y vertical, z distance
    print(pcd_df.describe())
    print(pcd_df['z'].value_counts())
    pcd_df = pcd_df[pcd_df['z'] > pcd_df['z'].min()]
    return pcd_df


def test_pointcloud_space():
    stereo_yolo_out = pd.read_csv("stereo/stereo_yolo_boxes.txt", header=None, sep=" ")
    stereo_yolo_out.columns = ["cx", "xmin", "xmax", "cy", "ymin", "ymax", "class"]

    colored = o3d.io.read_image("./stereo/rgb_camera.png")
    depth = o3d.io.read_image("./stereo/depth_camera.png")
    depth = np.array(depth)
    cones = np.zeros(shape=(0, 3))
    for index, row in stereo_yolo_out.iterrows():
        xmin = int(np.floor(row['xmin']))
        xmax = int(np.ceil(row['xmax']))
        ymin = int(np.floor(row['ymin']))
        ymax = int(np.ceil(row['ymax']))
        a = depth[ymin:ymax, xmin:xmax]
        depth2 = np.ones((720, 1280), dtype=np.uint8) * 255
        depth2[ymin:ymax, xmin:xmax] = a
        depth2 = o3d.geometry.Image(depth2.astype(np.uint8))
        rgbdimage = o3d.geometry.RGBDImage.create_from_color_and_depth(colored, depth)
        cone_pcd = depth_to_pcd(rgbdimage)
        cone_pcd = cone_pcd[cone_pcd['z'] == cone_pcd['z'].max()]
        # cone_pcd['class'] = row['class']
        print(cone_pcd['z'].value_counts())
        cones = np.vstack([cones, cone_pcd.values])
        print(cone_pcd)
        visualize_cartesian(cone_pcd['x','y','z'].values)

test_pointcloud_space()