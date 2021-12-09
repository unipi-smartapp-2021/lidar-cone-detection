import numpy as np
import pandas as pd

from utils import min_max_scale

"""def depth_to_pcd(depth_img):
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
    raw_pcd = np.array(pcd.points)
    pcd_df = pd.DataFrame(raw_pcd, columns=["x", "y", "z"])  # x horizontal, y vertical, z distance
    print(pcd_df.describe())
    print(pcd_df['z'].value_counts())
    pcd_df = pcd_df[pcd_df['z'] > pcd_df['z'].min()]
    return pcd_df
    # o3d.visualization.draw_geometries([pcd])"""

"""def test_pointcloud_space():
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
        rgbdimage = o3d.geometry.RGBDImage.create_from_color_and_depth(colored, depth2)
        cone_pcd = depth_to_pcd(rgbdimage)
        cone_pcd = cone_pcd[cone_pcd['z'] == cone_pcd['z'].max()]
        # cone_pcd['class'] = row['class']
        print(cone_pcd['z'].value_counts())
        cones = np.vstack([cones, cone_pcd.values])"""


def output_fusion(lidar_output, stereo_yolo_out):
    """
    Output fusion assign on each cone from the lidar output the corresponding label.
    :param lidar_output: the output from postprocessing on the lidarmodel
    :param stereo_yolo_out: the yolo numpy array with ['xmin', 'ymin', 'xmax', 'ymax', 'conf', 'class']
    :return:
    """
    lidar_output = pd.DataFrame(lidar_output, columns=['x', 'y', 'z'])
    lidar_output['ys'], _ = min_max_scale((0, 1280), lidar_output['y'].values, max=15, min=-15)
    stereo_yolo_out = pd.DataFrame(stereo_yolo_out, columns=['xmin', 'ymin', 'xmax', 'ymax', 'conf', 'class'])
    stereo_yolo_out['xcenter'] = stereo_yolo_out['xmin'] + (stereo_yolo_out['xmax'] - stereo_yolo_out['xmin']) / 2
    stereo_yolo_out['ycenter'] = stereo_yolo_out['ymin'] + (stereo_yolo_out['ymax'] - stereo_yolo_out['ymin']) / 2
    stereo_yolo_out = stereo_yolo_out.sort_values(by=['ycenter'], ascending=False)
    stereo_yolo_out = stereo_yolo_out[:2]
    print(stereo_yolo_out.values)
    print(lidar_output)
    lidar_output['c'] = -1
    for i, row in lidar_output.iterrows():
        stereo_yolo_out['distance'] = np.abs(stereo_yolo_out['xcenter'] - row['ys'])
        classe = stereo_yolo_out[stereo_yolo_out['distance'] == stereo_yolo_out['distance'].min()]['class']
        lidar_output.at[i, 'c'] = classe
    lidar_output = lidar_output[['x', 'y', 'z', 'c']]
    return lidar_output.values



""""
# Example
lidar_output = np.array([[-3.313545776951697386e-01, 1.468945280198127978e+00, 1.660011298954486847e-01],
                [-2.138221520516607466e-01, -1.452068110307057625e+00, 7.976718619465827942e-02]], dtype=np.float16)
stereo_yolo_out = np.array([[4.717500000000000000e+02, 4.142500000000000000e+02, 4.967500000000000000e+02, 4.472500000000000000e+02, 8.720703125000000000e-01, 2.000000000000000000e+00],
[7.460000000000000000e+02, 4.125000000000000000e+02, 7.710000000000000000e+02, 4.465000000000000000e+02, 8.686523437500000000e-01, 3.000000000000000000e+00],
[6.810000000000000000e+02, 3.742500000000000000e+02, 7.000000000000000000e+02, 4.007500000000000000e+02, 8.339843750000000000e-01, 1.000000000000000000e+00],
[5.445000000000000000e+02, 3.772500000000000000e+02, 5.625000000000000000e+02, 4.017500000000000000e+02, 7.661132812500000000e-01, 1.000000000000000000e+00]], dtype=np.float16)
out = output_fusion(lidar_output, stereo_yolo_out)
print(out)
"""
