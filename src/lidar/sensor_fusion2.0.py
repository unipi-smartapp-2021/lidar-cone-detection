import math
import pickle

import cv2
import numpy as np
import open3d as o3d
import pandas as pd

from src.lidar.post_processing import post_processing_frompath


def depth_to_pcd(depth_path, rgb_path, labels_path):
    colored = cv2.imread(rgb_path, 1)
    depth = o3d.io.read_image(depth_path)
    colored = o3d.geometry.Image(cv2.cvtColor(np.asarray(colored), cv2.COLOR_BGR2RGB))
    boxes = pd.read_csv(labels_path, header=None, sep=" ")
    boxes.columns = ["xmin", "ymin", "xmax", "ymax", "prob", "class", "classname"]
    depth = np.asarray(depth)

    cones = np.zeros(shape=(0, 3))
    cone_pcds = []
    cone_centres = []
    for i, box in boxes.iterrows():
        xmin = int(np.floor(box['xmin']))
        xmax = int(np.ceil(box['xmax']))
        ymin = int(np.floor(box['ymin']))
        ymax = int(np.ceil(box['ymax']))

        a = depth[ymin:ymax, xmin:xmax]

        depth2 = np.ones((720, 1280), dtype=np.uint8) * 255
        depth2[ymin:ymax, xmin:xmax] = a

        depth2 = o3d.geometry.Image(depth2.astype(np.uint8))
        cone_pcd = create_colored_pcd(colored, depth2)

        cone_df = pd.DataFrame(np.array(cone_pcd.points), columns=["x", "y", "z"])
        # cone_colors = pd.DataFrame(np.array(cone_pcd.points), columns=["r", "g", "b"])

        # print(cone_df.describe())
        cone_df = cone_df[cone_df['z'] == cone_df['z'].min()]
        cones = np.vstack([cones, cone_df.values])

        cone_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(cone_df[["x", "y", "z"]].values))
        cone_pcd = transform_pcd_to_lidar_scale(cone_pcd)
        # print(np.asarray(cone_pcd.points)[:, 0].shape)
        pointarr = np.asarray(cone_pcd.points)
        cx = np.mean(pointarr[:, 0])
        cy = np.mean(pointarr[:, 1])
        cz = np.mean(pointarr[:, 2])
        cone_pcds.append(cone_pcd)
        cone_centres.append([cx, cy, cz])
        # visualize_cartesian(cone_pcd[["x", "y", "z"]].values)

    cone_centres = np.array(cone_centres)
    return cone_centres, boxes["classname"].values


def create_colored_pcd(colored, depth):
    cx, cy, depth_scale, fx, fy, img_height, img_width = transform_params()
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(colored, depth, convert_rgb_to_intensity=False)
    cam_mat = o3d.camera.PinholeCameraIntrinsic(img_width, img_height, fx, fy, cx, cy)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, cam_mat)
    return pcd


def transform_pcd_to_lidar_scale(pcd):
    pcd.scale(1e5, center=(0, 0, 0))
    R = pcd.get_rotation_matrix_from_xyz((np.pi, np.pi / 2, np.pi / 2))
    # after this rotation the coordinates x,y,z -> z, -x, -y
    pcd.rotate(R, center=(0, 0, 0))
    return pcd


def transform_params():
    fovx = math.radians(110)
    fovy = math.radians(70)
    img_height = 720
    img_width = 1280
    fx = img_width / (2 * math.tan(fovx / 2))
    fy = img_height / (2 * math.tan(fovy / 2))
    cx = img_width / 2
    cy = img_height / 2
    depth_scale = 1e3
    return cx, cy, depth_scale, fx, fy, img_height, img_width


def assign_labels(lidar_points, depth_mapped_points, labels):
    '''Output fusion assign on each cone from the lidar output the corresponding label.
    :param lidar_output: the output from postprocessing on the lidarmodel
    :param stereo_yolo_out: the yolo numpy array with ['xmin', 'ymin', 'xmax', 'ymax', 'conf', 'class']
    :return:'''

    norms_matrix = np.empty((lidar_points.shape[0], depth_mapped_points.shape[0]))

    for j, lidar_point in enumerate(lidar_points):
        for i, depth_center in enumerate(depth_mapped_points):
            norms_matrix[j, i] = np.linalg.norm(depth_center - lidar_point)

    nearests = np.full(lidar_points.shape[0], -1)

    while np.any(nearests == -1):
        # selects minimum norm in the matrix
        lidar, depth = np.where(norms_matrix == np.min(norms_matrix))
        lidar_index = lidar[0]
        depth_index = depth[0]
        # assign the nearest point label
        nearests[lidar_index] = depth_index
        # set all the norms for the pair to inf in order to exclude them from the other assignments
        norms_matrix[lidar_index, :] = np.inf
        norms_matrix[:, depth_index] = np.inf

    lidar_labels = labels[nearests]
    return lidar_labels


def sensor_fusion(lidar_centres, depth_centres, labels, model="single"):
    # map to the lidar space via the model
    if model == "multi":
        with open("interpolation_models/xyz_svr.pkl", mode="rb") as fp:
            modelxyz = pickle.load(fp)
            mapped_points = modelxyz.predict(depth_centres)

    elif model == "single":
        with open("interpolation_models/x axis_svr.pkl", mode="rb") as fp:
            modelx = pickle.load(fp)
        with open("interpolation_models/y_axis_svr.pkl", mode="rb") as fp:
            modely = pickle.load(fp)
        with open("interpolation_models/z_axis_svr.pkl", mode="rb") as fp:
            modelz = pickle.load(fp)

        mapped_points = np.zeros(shape=depth_centres.shape)
        mapped_points[:, 0] = modelx.predict(depth_centres[:, 0].reshape(-1, 1))
        mapped_points[:, 1] = modely.predict(depth_centres[:, 1].reshape(-1, 1))
        mapped_points[:, 2] = modelz.predict(depth_centres[:, 2].reshape(-1, 1))

    # compute 1-nn in the lidar space
    final_labels = assign_labels(lidar_centres, mapped_points, labels)

    return np.hstack((mapped_points, final_labels.reshape(-1, 1)))


lidar_path = "post_process_data"
pcd_file = "post_process_data/lidar.pcd"
pcd_labels_file = "post_process_data/lidar.csv"

rgb_labels_file = "stereo/stereo_yolo_boxes.txt"
rgb_image = "stereo/rgb_camera.png"
depth_image = "stereo/depth_camera.png"

# compute centers over pcd points
lidar_centres = post_processing_frompath(lidar_path, front_cut=True)
# load rgb labels
depth_centres, labels = depth_to_pcd(depth_image, rgb_image, rgb_labels_file)
output = sensor_fusion(lidar_centres, depth_centres, labels)
print(output)
