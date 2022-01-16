import math
import os

import cv2
import numpy as np
import open3d as o3d
import pandas as pd

from src.lidar.utils import read_pcd_and_filter, visualize_cartesian





def convert_depth_to_pcd_batch(depths_basepath, rgb_basepath, output_path):
    if not os.path.exists(output_path):
        os.mkdir(output_path)

    for depth_filename in os.listdir(depths_basepath):
        pcd = depth_to_pcd(os.path.join(depths_basepath, depth_filename), os.path.join(rgb_basepath, depth_filename))
        o3d.io.write_point_cloud(os.path.join(output_path, depth_filename.replace("png", "pcd")), pcd)


def get_centre_from_boxes(boxes, pcds_path):
    for i, pcd in enumerate(os.listdir(pcds_path)):
        print("reading {}".format(pcd))
        path = os.path.join(pcds_path, pcd)
        pcd_arr = read_pcd_and_filter(path, front_cut=False)
        df = pd.DataFrame(pcd_arr)
        df.columns = ["X", "Y", "Z"]
        boxes = boxes[i]
        print("number of boxes ", len(boxes))
        for box in boxes:
            x, y, z, dx, dy, dz = box

            inner_points = df[df["Y"].between(y, y + dy)]
            inner_points = inner_points[inner_points["Z"].between(z, z + dz)]
            inner_points = inner_points[inner_points["X"].between(x, x + dx)]

            print("\tnumber of points ", len(inner_points.values))
    pass


def load_matlab_exported_labels(depth_filename, lidar_filename):
    depth_pcds_path = "interpolation_data/depth_pcd"
    lidar_pcds_path = "interpolation_data/pcd"
    depth_boxes = get_boxes(depth_filename)
    lidar_boxes = get_boxes(lidar_filename)
    print(depth_boxes)

    # centres = get_centre_from_boxes(lidar_boxes, lidar_pcds_path)
    depth_centres = get_centres(depth_filename)
    lidar_centres = get_centres(lidar_filename)

    merged_dict = {}
    for frame in depth_centres.keys():
        merged_centres = []
        for i in range(len(depth_centres[frame])):
            merged_lists = depth_centres[frame][i] + lidar_centres[frame][i]
            merged_centres.append(merged_lists)
        merged_dict[frame] = merged_centres
    print(merged_dict)
    only_centres = []
    for centres in merged_dict.values():
        only_centres += centres

    header = ["depth_x", "depth_y", "depth_z", "lidar_x", "lidar_y", "lidar_z"]
    df = pd.DataFrame(data=only_centres)
    df.columns = header
    df = df.dropna(axis=0, how='all')
    print(df)
    df.to_csv("interpolation_data/centres.csv", index=False)


def get_centre(point):
    x = point[0] + point[3] / 2
    y = point[1] + point[4] / 2
    z = point[2] + point[5] / 2

    return [x, y, z]


def get_centres(matlab_labels_filename):
    """
    Returns a dictionary {frame: [list of centres]}
    """
    boxes = {}
    depth_df = pd.read_csv(matlab_labels_filename)
    for r, row in depth_df.iterrows():
        centres = []
        col_count = 0
        # print("number of colums ", len(depth_df.columns))
        for i, col in enumerate(depth_df.columns):
            if i > 0:
                if col_count == 0:
                    point = []
                if col_count < 6:
                    point.append(depth_df[col].values[r])
                if col_count == 8:
                    centre = get_centre(point)
                    centres.append(centre)
                    col_count = -1
                col_count += 1
        boxes[r] = centres
    return boxes


def get_boxes(matlab_labels_filename):
    boxes = {}
    depth_df = pd.read_csv(matlab_labels_filename)
    for r, row in depth_df.iterrows():
        points = []
        col_count = 0
        # print("number of colums ", len(depth_df.columns))
        for i, col in enumerate(depth_df.columns):
            if i > 0:
                if col_count == 0:
                    point = []
                if col_count < 6:
                    point.append(depth_df[col].values[r])
                if col_count == 8:
                    # print(point)
                    points.append(point)
                    col_count = -1
                col_count += 1
        boxes[r] = points
    return boxes

# convert_depth_to_pcd_batch("interpolation_data/depth", "interpolation_data/rgb", "interpolation_data/depth_pcd")
# load_matlab_exported_labels("interpolation_data/depth_labels.csv", "interpolation_data/pcd_labels.csv")
