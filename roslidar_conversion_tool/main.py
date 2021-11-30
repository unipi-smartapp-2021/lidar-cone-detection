import cv2
import ros_numpy  # apt install ros-noetic-ros-numpy
import rosbag
import numpy as np
from PIL import Image
from pypcd import pypcd
import open3d as o3d
from scipy.interpolate.ndgriddata import griddata
import pandas as pd
import matplotlib.pyplot as plt
from scipy.ndimage import rotate
import math
"""
Simple tool to convert a RosBag file containing Lidar Topics into a
numpy arrays and .pcd files.
"""

def convert_pc_msg_to_np(pc_msg):
    """
    This method read a rosbag Lidar message and convert it in a Numpy array.
    :param pc_msg: need the rosbag message
    :return: numpy array X,Y,Z of the Point Cloud and pcd_object.
    """
    offset_sorted = {f.offset: f for f in pc_msg.fields}
    pc_msg.fields = [f for (_, f) in sorted(offset_sorted.items())]

    # Conversion from PointCloud2 msg to np array.
    pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg, remove_nans=True)
    df = pd.DataFrame(data=pc_np, columns=["X", "Y", "Z"])
    print(df.X.describe())
    print(df.Y.describe())
    print(df.Z.describe())
    pc_np = np.asarray(list(filter(lambda x: ~(-90 < x[0] < -10) and (-0.15 < x[2] < 0.90), pc_np))) # Cutting the view between 120°
    pcd_obj = pypcd.make_xyz_point_cloud(pc_np)
    return pc_np, pcd_obj  # point cloud in numpy and pcd format


def cart2sph(x, y, z):
    xy = np.sqrt(x ** 2 + y ** 2)  # sqrt(x² + y²)
    x_2 = x ** 2
    y_2 = y ** 2
    z_2 = z ** 2
    r = np.sqrt(x_2 + y_2 + z_2)  # r = sqrt(x² + y² + z²)
    theta = np.arctan2(y, x)
    phi = np.arctan2(xy, z)
    return r, theta, phi


def polar2cart(r, theta, phi):
    return [
         r * math.sin(theta) * math.cos(phi),
         r * math.sin(theta) * math.sin(phi),
         r * math.cos(theta)
    ]

"""i = 0
topic_to_filter = "/carla/ego_vehicle/lidar"
bag_file__path = './lidar_bag.bag'
pcd_output_path = './pcd_outputs'
for topic, msg, t in rosbag.Bag(bag_file__path).read_messages():
    if topic == topic_to_filter:
        pc_np, pcd_obj = convert_pc_msg_to_np(msg)
        pcd_obj.save(pcd_output_path+'/cloud'+str(i)+'.pcd')
        i += 1"""

""" Visualizzation of .pcd files"""
cloud = o3d.io.read_point_cloud("./pcd_outputs/cloud0.pcd") # Read the point cloud file
out_arr = np.asarray(cloud.points)
out_arr = np.asarray(list(filter(lambda x: (0 < x[0] < 40) and (-0.5 < x[2] < 0.90), out_arr)))
df = pd.DataFrame(data=out_arr, columns=["X", "Y", "Z"])
r, theta, phi = cart2sph(df.Y.values, df.Z.values, df.X.values, )
df['R'] = r
df.R = ((df.R - df.R.min()) / (df.R.max() - df.R.min()))*255
df['tetha'] = ((theta - np.min(theta)) / (np.max(theta) - np.min(theta)))*600
df['phi'] = ((phi - np.min(phi)) / (np.max(phi) - np.min(phi)))*400
print("Min theta:" + str(np.mean(np.abs(theta[:-1] - theta[1:])))) # 0.25 tetha
print("Min phi:" + str(np.mean(np.abs(phi[:-1] - phi[1:]))))
image = np.ones([401, 601, 1], np.uint8)*255
df = df.astype(int)
for index, row in df.iterrows():
    image[row["phi"], row["tetha"]] = row["R"]

df.plot.scatter(x='tetha', y='phi', c='R', colormap='viridis', s=5)
plt.show()
arr = df[["tetha", "phi", "R"]].values
cv2.imwrite('a.png', image)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(out_arr)
# o3d.io.write_point_cloud("./sync.ply", pcd)

pcd_load = o3d.io.read_point_cloud("./sync.ply")
# o3d.visualization.draw_geometries([pcd_load])
# o3d.visualization.draw_geometries([cloud]) # Visualize the point cloud