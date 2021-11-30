import ros_numpy  # apt install ros-noetic-ros-numpy
import rosbag
import numpy as np
import argparse
from pypcd import pypcd
import pptk
import open3d as o3d


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
    pc_np = np.asarray(list(filter(lambda x: ~(-90 < x[0] < -10) and (-0.15 < x[2] < 0.90), pc_np))) # Cutting the view between 120°
    pcd_obj = pypcd.make_xyz_point_cloud(pc_np)
    return pc_np, pcd_obj  # point cloud in numpy and pcd format


def parse_arguments(known=False):
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', type=str, default="", help='message topic')
    parser.add_argument('--rosbag', type=str, default="", help='dataset_bag.bag path')
    parser.add_argument('--path', type=str, default="", help='path where the pointclouds are going to be stored')
    opt = parser.parse_known_args()[0] if known else parser.parse_args()
    return opt


if __name__ == "__main__":
    """
    Simple tool to convert a RosBag file containing Lidar Topics into a
    numpy arrays and .pcd files.
    """
    opt = parse_arguments()
    i = 0
    topic_to_filter = opt.topic
    bag_file__path = opt.rosbag
    pcd_output_path = opt.path
    for topic, msg, t in rosbag.Bag(bag_file__path).read_messages():
        if topic == topic_to_filter:
            pc_np, pcd_obj = convert_pc_msg_to_np(msg)
            pcd_obj.save(pcd_output_path+'/cloud'+str(i)+'.pcd')
            i += 1
