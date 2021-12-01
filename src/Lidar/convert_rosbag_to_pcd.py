from ros_numpy.src import ros_numpy
from tqdm import tqdm
import rosbag
import numpy as np
import argparse
from pypcd import pypcd


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
    pc_np = np.asarray(
        list(filter(lambda x: ~(-90 < x[0] < -10) and (-0.15 < x[2] < 0.90), pc_np)))  # Cutting the view between 120°
    pcd_obj = pypcd.make_xyz_point_cloud(pc_np)
    return pc_np, pcd_obj  # point cloud in numpy and pcd format


def parse_arguments(known=False):
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', type=str, default="/carla/ego_vehicle/lidar", help='message topic')
    parser.add_argument('--rosbag', type=str, default="", help='dataset_bag.bag path')
    parser.add_argument('--path', type=str, default="src/Lidar/datasets",
                        help='path where the pointclouds are going to be stored')
    parser.add_argument('--save', type=str, default=False, help='save with built-in method')
    opt = parser.parse_known_args()[0] if known else parser.parse_args()
    return opt


if __name__ == "__main__":
    """
    Simple tool to convert a RosBag file containing Lidar Topics into a numpy arrays and .pcd files.
    """
    opt_parser = parse_arguments()
    topic_to_filter = opt_parser.topic
    bag_file__path = opt_parser.rosbag
    pcd_output_path = opt_parser.path
    save_with_method = opt_parser.save
    i = 0
    bag = [msg for msg in rosbag.Bag(bag_file__path).read_messages()]
    for topic, msg, t in tqdm(bag):
        if topic == topic_to_filter:
            pcd_np, pcd = convert_pc_msg_to_np(msg)
            if save_with_method:
                pcd.save(pcd_output_path + '/lidar_pointcloud' + str(i) + '.pcd')
            else:
                with open(pcd_output_path + "/lidar_pointcloud" + str(i) + ".pcd", "w") as datafile:
                    for label, data in zip(pcd.get_metadata(), pcd.get_metadata().values()):
                        label = label.upper()
                        if type(data) == list:
                            data = " ".join([str(x) for x in data])

                        datafile.write("{0} {1}\n".format(label, data))

                    for pointcloud in pcd.pc_data:
                        pointcloud_str = ""
                        for point in pointcloud[0]:
                            pointcloud_str = pointcloud_str + str(point) + " "

                        datafile.write("{0}\n".format(pointcloud_str[:-1]))

            i += 1
