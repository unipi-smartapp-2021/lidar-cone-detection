import numpy as np
import pandas as pd
import rospy
import time
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension, Float32
import os
import argparse

from sensory.utilities import min_max_scale, convert_numpy_to_rosMultiArr


class OutputFusion(object):
    def __init__(self):
        rospy.init_node('output_fusion')
        self.script_path = os.path.dirname(os.path.realpath(__file__))
        self.parsed_camera = None
        rospy.loginfo("Ready")
        rospy.Subscriber('/model/lidar/output', Float32MultiArray, self.callback_lidar, queue_size=1)
        rospy.Subscriber('/model/camera/output', Float32MultiArray, self.callback_camera, queue_size=1)
        self.pub = rospy.Publisher('/sensor_fusion/output', Float32MultiArray, queue_size=1)
        
        rospy.spin()


    def callback_lidar(self, lidar_out:Float32MultiArray):

        parsed_lidar = self.parse_message(lidar_out)

        if self.parsed_camera is not None:
            fusion = self.output_fusion(parsed_lidar, self.parsed_camera)
            
            if opt.visualize:
                print(fusion)
                
            mat = convert_numpy_to_rosMultiArr(fusion)
            self.pub.publish(mat)
                #time.sleep(1)

    def callback_camera(self, camera_out:Float32MultiArray):

        self.parsed_camera = self.parse_message(camera_out)

    

    def parse_message(self, message:Float32MultiArray):
        out_from_lidar = message.data
        parsed_array = np.zeros((message.layout.dim[0].size, message.layout.dim[1].size))
        for i in range(parsed_array.shape[0]):
            for j in range(parsed_array.shape[1]):
                parsed_array[i,j] = out_from_lidar[i*parsed_array.shape[1] + j]
                index = i*parsed_array.shape[1] + j
        return parsed_array        


    def output_fusion(self, lidar_output, stereo_yolo_out):
        '''Output fusion assign on each cone from the lidar output the corresponding label.
        :param lidar_output: the output from postprocessing on the lidarmodel
        :param stereo_yolo_out: the yolo numpy array with ['xmin', 'ymin', 'xmax', 'ymax', 'conf', 'class']
        :return:'''
        try:
            lidar_output = pd.DataFrame(lidar_output, columns=['x', 'y', 'z'])
        except ValueError:
            return np.empty((0,4))
        try:
            stereo_yolo_out = pd.DataFrame(stereo_yolo_out, columns=['xmin', 'ymin', 'xmax', 'ymax', 'conf', 'class'])
        except ValueError:
            return np.empty((0,4))

        if lidar_output.empty or stereo_yolo_out.empty:
            return np.empty((0,4))

        lidar_output['ys'], _ = min_max_scale((0, 1280), lidar_output['y'].values, max=15, min=-15)

        stereo_yolo_out['xcenter'] = stereo_yolo_out['xmin'] + (stereo_yolo_out['xmax'] - stereo_yolo_out['xmin']) / 2
        stereo_yolo_out['ycenter'] = stereo_yolo_out['ymin'] + (stereo_yolo_out['ymax'] - stereo_yolo_out['ymin']) / 2
        stereo_yolo_out = stereo_yolo_out.sort_values(by=['ycenter'], ascending=False)
        stereo_yolo_out = stereo_yolo_out[:2]
        
        lidar_output['c'] = -1
        if len(lidar_output) > len(stereo_yolo_out):
            for i, cam_box in stereo_yolo_out.iterrows():
                lidar_output['distance'] = np.abs(cam_box['xcenter'] - lidar_output['ys'])
                lidar_index = lidar_output[lidar_output['distance'] == lidar_output['distance'].min()].index[0]
                lidar_output.at[lidar_index, 'c'] = cam_box['class']
        else:
            for i, lidar_box in lidar_output.iterrows():
                stereo_yolo_out['distance'] = np.abs(stereo_yolo_out['xcenter'] - lidar_box['ys'])
                classe = stereo_yolo_out[stereo_yolo_out['distance'] == stereo_yolo_out['distance'].min()]['class']
                if len(classe) == 1:
                    lidar_output.at[i, 'c'] = classe
                elif len(classe) > 1:
                    lidar_output.at[i, 'c'] = classe[0]

        lidar_output = lidar_output[['x', 'y', 'z', 'c']][lidar_output['c'] != -1]
        return lidar_output.values

def parse_arguments(known=False):
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--visualize', action='store_true', help= "It will open a window and shows the cone detection made by the stereo camera model.")
    opt = parser.parse_known_args()[0] if known else parser.parse_args()
    return opt

def main():
    try:
        OutputFusion()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    opt = parse_arguments()
    main()

