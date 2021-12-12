#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension, Float32
import numpy as np
import cv2
from sensory.utilities import from_matrix_to_image
from sensory.post_processing import *
#from utilities import from_matrix_to_image
#from post_processing import *

import os
import torch


class SubscribePointCloud(object):
    def __init__(self):
        rospy.init_node('subscribe_custom_point_cloud')
        self.script_path = os.path.dirname(os.path.realpath(__file__))
        self.model = torch.hub.load(self.script_path + '/ultralytics_yolov5_master', 'custom',
                                    path=self.script_path + '/ultralytics_yolov5_master/weights/lidar_weights.pt', source='local', force_reload = True)
       
        self.set_model_confidence()
        self.model.to(torch.device("cuda"))
        rospy.Subscriber('/carla/ego_vehicle/lidar', PointCloud2, self.callback, queue_size=1)
        rospy.Subscriber('/lidar/confidence', Float32, self.confidence_callback)
        self.pub = rospy.Publisher('/model/lidar/output', Float32MultiArray, queue_size=1)
        
        rospy.spin()

    def writeFile(self, r_minmax, theta_minmax, phi_minmax):
        f = open(self.script_path + "/minmax.txt", "w")
        f.write(str(r_minmax[0])+" "+str(r_minmax[1]))
        f.write(" "+str(theta_minmax[0])+" "+str(theta_minmax[1]))
        f.write(" "+str(phi_minmax[0]) + " " + str(phi_minmax[1]))
        f.close()
    
    def set_model_confidence(self):
        def_confidence = 0.4
        if rospy.has_param("/lidar/model_confidence"):
            conf = float(rospy.get_param("/lidar/model_confidence"))
            self.model.conf = conf
            rospy.loginfo('Using confidence found in the param server: '+ str(conf))
        else:
            rospy.logwarn('Lidar model confidence parameter is not setted in the param server. Using default parameter: '+ str(def_confidence)) 
            self.model.conf = def_confidence # default confidence

    def confidence_callback(self, confidence):
        confidence=confidence.data
        if type(confidence) is float:
            rospy.loginfo("Received a new value for the confidence: " + str(confidence))
            self.model.conf = confidence     

    def create_image_from_lidar(self, point_cloud:PointCloud2):
        i=0
        n_rows, n_colums = point_cloud.width, 3
        m_from_lidar = np.zeros((n_rows, n_colums))
        for point in pc2.read_points(point_cloud):
            m_from_lidar[i] = np.array([point[0],point[1],point[2]])
            i = i+1

        img_name = 'lidar_frame_out_'+ str(point_cloud.header.seq)+'.png'
        #rospy.loginfo(img_name)
        #save_numpy_as_pcd(m_from_lidar, './lidar_pcd.pcd')
        img = from_matrix_to_image(m_from_lidar, img_name=img_name, out_img=False)
        #cv2.imshow("Image window", img) #only for visualization purpose   
        #cv2.waitKey(1) #only for visualization purpose
        return img, m_from_lidar

    def callback(self, lidar_frames):
        
        img, array_cloud = self.create_image_from_lidar(lidar_frames)
        #print(img.shape)
        
        results = self.model(img)#.reshape((img.shape[0], img.shape[1])))
        yolo_boxes = results.pandas().xyxy[0]
        
        raw_pcd = pd.DataFrame(array_cloud, columns=["X", "Y", "Z"])
        raw_pcd['theta'], raw_pcd['phi'], raw_pcd['r'] = cart2sph(raw_pcd['Y'], raw_pcd["Z"], raw_pcd['X']) #convert to spherical coordinate
        r_minmax, theta_minmax, phi_minmax = (raw_pcd['r'].min(), raw_pcd['r'].max()), (raw_pcd['theta'].min(), raw_pcd['theta'].max()), (raw_pcd['phi'].min(), raw_pcd['phi'].max())

        img_df = pd.DataFrame()
        img_df['r'], img_df['phi'], img_df['theta'] = read_spherical_image(img)

        #self.writeFile(r_minmax, theta_minmax, phi_minmax)


        output:np.ndarray = post_processing(raw_pcd, (r_minmax, theta_minmax, phi_minmax), yolo_boxes, img_df, (480, 480), 255, OutputTypes.CENTER)
        #print(output)

        '''results.save(self.script_path + '/lidars/')
        if len(results.files)>0:
            tmp = self.script_path + '/lidars/' + results.files[0]
            img = cv2.imread(tmp)
            cv2.imshow("Image window", img) #only for visualization purpose   
            cv2.waitKey(1) #only for visualization purpose
        else:
            cv2.imshow("Image window", results.imgs) #only for visualization purpose   
            cv2.waitKey(1) #only for visua'''


        #with open("postprocessed.txt", mode='w') as post:
        #np.savetxt(self.script_path + "/postprocessed.txt", output)
            
        mat = Float32MultiArray()
        mat.layout.dim.append(MultiArrayDimension())
        mat.layout.dim.append(MultiArrayDimension())
        mat.layout.dim[0].label = "height"
        mat.layout.dim[1].label = "width"
        mat.layout.dim[0].size = output.shape[0]
        mat.layout.dim[1].size = output.shape[1]
        
        matrix_flat = output.flatten().tolist()
        mat.data = matrix_flat

        #print(output)
        #print()

        self.pub.publish(mat)

        #time.sleep(4)


def main():
    try:
        SubscribePointCloud()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
