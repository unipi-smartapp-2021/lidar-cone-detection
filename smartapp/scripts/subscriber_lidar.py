#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
import numpy as np
import cv2
import subprocess
from smartapp.utilities import from_matrix_to_image
import os
import torch
import pandas


class SubscribePointCloud(object):
    def __init__(self):
        rospy.init_node('subscribe_custom_point_cloud')
        self.script_path = os.path.dirname(os.path.realpath(__file__))
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.script_path+'/lidar_weights.pt')
        self.model.conf = 0.4  # NMS confidence threshold
        rospy.Subscriber('/carla/ego_vehicle/lidar', PointCloud2, self.callback, queue_size=1)
        self.pub = rospy.Publisher('/model/lidar/output', String, queue_size=10)
        #self.talker()
        rospy.spin()
    
    def talker(self):
        #rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(100) # 10hz
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            self.pub.publish(hello_str)
            rate.sleep()

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
        return img

    def callback(self, point_cloud):

        img:np.ndarray = self.create_image_from_lidar(point_cloud)
        #print(img.shape)
        
        results = self.model(img)#.reshape((img.shape[0], img.shape[1])))
        ris = results.pandas().xyxy[0]
        
        
        #ris.to_csv(path_or_buf='./yolo_out.csv', index=False)
        #print(ris)
        results.save(self.script_path + '/lidars/')
        if len(results.files)>0:
            tmp = self.script_path + '/lidars/' + results.files[0]
            img = cv2.imread(tmp)
            cv2.imshow("Image window", img) #only for visualization purpose   
            cv2.waitKey(1) #only for visualization purpose
        else:
            cv2.imshow("Image window", results.imgs) #only for visualization purpose   
            cv2.waitKey(1) #only for visua

        #cv2.imshow("Image window", results.imgs[0]) #only for visualization purpose   
        #cv2.waitKey(1) #only for visualization purpose
        
        #cv2.imshow("Image window", results) #only for visualization purpose   
        #cv2.waitKey(1) #only for visualization purpose
        self.pub.publish("hello_str")
        #time.sleep(2)


def main():
    try:
        SubscribePointCloud()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
