#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import torch
import pandas
import os
from PIL import Image as pilimg
from matplotlib import pyplot as plt

class SubscribeRGBFrontImage(object):
    def __init__(self):
        rospy.init_node('subscribe_rbg_front_image')
        self.script_path = os.path.dirname(os.path.realpath(__file__))
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.script_path+'/camera_weights.pt')
        self.model.conf = 0.74 # NMS confidence threshold
        self.bridge = CvBridge()
        rospy.Subscriber('/carla/ego_vehicle/rgb_front/image', Image, self.callback)
        self.pub = rospy.Publisher('/model/camera/output', String, queue_size=10)
        rospy.spin()

    def create_image(self, rgb_image : Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_image, "rgb8")
            #pilimg.frombuffer(rgb_image.data).show()
            #cv_image = Image.frombytes(cv_image)
            #cv_image.show()
        except CvBridgeError as e:
            print(e)


        #cv2.imwrite('./cameras/'+str(rgb_image.header.seq)+'.png', cv_image)
        #cv2.imshow("Image window", cv_image) #only for visualization purpose   
        #cv2.waitKey(1) #only for visualization purpose
        return cv_image

    def callback(self, rgb_img):

        img = self.create_image(rgb_img)

        results = self.model(img)#.reshape((img.shape[0], img.shape[1])))
        ris = results.pandas().xyxy[0]
        
        
        #ris.to_csv(path_or_buf='./yolo_out.csv', index=False)
        #print(ris)
        
        results.save(self.script_path + '/cameras/')
        if len(results.files)>0:
            tmp = self.script_path + '/cameras/' + results.files[0]
            img = cv2.imread(tmp)
            cv2.imshow("Image window", img) #only for visualization purpose   
            cv2.waitKey(1) #only for visualization purpose
        else:
            cv2.imshow("Image window", results.imgs) #only for visualization purpose   
            cv2.waitKey(1) #only for visua
        #time.sleep(2)
        self.pub.publish("hello_str")


def main():
    try:
        SubscribeRGBFrontImage()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
