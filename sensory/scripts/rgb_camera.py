#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import torch
import os
import argparse
from sensory.utilities import convert_numpy_to_rosMultiArr, visualization

class SubscribeRGBFrontImage(object):
    def __init__(self):
        rospy.init_node('subscribe_rbg_front_image')
        self.script_path = os.path.dirname(os.path.realpath(__file__))
        self.model = torch.hub.load(self.script_path + '/yolov5', 'custom',
                                    path=self.script_path + '/weights/camera_weights.pt', source='local', force_reload=True)                       
        self.set_model_confidence()
        self.model.to(torch.device("cuda"))
        self.bridge = CvBridge()
        
        rospy.Subscriber('/carla/ego_vehicle/rgb_front/image', Image, self.callback)  
        rospy.Subscriber('/camera/confidence', Float32, self.confidence_callback)
        self.pub = rospy.Publisher('/model/camera/output', Float32MultiArray, queue_size=1)
        rospy.loginfo("Ready")
        rospy.spin()

    def confidence_callback(self, confidence):
        confidence=confidence.data
        if type(confidence) is float:
            rospy.loginfo("Received a new value for the confidence: " + str(confidence))
            self.model.conf = confidence


    def set_model_confidence(self):
        def_confidence = 0.74
        if rospy.has_param("/camera/model_confidence"):
            conf = float(rospy.get_param("/camera/model_confidence"))
            self.model.conf = conf
            rospy.loginfo('Using confidence found in the param server: '+ str(conf))
        else:    
            rospy.logwarn('Camera model confidence parameter is not setted in the param server. Using default parameter: '+ str(def_confidence)) 
            self.model.conf = def_confidence# default confidence


    def create_image(self, rgb_image : Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_image, "rgb8")
            #pilimg.frombuffer(rgb_image.data).show()
            #cv_image = Image.frombytes(cv_image)
            #cv_image.show()
        except CvBridgeError as e:
            print(e)
        return cv_image

    def callback(self, rgb_img):

        img = self.create_image(rgb_img)
        #cv2.imwrite(self.script_path + '/rgb_camera.png', img)
        results = self.model(img)#.reshape((img.shape[0], img.shape[1])))
        ris = results.pandas().xyxy[0]
       
        if opt.visualize:
            visualization(results)
        
        to_publish = ris.values[:,:-1]
        mat = convert_numpy_to_rosMultiArr(to_publish)

        #print(output)
        #print()

        self.pub.publish(mat)

def parse_arguments(known=False):
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--visualize', action='store_true', help= "It will open a window and shows the cone detection made by the stereo camera model.")
    opt = parser.parse_known_args()[0] if known else parser.parse_args()
    return opt
    


def main():
    
    try:
        SubscribeRGBFrontImage()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    opt = parse_arguments()
    main()
