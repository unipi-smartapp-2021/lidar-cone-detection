#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2


class SubscribeRGBFrontImage(object):
    def __init__(self):
        rospy.init_node('subscribe_rbg_front_image')
        self.bridge = CvBridge()
        rospy.Subscriber('/carla/ego_vehicle/rgb_front/image', Image, self.callback)
        rospy.spin()

    def create_image(self, rgb_image : Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_image, "bgr8")

        except CvBridgeError as e:
            print(e)


        #cv2.imwrite('./cameras/'+str(rgb_image.header.seq)+'.png', cv_image)
        cv2.imshow("Image window", cv_image) #only for visualization purpose   
        cv2.waitKey(1) #only for visualization purpose
        return cv_image

    def callback(self, rgb_img):

        img = self.create_image(rgb_img)
        #time.sleep(2)


def main():
    try:
        SubscribeRGBFrontImage()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
