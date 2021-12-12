#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Float32MultiArray, MultiArrayDimension
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import torch
import os

class SubscribeRGBFrontImage(object):
    def __init__(self):
        rospy.init_node('subscribe_rbg_front_image')
        self.script_path = os.path.dirname(os.path.realpath(__file__))
        self.model = torch.hub.load(self.script_path + '/ultralytics_yolov5_master', 'custom',
                                    path=self.script_path + '/ultralytics_yolov5_master/weights/camera_weights.pt', source='local', force_reload=True)                       
        self.set_model_confidence()
        self.model.to(torch.device("cuda"))
        self.bridge = CvBridge()
        rospy.Subscriber('/carla/ego_vehicle/rgb_front/image', Image, self.callback)
        #rospy.Subscriber('/carla/ego_vehicle/depth_front/image', Image, self.depth_callback)
        rospy.Subscriber('/camera/confidence', Float32, self.confidence_callback)
        self.pub = rospy.Publisher('/model/camera/output', Float32MultiArray, queue_size=1)
        rospy.spin()

    def depth_callback(self, depth_img):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(depth_img)
            cv2.imwrite(self.script_path + '/depth_camera.png', cv_image)
            #pilimg.frombuffer(rgb_image.data).show()
            #cv_image = Image.frombytes(cv_image)
            #cv_image.show()
        except CvBridgeError as e:
            print(e)
        pass


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


        #cv2.imwrite('./cameras/'+str(rgb_image.header.seq)+'.png', cv_image)
        #cv2.imshow("Image window", cv_image) #only for visualization purpose   
        #cv2.waitKey(1) #only for visualization purpose
        return cv_image

    def callback(self, rgb_img):

        img = self.create_image(rgb_img)
        #cv2.imwrite(self.script_path + '/rgb_camera.png', img)
        results = self.model(img)#.reshape((img.shape[0], img.shape[1])))
        ris = results.pandas().xyxy[0]
        
        #np.savetxt(self.script_path + "/stereo_yolo_boxes.txt", np.array(ris.values[:,:-1], dtype=np.float32))
        
        #ris.to_csv(path_or_buf='./yolo_out.csv', index=False)
        #print(ris)
        
        '''results.save(self.script_path + '/cameras/')
        if len(results.files)>0:
            tmp = self.script_path + '/cameras/' + results.files[0]
            img = cv2.imread(tmp)
            cv2.imshow("Image window", img) #only for visualization purpose   
            cv2.waitKey(1) #only for visualization purpose
        else:
            cv2.imshow("Image window", results.imgs) #only for visualization purpose   
            cv2.waitKey(1) #only for visua'''
        
        to_publish = ris.values[:,:-1]
        mat = Float32MultiArray()
        mat.layout.dim.append(MultiArrayDimension())
        mat.layout.dim.append(MultiArrayDimension())
        mat.layout.dim[0].label = "height"
        mat.layout.dim[1].label = "width"
        mat.layout.dim[0].size = to_publish.shape[0]
        mat.layout.dim[1].size = to_publish.shape[1]
        
        matrix_flat = to_publish.flatten().tolist()
        mat.data = matrix_flat

        #print(output)
        #print()

        self.pub.publish(mat)
        


def main():
    try:
        SubscribeRGBFrontImage()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
