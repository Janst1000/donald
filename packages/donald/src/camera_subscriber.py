#!/usr/bin/env python3

# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

# Duckietown Ros
#from duckietown.dtros import DTROS, NodeType

class image_detector():

    def __init__(self):
        self.subscriber = rospy.Subscriber("/donald/camera_node/image/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        self.img_pub = rospy.Publisher("/donald/camera_node/circles", CompressedImage)

    def callback(self, ros_data):
        # Load image into an numpy array
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # Converting image to grayscale
        gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
        circles_img = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT, 1, 20, param1=100,param2=100,minRadius=0,maxRadius=0)
        circles_img = np.uint16(np.around(circles_img))
        
        cv2.imshow('Original Image',circles_img)
        #cv2.imshow('Detected Circles',circles_img)

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', circles_img)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)
        

if __name__ == '__main__':
    id = image_detector()
    rospy.init_node('image_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindo