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

    def callback(self, ros_data):
        # Load image into an numpy array
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # Converting image to grayscale
        gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
        print(gray)

if __name__ == '__main__':
    id = image_detector()
    rospy.init_node('image_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindo