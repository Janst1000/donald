#!/usr/bin/env python3
import os
from os.path import isfile
from sys import exit
import time
from turtle import circle
import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import undistort


class circle_detector():

    def __init__(self):
        self.veh_name = os.environ['VEHICLE_NAME']
        self.subscriber = rospy.Subscriber("/donald/camera_node/image/compressed", CompressedImage, self.callback, queue_size=1)

        self.node_rate = 30
        # creating publisher
        self.image_pub = rospy.Publisher("/circles", Image, queue_size=1)
        self.gray_pub = rospy.Publisher("/gray", Image, queue_size=1)
        #self.bin_pub = rospy.Publisher("/binary_image", Image, queue_size=1)
        # initializing cv bridge
        self.bridge = CvBridge()
        self.intrinsics = undistort.load_camera_intrinsics(self.veh_name)


    def callback(self, ros_data):
        # Getting some ros parameters for the circle functions
        circle_param1 = rospy.get_param("circles/param1", 25)
        circle_param2 = rospy.get_param("circles/param2", 25)
        minR = rospy.get_param("circles/min", 0)
        maxR = rospy.get_param("circles/max", 15)
        blur = rospy.get_param("circles/blur", 5)
        # getting image as cv2 object
        camera_image = self.bridge.compressed_imgmsg_to_cv2(ros_data, "bgr8")
        # correcting the image
        corrected = undistort.rectify(camera_image, self.intrinsics)

        # Converting image to grayscale 
        gray = cv2.cvtColor(corrected, cv2.COLOR_BGR2GRAY)
        # blurring the image
        gray = cv2.medianBlur(gray, blur)
        gray = cv2.subtract(gray, 50)
        
        #binary = cv2.threshold(gray,235,255,cv2.THRESH_BINARY)
        #publishing the grayscale image
        self.gray_message = self.bridge.cv2_to_imgmsg(gray, "mono8")
        self.gray_pub.publish(self.gray_message)

        # Detecting the circles
        circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,20,
                            param1=circle_param1,param2=circle_param2,minRadius=minR,maxRadius=maxR)
        
        # Copying the corrected image for drawing the resulting circles
        circles_img = corrected

        # Drawing the circles
        if circles is not None:
          circles = np.uint16(np.around(circles))
          rospy.loginfo(circles)
          for i in circles[0, :]:
              center = (i[0], i[1])
              # circle center
              cv2.circle(circles_img, center, 1, (0, 100, 100), 3)
              # circle outline
              radius = i[2]
              cv2.circle(circles_img, center, radius, (255, 0, 255), 3)
        
        # Publishing the Circles image
        try:
           self.image_message = self.bridge.cv2_to_imgmsg(circles_img, "bgr8")
        except CvBridgeError as e:
            rospy.loginfo(e)
        self.image_pub.publish(self.image_message)


      

if __name__ == '__main__':
  #initializing the node
    ip = circle_detector()
    rospy.init_node('circle_detector', anonymous=True)
    try:
      # Keeping the node active and running it
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down circle_detector")
