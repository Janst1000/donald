#!/usr/bin/env python3
import os
import time
from turtle import circle
import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from duckietown_utils import load_camera_intrinsics, rectify


class circle_detector():

    def __init__(self):
        # getting namespace
        #self.veh_name = rospy.get_namespace().strip("/")
        self.veh_name = os.environ['VEHICLE_NAME']
        self.subscriber = rospy.Subscriber("/donald/camera_node/image/compressed", CompressedImage, self.callback, queue_size=1)

        self.node_rate = 1
        # creating publisher
        self.image_pub = rospy.Publisher("/circles", Image, queue_size=1)
        # initializing cv bridge
        self.bridge = CvBridge()

#        try:
#            self.image_message = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")
#        except CvBridgeError as e:
#            print(e)

    def callback(self, ros_data):
        circle_param1 = rospy.get_param("circle1")
        circle_param2 = rospy.get_param("circle2")
        # getting image as cv2 object
        camera_image = self.bridge.compressed_imgmsg_to_cv2(ros_data, "bgr8")
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # Converting image to grayscale
        gray = cv2.cvtColor(camera_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)

        #corrected = self.correct_Image(camera_image)

        #circles = None
        circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,20,
                            param1=circle_param1,param2=circle_param2,minRadius=0,maxRadius=100)
        #circles_img = np.uint16(np.around(circles_img))
        circles_img = camera_image
        if circles is not None:
          circles = np.uint8(np.around(circles))
          rospy.loginfo(circles)
          for i in circles[0, :]:
              center = (i[0], i[1])
              # circle center
              cv2.circle(circles_img, center, 1, (0, 100, 100), 3)
              # circle outline
              radius = i[2]
              cv2.circle(circles_img, center, radius, (255, 0, 255), 3)
        
        #draw = cv2.circle(camera_image, (100, 100), 50, (0,0,255), 2)
        try:
           self.image_message = self.bridge.cv2_to_imgmsg(circles_img, "bgr8")
        except CvBridgeError as e:
            rospy.loginfo(e)
        self.image_pub.publish(self.image_message)
    """
    def correct_Image(self, image):
      print(self.veh_name)
      print(get_duckiefleet_root())
      intrinsics = load_camera_intrinsics(self.veh_name)
      
      corrected_image = rectify(image, intrinsics)
      return corrected_image
    """

if __name__ == '__main__':
  #initializing the node
    ip = circle_detector()
    rospy.init_node('circle_detector', anonymous=True)
    try:
      # Keeping the node active and running it
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows
