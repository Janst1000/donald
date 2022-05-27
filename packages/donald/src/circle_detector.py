#!/usr/bin/env python3
import time
from turtle import circle
import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class circle_detector():

    def __init__(self):
        self.subscriber = rospy.Subscriber("/image_topic", CompressedImage, self.callback, queue_size=1)

        self.node_rate = 1
        # creating publisher
        self.image_pub = rospy.Publisher("circles", Image, queue_size=10)
        # reading image from file
        cv_image = cv2.imread('./image.png',0)
        # initializing cv bridge
        self.bridge = CvBridge()

        try:
            self.image_message = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")
        except CvBridgeError as e:
            print(e)
    
    def callback(self, ros_data):
        # Load image into an numpy array
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # Converting image to grayscale
        #gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
        rospy.loginfo(image_np.shape)
        circles = None
        circles_img = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT, 1, 20, circles, param1=100,param2=100,minRadius=0,maxRadius=0)
        circles_img = np.uint16(np.around(circles_img))
        print(circle)
        self.image_message = circles_img
        self.image_pub.publish(self.image_message)

if __name__ == '__main__':
  #initializing the node
    ip = circle_detector()
    rospy.init_node('circle_detector', anonymous=True)
    try:
      # Keeping the node active and running it
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindo
