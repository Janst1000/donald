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
from sensor_msgs.msg import Image
# We do not use cv_bridge it does not support CompressedImage in python
from cv_bridge import CvBridge, CvBridgeError

# Duckietown Ros
#from duckietown.dtros import DTROS, NodeType

class ImagePublisher():

  def __init__(self):
    self.node_rate = 1
    # creating publisher
    self.image_pub = rospy.Publisher("image_topic", Image, queue_size=10)
    # reading image from file
    cv_image = cv2.imread('./image.png',0)
    # initializing cv bridge
    self.bridge = CvBridge()

    try:
      self.image_message = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")
    except CvBridgeError as e:
      print(e)

  def doSmth(self):
    #publishing the image
    rospy.loginfo('its working!')
    self.image_pub.publish(self.image_message)

  def run(self):
    #publishing at a fixed rate
    loop = rospy.Rate(self.node_rate)
    while not rospy.is_shutdown():
      #publishing the image
      self.doSmth()
      loop.sleep()

if __name__ == '__main__':
  #initializing the node
    ip = ImagePublisher()
    rospy.init_node('image_publisher', anonymous=True)
    try:
      # Keeping the node active and running it
        ip.run()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindo