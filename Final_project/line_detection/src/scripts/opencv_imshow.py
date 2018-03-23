#!/usr/bin/env python

#env python3

# This script uses cv_bridge package to convert images from /sensor_msgs/Image topic to
#    OpenCV messages and display them on the screen

import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge

class Line_Detector:
        def __init__(self):
            self.bridge = cv_bridge.CvBridge()
            cv2.namedWindow("window", 1)
            self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
            
        def image_callback(self, msg):
            print("Image updated.")
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("window", image)
            cv2.waitKey(3)
            print("Image updated.")

rospy.init_node('detector')
detector = Line_Detector()
rospy.spin()
