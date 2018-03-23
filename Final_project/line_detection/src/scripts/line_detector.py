#!/usr/bin/env python

# This script uses the cv_bridge package to do the following:
#   1) convert images from sensor_msgs/Image topic to OpenCV messages
#   2) convert their colors from RGB to HSV
#   3) apply a threshold of hues NEAR THE COLOR YELLOW to obtain a binary image.
#
# The script uses an approach called proportional and simple means.
#
# Code adapted from: edu.gaitech.hk/turtlebot/line-follower.html

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Line_Detector:
        def __init__(self):
            # Motion setup
            self.linear_speed = 0.2
            self.twist = Twist()
            self.vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
            
            # Image processing setup
            self.bridge = cv_bridge.CvBridge()
            cv2.namedWindow("Turtlebot View", 1)
            cv2.namedWindow("Binary View", 1)
            self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
            


        def prop_control(centroid_x, center_of_view):
            err = centroid_x - center_of_view/2
            self.twist.linear.x = self.linear_speed
            self.twist.angular.z = -float(err) / 100
            self.vel_pub.publish(self.twist)



        def image_callback(self, msg):
            print("In callback")
            
            # grab image and convert to OpenCV message
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # convert to HSV
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # create a mask for finding color of line in image
            mask_low = numpy.array([7, 100, 200])   # ADJUST THESE VALUES
            mask_high = numpy.array([35, 255, 255])
            masked_image = cv2.inRange(hsv, mask_low, mask_high)
            # only focus on center of image
            """
            h, w, d = image.shape
            search_top = 3*h/4
            search_bot = 3*h/4 + 20
            masked_image[0:search_top, 0:w] = 0
            masked_image[search_bot:h, 0:w] = 0
            """
            # determine the centroid of remaining points
            M = cv2.moments(masked_image)
            if M['m00'] > 0:
                centroid_x = int(M['m10']/M['m00'])
                centroid_y = int(M['m01']/M['m00'])
                # draw a red circle around centroid for debugging on original image
                cv2.circle(image, (centroid_x, centroid_y), 20, (0,0,255), -1)
                # publish velocity command
                #prop_control(centroid_x, w)
                print((centroid_x, centroid_y))

            print(type(image))
            print(type(masked_image))
            
            print("About to do imshow 1")
            cv2.imshow("Turtlebot View", image)
            cv2.waitKey(3)
            print("About to do imshow2")
            cv2.imshow("Binary View", masked_image)
            cv2.waitKey(3)
            print("Exiting callback\n")


rospy.init_node('detector')
detector = Line_Detector()
rospy.spin()
