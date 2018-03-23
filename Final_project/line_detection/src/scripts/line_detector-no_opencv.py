#!/usr/bin/env python

# This script uses the cv_bridge package to do the following:
#   1) convert images from sensor_msgs/Image topic to OpenCV messages
#   2) convert their colors from RGB to HSV
#   3) apply a threshold of hues NEAR THE COLOR YELLOW to obtain a binary image.
#
# The script uses an approach called proportional and simple means.
#
# Code adapted from: edu.gaitech.hk/turtlebot/line-follower.html


import rospy 
import numpy as np
import ros_numpy as rn
from scipy import stats
from PIL import Image as pImage
from skimage import color
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Line_Detector:
        def __init__(self):
            # Motion setup
            self.linear_speed = 0.2
            self.twist = Twist()
            self.vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
            
            # Image processing setup
            self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
            self.image_pub = rospy.Publisher('line_detection', Image, queue_size=1)

        def prop_control(centroid_x, center_of_view):
            err = centroid_x - center_of_view/2
            self.twist.linear.x = self.linear_speed
            self.twist.angular.z = -float(err) / 100
            self.vel_pub.publish(self.twist)

        def display_image(numpy_array):
            pass
            # plt.imshow(numpy_array)
            # display.display(plt.gcf())
            # display.clear_output(wait=True)
            # time.sleep(1)
            # fig,ax = subplots(1,1)
            # im = ax.imshow(binary_image, cmap=cm.hot, animated=True)
            # fig.canvas.draw()
            # background = fig.canvas.copy_from_bbox(ax.bbox)

        def image_callback(self, msg):
            
            # Convert Image message to HSV numpy array
            rgb_image = rn.numpify(msg)
            hsv_image = color.rgb2hsv(rgb_image)
            
            # Threshold between values for H, S, V
            low_H = 0.18
            high_H = 0.19
            low_S = 0.3
            high_S = 0.4
            low_V = 0.3
            high_V = 0.5
            
            H_mask = stats.threshold(hsv_image[:,:,0], threshmin=low_H, threshmax=high_H, newval=0)
            H_mask[np.nonzero(H_mask)] = 1

            S_mask = stats.threshold(hsv_image[:,:,1], threshmin=low_S, threshmax=high_S, newval=0)
            S_mask[np.nonzero(S_mask)] = 1

            V_mask = stats.threshold(hsv_image[:,:,2], threshmin=low_V, threshmax=high_V, newval=0)
            V_mask[np.nonzero(V_mask)] = 1
            
            binary_image = np.multiply( H_mask, np.multiply( S_mask, V_mask ))
            
            print(np.sum(binary_image))
            
            # convert back to ros img message and publish
            temp = np.multiply(binary_image, 255)
            bin_img_3d = np.dstack((temp, temp, temp))

            # @TODO: FIX THIS PUBLISHING FIASCO
            self.image_pub.publish(rn.msgify(Image, bin_img_3d))
            
            
            """
            h, w, d = image.shape
            search_top = 3*h/4
            search_bot = 3*h/4 + 20
            masked_image[0:search_top, 0:w] = 0
            masked_image[search_bot:h, 0:w] = 0
            ""
            # determine the centroid of remaining points
            M = cv2.moments(masked_image)
            if M['m00'] > 0:
                centroid_x = int(M['m10']/M['m00'])
                centroid_y = int(M['m01']/M['m00'])
                # draw a red circle around centroid for debugging on original image
                cv2.circle(image, (centroid_x, centroid_y), 20, (0,0,255), -1)
                # publish velocity command
                #prop_control(centroid_x, w)

            cv2.imshow("Turtlebot View", image)
            cv2.imshow("Binary View", masked_image)
            cv2.waitKey(3)
            """

rospy.init_node('detector')
detector = Line_Detector()
rospy.spin()
