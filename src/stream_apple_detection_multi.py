#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os
from detection_functions import apple_detection
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

i = 0
apple_detection = apple_detection()

centers_pub = rospy.Publisher('/vision/apples_center', PoseArray, queue_size=10)

class stream_objects():
    def __init__(self):
        rospy.init_node('apple_detection_multi', anonymous=True)
        self.bridge = CvBridge()
        # image subscriber
        rospy.Subscriber("/zed2/zed_node/rgb/image_rect_color", Image, self.img_callback)
        print("Succesfully subscribed to image")

        rospy.loginfo("Detector node initiated.")
        print("Reading the image... \n")
    pass

    def img_callback(self,image_data):
        global rgb_image
        rgb_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        # cv2.imshow("RGB Image",rgb_image)   
        # cv2.waitKey(3)  
        binary_image_mask = apple_detection.filter_color(rgb_image)
        contours = apple_detection.getContours(binary_image_mask)
        c = apple_detection.draw_ball_contour(binary_image_mask, rgb_image, contours)
        centers = PoseArray()
        for n in range(len(c)):
            pose = Pose()
            pose.position.x = c[n][0]
            pose.position.y = c[n][1]
            centers.poses.append(pose)
    
        centers_pub.publish(centers)

    

if __name__ == '__main__':
    try:
        stream = stream_objects()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Detector node terminated.")
        print("Shutting down")
        cv2.destroyAllWindows()
