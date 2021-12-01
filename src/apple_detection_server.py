#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
import numpy as np
from detection_functions import apple_detection
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import sensor_msgs.point_cloud2 as pc2
from tennisball_demo.srv import apple_srv_msg2, apple_srv_msg2Response


rgb_image = np.zeros((720,1280,3)) # initialize imgage variable
apple_det = apple_detection()
bridge = CvBridge()


# function to get the x,y,z offsets to get apple 3D center
def get_xyz_offsets(x,y,z, offset):
        #distance = np.sqrt(x**2 + y**2 + z**2)
        distance = np.linalg.norm(np.array([x,y,z]))
        x = offset*(x/distance)
        y = offset*(y/distance)
        z = offset*(z/distance)
        return x, y, z

def get_center_avg(pixel, int_data):
    pass

def apple_detection_service(data):

    ########################## GETTING APPLES CENTERS ##########################
    rgb_image = bridge.imgmsg_to_cv2(data.image, "bgr8")
    binary_image_mask = apple_det.filter_color(rgb_image)
    contours = apple_det.getContours(binary_image_mask)
    centers = apple_det.draw_ball_contour(binary_image_mask, rgb_image, contours)

    ########################## GETTING APPLES LOCATION ##########################
    apple_loc = PoseArray()
    apple_loc.header = data.point_cloud.header
    gen = pc2.read_points(data.point_cloud, skip_nans=False, field_names=("x","y","z"))
    int_data = list(gen)
    detected_objects = len(centers)
    rejected_objects = 0
    print("Apples center pixels:")
    for n in range(len(centers)):
        cx = int(centers[n][0])
        cy = int(centers[n][1])
        print("cx:", cx, "cy:", cy)
        pixel_center = cx+(cy*data.point_cloud.width) # formula to get pixel center from matrix to array
        #pixel_centers.append(pixel_c) # list with apple centers pixel expressed in 1D array
        offset = 0.03 # offset of 3 cm to get the 3d apple center
        x_offset, y_offset, z_offset = get_xyz_offsets(int_data[pixel_center][0], int_data[pixel_center][1], int_data[pixel_center][2], offset)
        # x_offset = 0
        # y_offset = 0
        # z_offset = 0
        apple_pose = Pose()
        apple_pose.position.x = int_data[pixel_center][0] + x_offset # depth value of apple center (if needed apply depth compensation here)
        apple_pose.position.y = int_data[pixel_center][1] + y_offset
        apple_pose.position.z = int_data[pixel_center][2] + z_offset
        position_array = np.array([apple_pose.position.x, apple_pose.position.y, apple_pose.position.z]) # needed since np.isnan() doesn't support Pose input type
        check = np.isnan(position_array).any() # returms True if at least one coordinate is nan
        if check:
            rejected_objects += 1
        else:
            apple_loc.poses.append(apple_pose)

    # print("Detected objects:", detected_objects)
    # print("Rejected objects:", rejected_objects)
    return apple_srv_msg2Response(apple_loc, detected_objects, rejected_objects)


if __name__ == '__main__':
    try:
        rospy.init_node('apple_detection_service', anonymous=True)
        rospy.loginfo("Apple detection Server ready...")

        apple_service = rospy.Service('apple_service', apple_srv_msg2, apple_detection_service)


        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Server node terminated.")
        print("Shutting down")
        cv2.destroyAllWindows()
