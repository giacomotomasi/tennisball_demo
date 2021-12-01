#!/usr/bin/env python

from math import atan2
import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import numpy as np

# define a publisher for the apples x,y,z positions
apple_loc_pub = rospy.Publisher('/vision/apple_location_multi', PoseArray, queue_size=10)

def get_offset():
    pass

# function to get the x,y,z offsets to get apple 3D center
def get_xyz_offsets(x,y,z, offset):
        #distance = np.sqrt(x**2 + y**2 + z**2)
        distance = np.linalg.norm(np.array([x,y,z]))
        x = offset*(x/distance)
        y = offset*(y/distance)
        z = offset*(z/distance)
        return x, y, z

def center_callback(center_data):
    global centers
    centers = []
    for c in range(len(center_data.poses)):
        centers.append([center_data.poses[c].position.x, center_data.poses[c].position.y])


def point_cloud_callback(point_cloud_data):
    apple_loc = PoseArray()
    apple_loc.header = point_cloud_data.header
    gen = pc2.read_points(point_cloud_data, skip_nans=False, field_names=("x","y","z"))
    #int_data = np.array(list(gen))
    int_data = list(gen)
    #print(len(int_data))

    for n in range(len(centers)):
        cx = int(centers[n][0])
        cy = int(centers[n][1])


        #print("cx:", cx, "cy:", cy)
        pixel_center = cx+(cy*point_cloud_data.width) # formula to get pixel center from matrix to array
        #pixel_centers.append(pixel_c) # list with apple centers pixel expressed in 1D array
        offset = 0.03 # offset of 3 cm to get the 3d apple center
        x_offset, y_offset, z_offset = get_xyz_offsets(int_data[pixel_center][0], int_data[pixel_center][1], int_data[pixel_center][2], offset)
        print("offset:", np.sqrt(x_offset**2 + y_offset**2 + z_offset**2))
        apple_pose = Pose()
        apple_pose.position.x = int_data[pixel_center][0] + x_offset # depth value of apple center (if needed apply depth compensation here)
        apple_pose.position.y = int_data[pixel_center][1] + y_offset
        apple_pose.position.z = int_data[pixel_center][2] + z_offset
        apple_loc.poses.append(apple_pose)
        #print("apple pose camera frame:", apple_pose)

    print("Apple location:", apple_loc)
    apple_loc_pub.publish(apple_loc)

    


        

if __name__ == '__main__':
    try:
        rospy.init_node('get_apple_location_multi', anonymous=True)
        # #depth subscriber
        rospy.Subscriber("/zed2/zed_node/point_cloud/cloud_registered", PointCloud2, point_cloud_callback)
        print("Succesfully subscribed to PSoint Cloud")
        print("Reading the Point Cloud... \n")
        # center subscriber
        rospy.Subscriber("/vision/apples_center", PoseArray, center_callback)
        print("Succesfully subscribed to center")
        rospy.loginfo("Detector node initiated.")

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Detector node terminated.")
        print("Shutting down")
        cv2.destroyAllWindows()