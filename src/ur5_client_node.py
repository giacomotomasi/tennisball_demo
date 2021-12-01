#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
from tennisball_demo.srv import apple_srv_msg2, apple_srv_msg2Request
import time
import numpy as np
import urx
import logging

class ur5_client():
    def __init__(self):
        rospy.init_node('ur5_client_node', anonymous=True)
        # point cloud subscriber
        rospy.Subscriber("/zed2/zed_node/point_cloud/cloud_registered", PointCloud2, self.point_cloud_callback)
        print("Succesfully subscribed to Point Cloud!")
        # image subscriber
        rospy.Subscriber("/zed2/zed_node/rgb/image_rect_color", Image, self.img_callback)
        print("Succesfully subscribed to image!")

        self.rgb_image = Image()
        self.point_cloud = PointCloud2()


    def img_callback(self,image_data):
        self.rgb_image = image_data
        #print(rgb_image.encoding)

    def point_cloud_callback(self,point_cloud_data):
        self.point_cloud = point_cloud_data

    def apple_detection_client(self,rgb_image, point_cloud, n_attempt):
        print("Waiting for the Server...")
        rospy.wait_for_service('apple_service')

        try:
            print("Sending server request...")
            print("Attempt:", n_attempt)
            apple_srv = rospy.ServiceProxy('apple_service', apple_srv_msg2)
            data_from_camera = apple_srv_msg2Request(rgb_image, point_cloud)
            service_resp = apple_srv(data_from_camera)

            #service_call = apple_srv(rgb_image, point_cloud)
            
            return service_resp.apples_pose, service_resp.detected_objects, service_resp.rejected_objects

        except rospy.ServiceException as e:
            print("Service call failed")

    def camera2robot_frame(self, apples):
        arm_apples = PoseArray()
        # left camera frame origin wrt robot frame (in meters)
        xc = 326.73/1000 # from [mm] to [m]
        yc = -270.01/1000
        zc = 150/1000
        # rotation matrix of camera frame wrt robot frame
        theta = np.deg2rad(225)
        R01 = np.array([[np.cos(theta), -np.sin(theta), 0],
                        [np.sin(theta), np.cos(theta), 0],
                        [0, 0, 1]])
        # homogeneous transformation matrix
        A01 = np.array([[R01[0,0], R01[0,1], R01[0,2], xc],
                        [R01[1,0], R01[1,1], R01[1,2], yc],
                        [R01[2,0], R01[2,1], R01[2,2], zc],
                        [0, 0, 0, 1]])

        ###### TRANFORMATION ##########
        arm_apples.header = apples.header
        for idx in range(len(apples.poses)):
            x = apples.poses[idx].position.x # in meters
            y = apples.poses[idx].position.y
            z = apples.poses[idx].position.z
            apple_camera_coordinates = np.array([x,y,z, 1])
            # coordinates transformation
            apple_loc_ = np.dot(A01, apple_camera_coordinates)
            new_pose = Pose()
            new_pose.position.x = apple_loc_[0]
            new_pose.position.y = apple_loc_[1]
            new_pose.position.z = apple_loc_[2]
            arm_apples.poses.append(new_pose)

        return arm_apples

class ur5_robot():
    def __init__(self):
        logging.basicConfig(level=logging.WARN)
        self.rob = urx.Robot("192.168.56.1")
        self.rob.set_tcp((0,0,0.18,0,0,0)) # set tcp offset im meters (18 cm along z axis)
        # self.rob.set_payload(0.5, (0,0,0))
        self.home_pos = [0.0591, -0.3031, 0.3559, 3.14, 0, 0]
        self.place_pos = [0.46915, -0.36037, 0.15146, 2.682, 1.643, 0] # orientation set in order not to blend the air cable
        self.v = 0.35
        self.a = 0.5
        pass

    def get_ur_info(self):
        # get info
        joints = self.rob.getj()
        position = self.rob.get_pos()  # get current position vector (x, y, z)
        trans = self.rob.get_pose()  # get current transformation matrix (tool to base)
        orient = self.rob.get_orientation() # get current orientation matrix

        # display robot information
        print("Joint positions (rad):", joints)
        print("----------------------------")
        print("Joint positions (deg):", np.rad2deg(joints))
        print("----------------------------")
        print("Position vector (x, y, z):", position)
        print("----------------------------")
        print("Orientation matrix:", orient)
        print("----------------------------")
        print("Transformation matrix (tool to base):", trans)

        return print("Information displayed!")
    
    def check_apples_pos(self, apple_pos):
        rx = 3.14
        ry = 0
        rz = 0
        h = 0.07
        for i in range(len(apple_pos.poses)):
            #rob.movej(pose, acc=a, vel=v) # moves each joint given joint goal pos
            print("Robot moving to home position...")
            self.rob.movej_to_pose(self.home_pos, acc=self.a, vel=self.v) # move each joint to reach position goal of tool
            print("Done!")
            print("----------------------------")
            time.sleep(1)
            print("Robot moving to apple",i+1,"...")
            approaching_pick_pos = [apple_pos.poses[i].position.x,apple_pos.poses[i].position.y,apple_pos.poses[i].position.z+h, rx,ry,rz] # approaching position translated of 7 cm along z axis
            self.rob.movej_to_pose(approaching_pick_pos, acc=self.a, vel=self.v)
            print("Done!")
            print("----------------------------")
            time.sleep(1)


    def move_arm(self, apple_pos):
        rx = 3.14
        ry = 0
        rz = 0
        h = 0.07
        pose = self.rob.getl() #gives a lists with 6 elements (x, y, z, rx, ry, rz) --> rotation vector
        print("robot tcp is at: ", pose)
        print("----------------------------")
        # self.rob.set_digital_out(0,0) # initialize gripper
        # time.sleep(1.5)

        for i in range(len(apple_pos.poses)):
            #rob.movej(pose, acc=a, vel=v) # moves each joint given joint goal pos
            print("Robot moving to home position...")
            self.rob.movej_to_pose(self.home_pos, acc=self.a, vel=self.v) # move each joint to reach position goal of tool
            print("Done!")
            print("----------------------------")
            time.sleep(1)
            
            # open gripper
            #self.gripper()
            print("Robot moving to apple",i+1,"...")
            goal_pose = [apple_pos.poses[i].position.x,apple_pos.poses[i].position.y,apple_pos.poses[i].position.z, rx,ry,rz]
            approaching_pick_pos = [apple_pos.poses[i].position.x,apple_pos.poses[i].position.y,apple_pos.poses[i].position.z+h, rx,ry,rz] # approaching position translated of 7 cm along z axis
            #approaching_pos = goal_pose
            #approaching_pos[2] += 0.07 # approaching position translated of 7 cm along z axis 
            self.rob.movej_to_pose(approaching_pick_pos, acc=self.a, vel=self.v)
            time.sleep(0.5)
            self.rob.movel(goal_pose, acc=self.a, vel=self.v)
            self.gripper() # close gripper
            time.sleep(0.5)
            self.rob.movel(approaching_pick_pos, acc=self.a, vel=self.v)
            print("Done!")
            print("----------------------------")
            time.sleep(0.5)
            approaching_place_pos = [0.46915, -0.36037, 0.15146+h, 2.682, 1.643, 0] # place_pos with z offset
            #self.rob.movej_to_pose(self.home_pos, acc=self.a, vel=self.v) # kinda waypoint in this case
            self.rob.movej_to_pose(approaching_place_pos, acc=self.a, vel=self.v)
            #self.rob.movel(approaching_place_pos, acc=self.a, vel=self.v)
            time.sleep(0.5)
            self.rob.movel(self.place_pos, acc=self.a, vel=self.v)
            self.gripper() # open gripper
            time.sleep(1)
            self.rob.movel(approaching_place_pos, acc=self.a, vel=self.v)
            #print(goal_pose)
            print("Done!")
            print("----------------------------")
            time.sleep(1)
        # returning to home position after pick and place is completed
        print("Robot moving to home position...")
        self.rob.movej_to_pose(self.home_pos, acc=self.a, vel=self.v) # move each joint to reach position goal of tool
        print("Done!")
        print("----------------------------")
        time.sleep(0.5)
    
        print("Motion ended!")

        pose_final = self.rob.getl()
        print("robot tcp is at final pose: ", pose_final)
    
    def gripper(self):
        # this sequence of commands opens/closes the gripper
        # the same command is used for opening and closing, no gripper state memory
        self.rob.set_digital_out(0, 1)
        time.sleep(0.5)
        self.rob.set_digital_out(0,0)


if __name__ == '__main__':
    try:
        
        ur5_cli = ur5_client()
        t_init = time.time()
        ur5 = ur5_robot()
        
        time.sleep(0.6)
        if time.time() - t_init > 0.5:

            # try 5 times to locate all apples
            attempt = 1
            while attempt < 6: # it starts from 1 not 0
                detected_apples, detected_objects, rejected_objects = ur5_cli.apple_detection_client(ur5_cli.rgb_image, ur5_cli.point_cloud, attempt)
                if rejected_objects == 0:
                    print("All apples have been localized!")
                    break
                attempt += 1
            
            '''print camera frame apples coordinates'''
            # print("Server response (coordinates in camera frame):")
            # print(detected_apples)

            # find apples coordinates wrt robot frame
            detected_apples_arm = ur5_cli.camera2robot_frame(detected_apples)
            print("Apples position (coordinates in robot frame):")
            print("Detected objects:", detected_objects)
            print("Rejected objects:", rejected_objects)
            if detected_objects != rejected_objects:
                print(detected_apples_arm)
            else:
                print("Nunmber of iterations exceeded, no object has been localized!")

            time.sleep(1)
            #ur5.get_ur_info()
            ur5.check_apples_pos(detected_apples_arm)
            #ur5.move_arm(detected_apples_arm)
            ur5.rob.close()
        #rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Server node terminated.")
        print("Shutting down")
        cv2.destroyAllWindows()
