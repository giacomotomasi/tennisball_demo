import urx
import numpy as np


robot_ip = "192.168.56.1"
robot = urx.Robot(robot_ip)

a = 0.3 # acceleration
v = 0.05 # speed

# positions are given as lists
home_pose = [x_home, y_home, z_home, rx_home, ry_home, rz_home] # set home position
robot.movej(home_pose, a, v) # move the robot to home position

########## CAMERA CODE #######
"""
code to get apple position in space


apple_num = 3
apple_pos = np.zeros(3)

for i in apple_num:
    apple_pos[i] = zed_apple_pos
    
"""


##############################





