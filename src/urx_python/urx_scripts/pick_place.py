import urx
import logging
from urx.gripper import OnRobotGripperRG2

import time
import math
from math import pi
from math import sqrt
import numpy as np

import math3d as m3d

if __name__ == "__main__":
    logging.basicConfig(level=logging.WARN)

    rob = urx.Robot("192.168.56.1")
    gripper = OnRobotGripperRG2(rob)
    #rob = urx.Robot("localhost")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))

    z_offeset = 0.21
    h = 0.05 # approach distance

    home_pos = [0.0755, -0.2824, 0.3477, -0.0387, -3.0754, 0.4400]
    time.sleep(0.2)
    gripper.rg_grip(42, 10) # open the gripper
    pick_pos = [-0.0168, -0.4394, 0.02+z_offeset,  0.2061, 3.1221, -0.0263]
    waypoint = [-0.0160, -0.4394, 0.02+h+z_offeset,  0.2061, 3.1221, -0.0263]
    place_pos = [0.3836, -0.3631, 0.0914+z_offeset, -1.1683, 2.8908, -0.0509]
    

    try:
        v = 0.1
        a = 0.3

        rob.movel(home_pos, acc=a, vel=v) #gives a lists with 6 elements (x, y, z, rx, ry, rz) --> rotation vector

        rob.movel(waypoint, acc=a, vel=v)
        time.sleep(0.5)
        rob.movel(pick_pos, acc=a, vel=v)
        time.sleep(0.5)
        gripper.rg_grip(37, 10) # close gripper
        time.sleep(0.5)
        rob.movel(waypoint, acc=a, vel=v)
        time.sleep(0.5)
        rob.movel(place_pos, acc=a, vel=v)
        time.sleep(0.5)
        gripper.rg_grip(42, 10)
        time.sleep(0.5)
        rob.movel(home_pos, acc=a, vel=v)
        #time.sleep(1)

        pose_final = rob.getl()
        print("robot tcp is at (final): ", pose_final)

    finally:
        rob.close()