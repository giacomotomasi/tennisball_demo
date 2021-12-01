# Library urx from https://github.com/Mofeywalker/python-urx

import urx
#print(urx.__file__) # get path of python libreries

from urx.gripper import OnRobotGripperRG2

import time
import math
from math import pi
from math import sqrt

import numpy as np
import math3d as m3d

rob = urx.Robot("192.168.56.1")
gripper = OnRobotGripperRG2(rob)

#rob.set_tcp((0, 0, 0, 0, 0, 0))
#rob.set_payload(0, (0, 0, 0))

time.sleep(0.2)  #leave some time to robot to process the setup commands

print("Opening gripper...")
gripper.open_gripper(target_width=50, target_force=10)
#gripper.rg_grip(50, 10)

time.sleep(2.0)

print("Closing gripper...")
gripper.close_gripper(target_width=10, target_force=10)
#gripper.rg_grip(50, 10)

time.sleep(2.0)

print("Closing...")
rob.close()



#gripper.open_gripper(
#    target_width=110,  # Width in mm, 110 is fully open
#    target_force=40,  # Maximum force applied in N, 40 is maximum
#    payload=0.5,  # Payload in kg
#    set_payload=False,  # If any payload is attached
#    depth_compensation=False,  # Whether to compensate for finger depth
#    slave=False,  # Is this gripper the master or slave gripper?
#    wait=2  # Wait up to 2s for movement
#)
