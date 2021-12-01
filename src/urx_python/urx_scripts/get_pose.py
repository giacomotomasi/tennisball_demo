import urx

robot = urx.Robot("192.168.56.1")

# get current pose, transform it and move robot to new pose
trans = robot.get_pose()  # get current transformation matrix (tool to base)
position = robot.get_pos()  # get current position vector (x, y, z)
orient = robot.get_orientation() # get current orientatio matrix

robot.set_tcp((0,0,0,0,0,0)) # set the offset of the gripper wrt robot flange
robot.set_payload(0.5, (0,0,0))

# print("Transformation from base to tcp is: ", trans)
# print("Position: ", position)
# print("Orientation: ", orient)
current_pose = robot.getl()
print("Current tool pose is: ",  current_pose) # gives a vector with 6 elements (x, y, z, rx, ry, rz) --> rotation vector
print("current_pose variable type:", type(current_pose))


robot.close()
