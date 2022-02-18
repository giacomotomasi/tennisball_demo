
import urx
import logging
import time

if __name__ == "__main__":
    logging.basicConfig(level=logging.WARN)


    # home_pos = [0.0755, -0.2824, 0.3477, -0.0387, -3.0754, 0.4400] # rest position (good to place/remove gripper)
    rob = urx.Robot("192.168.56.1")
    #rob = urx.Robot("localhost")
    rob.set_tcp((0,0,0,0,0,0))
    #rob.set_payload(0.5, (0,0,0))
    try:
        l = 0.05
        v = 0.05
        a = 0.3
        pose = rob.getl() #gives a lists with 6 elements (x, y, z, rx, ry, rz) --> rotation vector
        print("robot tcp is at: ", pose)
     
        pose[2] += l
        #rob.movej(pose, acc=a, vel=v) # moves each joint given joint goal pos
        rob.movej_to_pose(pose, acc=a, vel=v) # move each joint to reach position goal of tool
        time.sleep(1)
        pose[2] -= l
        #rob.movej(pose, acc=a, vel=v)
        rob.movej_to_pose(pose, acc=a, vel=v)
        pose_final = rob.getl()
        print("robot tcp is at (final): ", pose_final)

    finally:
        rob.close()