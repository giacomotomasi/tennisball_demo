
import urx
import logging
import time

if __name__ == "__main__":
    logging.basicConfig(level=logging.WARN)

    #gripper_remove_pos = [0.0755, -0.2824, 0.3477, -0.0387, -3.0754, 0.4400] # rest position (good to place/remove gripper)

    rob = urx.Robot("192.168.56.1")

    #rob.set_tcp((0,0,0,0,0,0))
    #rob.set_payload(0.5, (0,0,0))

    home_pos = [-0.0153, -0.4213, 0.3469, 1.2430, 2.6540, -0.9590]
    appro1 = [-0.0762, -0.5575, 0.3546, 0.6110, 2.7090, -1.7840]
    apple1 = [-0.1042, -0.6244, 0.3209, 1.4510, 1.9160, -1.4980]
    get_far1 = [-0.0510, -0.5086, 0.3215, 0.4900, 2.6510, -1.8690]
    appro2 = [-0.1767, -0.4281, 0.3204, 1.8210, 2.0030, -1.5280]
    apple2 = [-0.2129, -0.4926, 0.2951, 1.8210, 2.0030, -1.5280]
    get_far2 = [-0.1324, -0.3790, 0.3112, 1.8210, 2.0030, -1.5280]
    appro_place = [0.3571, -0.3540, 0.3563, 1.2360, 2.8850, -0.0780]
    place_pos = [0.3571, -0.3540, 0.2983, 1.2360, 2.8850, -0.0780]


    try:
        v = 0.2
        a = 0.3
        rob.set_digital_out(0,0) # initialize gripper
        # open gripper
        rob.set_digital_out(0, 1)
        time.sleep(0.5)
        rob.set_digital_out(0,0)

        pose = rob.getl() #gives a lists with 6 elements (x, y, z, rx, ry, rz) --> rotation vector
        #print("robot tcp is at: ", pose)
     
        # move to home position
        #rob.movej(joint_pose, acc=a, vel=v) # it takes as inputs the joints goal values!
        rob.movej_to_pose(home_pos, acc=a, vel=0.3)
        time.sleep(0.01)

        # move towards the first apple to pick (approach it, move to a suitable grabbing position, get away)
        rob.movej_to_pose(appro1, acc=a, vel=v)
        time.sleep(0.01)
        rob.movel(apple1, acc=a, vel=v)
        # close gripper
        rob.set_digital_out(0, 1)
        time.sleep(0.5)
        rob.set_digital_out(0,0)

        time.sleep(1)
        rob.movel(get_far1, a, v)

        #move towards the place position
        rob.movej_to_pose(appro_place, a, vel=0.3)
        time.sleep(0.01)
        rob.movel(place_pos, a, v)
        # open gripper
        rob.set_digital_out(0, 1)
        time.sleep(0.5)
        rob.set_digital_out(0,0)

        time.sleep(1)
        rob.movel(appro_place, a, v)

        # move to home position
        rob.movej_to_pose(home_pos, a, v)

        pose_final = rob.getl()
        print("robot tcp is at (final): ", pose_final)

    finally:
        rob.close()