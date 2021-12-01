import urx
import logging
import time

home_pos = [0.0755, -0.2824, 0.3477, -0.0387, -3.0754, 0.4400] # rest position (good to place/remove gripper)

if __name__ == "__main__":
    logging.basicConfig(level=logging.WARN)

    rob = urx.Robot("192.168.56.1")
    rob.set_tcp((0,0,0,0,0,0))
    #rob.set_payload(0.5, (0,0,0))
    try:
        l = 0.05
        v = 0.05
        a = 0.3

        rob.movel(home_pos, acc=a, vel=v)
        time.sleep(1)

        pose_final = rob.getl()
        print("robot tcp is at (final): ", pose_final)

    finally:
        rob.close()