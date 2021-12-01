import urx
import logging
import time

if __name__ == "__main__":
    logging.basicConfig(level=logging.WARN)

    rob = urx.Robot("192.168.56.1")
    #rob = urx.Robot("localhost")
    rob.set_tcp((0,0,0,0,0,0))
    rob.set_payload(0.5, (0,0,0))
    try:
        l = 0.05
        v = 0.05
        a = 0.3
        pose = rob.getl() #gives a lists with 6 elements (x, y, z, rx, ry, rz) --> rotation vector
        print("robot tcp is at: ", pose)
        print("absolute move in base coordinate ")
        pose[2] += l
        rob.movel(pose, acc=a, vel=v)
        time.sleep(1)
        pose[2] -= l
        rob.movel(pose, acc=a, vel=v)
        pose_final = rob.getl()
        print("robot tcp is at (final): ", pose_final)

    finally:
        rob.close()
