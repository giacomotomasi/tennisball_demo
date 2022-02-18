
import urx
import logging
import time

if __name__ == "__main__":
    logging.basicConfig(level=logging.WARN)
    rob = urx.Robot("192.168.56.1")

    try:

        # close gripper
        rob.set_digital_out(0, 1)
        time.sleep(0.5)
        rob.set_digital_out(0,0)
        time.sleep(2)
        # open gripper
        rob.set_digital_out(0, 1)
        time.sleep(0.5)
        rob.set_digital_out(0,0)

    finally:
        rob.close()