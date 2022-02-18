
import urx
from IPython import embed
import logging

if __name__ == "__main__":
    try:
        logging.basicConfig(level=logging.INFO)
        robot = urx.Robot("192.168.56.1")
        r = robot
        print("Robot object is available as robot or r")
        embed()
    finally:
        robot.close()
