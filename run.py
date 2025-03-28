# Launcher script
import time
from BananasRobotMain import BananasRobot

r = BananasRobot()

# Keep it alive until keyboard interrupt (CTRL+C or CTRL+D)
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    r.exit()
    print("Stopped the robot")