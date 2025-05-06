# Launcher script
import os
import sys
sys.path.append(os.path.expanduser("~/jetbot")) # Enables import from jetbot

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