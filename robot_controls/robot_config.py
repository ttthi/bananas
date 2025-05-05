import math

# Params for the arm
STABILIZE_LAST = True
LENGTHS = [95, 59, 104]
REL_ANGLES = [0, 0, 0]
NUM_JOINTS = len(REL_ANGLES)
BASE_ANGLE = 0
# Limits for the arm
MIN_ANGLES = [-math.pi / 2, -5 * math.pi / 6, -5 * math.pi / 6]
MAX_ANGLES = [3 * math.pi / 2, 5 * math.pi / 6, 5 * math.pi / 6]
# Parms for the kinematic algorithm
MAX_ITER = 10
DST_THRESHOLD = 0.01
BASE_POS = (0, 0)