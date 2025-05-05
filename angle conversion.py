import math
import json
from IKsolver import forward_kinematics

# Idea is that front to back there are 3 arm positions for each floor, and robot moves left to right on it's trachs
# with the arm being stationary
# for each floor the angles need to be recorded experimentally or with software

# --- Arm Parameters ---
LENGTHS = [95, 59, 104]  # in mm
BASE_POS = (0, 0)
STEP_DISTANCE_MM = 6.67  # each step
STEP_UNITS_PER_ROW = 6
COLUMNS = 3
ROWS = 3
FLOORS = 2

def main():
    base_angles = [
        [-90, -17, 0, 17],  # Floor 1, Row 1 (Far)
        [-90, 6, -82, 76],  # Floor 1, Row 2 (Mid)
        [-90, 5, -131, 125],  # Floor 1, Row 3 (Near)

        [-90, -9, 0, 9],  # Floor 2, Row 1 (Far)
        [-90, 27, -88, 60],  # Floor 2, Row 2 (Mid)
        [-90, 2, 7, -10],  # Floor 2, Row 3 (Near)
    ]

    blocks = []
    block_num = 1

    print(f"{'Block':<6}{'Col':<6}{'Row':<6}{'Floor':<7}{'StepsBack':<12}{'Target X':<12}{'Target Y':<12}")
    print("-" * 72)

    index = 0

    for floor in range(1, FLOORS + 1):
        for angle_set in base_angles[index:index + COLUMNS]:
            base_angle_deg, *rel_angles_deg = angle_set
            rel_angles_rad = [math.radians(a) for a in rel_angles_deg]
            joint_positions = forward_kinematics(rel_angles_rad, LENGTHS, BASE_POS)
            x, y = joint_positions[-1]
            for row in range(ROWS):
                steps_back = (2 - row) * STEP_UNITS_PER_ROW
                blocks.append({
                    "block": block_num,
                    "column": (block_num - 1) % COLUMNS + 1,
                    "row": row + 1,
                    "floor": floor,
                    "steps_backward": steps_back,
                    "target_xy": [round(x, 2), round(y, 2)],
                    "base_angle": base_angle_deg
                })
                print(f"{block_num:<6}{(block_num - 1) % COLUMNS + 1:<6}{row + 1:<6}{floor:<7}"
                      f"{steps_back:<12}{x:<12.2f}{y:<12.2f}")
                block_num += 1
        index += COLUMNS

    with open("block_output_computed.json", "w") as f:
        json.dump(blocks, f, indent=2)

if __name__ == "__main__":
    main()