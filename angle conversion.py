
import math
import json

#TODO: FIX NOT WORKING CORRECTLY

# --- Arm Parameters ---
LENGTHS = [95, 59, 104]  # in mm
BASE_POS = (0, 0)
STEP_DISTANCE_MM = 6.67  # each step
STEP_UNITS_PER_ROW = 6

def forward_kinematics(rel_angles, lengths, base_pos):
    x, y = base_pos
    theta = 0
    for angle, length in zip(rel_angles, lengths):
        theta += angle
        x += length * math.cos(theta)
        y += length * math.sin(theta)
    return round(x, 2), round(y, 2)

def main():
    # Angles for each block placement
    base_angles = [
        [-90, -17, 0, 17],
        [-90, 6, -82, 76],
        [-90, 5, -131, 125],
        [-90, -9, 0, 9],
        [-90, 27, -88, 60],
        [-90, 2, 7, -10],
    ]

    blocks = []
    block_num = 1
    index = 0

    print(f"{'Block':<6}{'Col':<6}{'Row':<6}{'Floor':<7}{'StepsBack':<12}{'Target X':<10}{'Target Y':<10}")
    print("-" * 60)

    for floor in range(1, 2):  # 1 floor for now
        for col_index in range(3):  # columns: left to right
            for row_index in range(3): # rows: far to near (backward steps)
                steps_backward = (2 - row_index) * STEP_UNITS_PER_ROW
                if index >= len(base_angles):
                    break
                base_angle_deg, *rel_angles_deg = base_angles[index]
                rel_angles_rad = [math.radians(a) for a in rel_angles_deg]
                x, y = forward_kinematics(rel_angles_rad, LENGTHS, BASE_POS)
                x += col_index * 10  # simulate column spacing

                blocks.append({
                    "block": block_num,
                    "column": col_index + 1,
                    "row": row_index + 1,
                    "floor": floor,
                    "steps_backward": steps_backward,
                    "target_xy": [x, y],
                    "base_angle": base_angle_deg
                })
                print(
                f"{block_num:<6}{col_index + 1:<6}{row_index + 1:<6}{floor:<7}{steps_backward:<12}{x:<10.2f}{y:<10.2f}")

                block_num += 1
                index += 1

    with open("block_output.json", "w") as f:
        json.dump(blocks, f, indent=2)

if __name__ == "__main__":
    main()
