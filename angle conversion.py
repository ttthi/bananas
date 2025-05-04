
import math
import json

# --- Arm Parameters ---
LENGTHS = [95, 59, 104]  # in mm
BASE_POS = (0, 0)
STEP_DISTANCE_MM = 6.67  # each step, approx. 6.67mm
STEP_UNITS_PER_COLUMN = 6

def forward_kinematics(rel_angles, lengths, base_pos):
    x, y = base_pos
    theta = 0
    positions = []
    for i in range(len(rel_angles)):
        theta += rel_angles[i]
        x += lengths[i] * math.cos(theta)
        y += lengths[i] * math.sin(theta)
        positions.append((x, y))
    return positions

def main():
    columns = [
        [  # Column 1
            [-90, -17, 0, 17],
            [-90, -9, 0, 9]
        ],
        [  # Column 2
            [-90, 6, -82, 76],
            [-90, 27, -88, 60]
        ],
        [  # Column 3
            [-90, 5, -131, 125],
            [-90, 2, 7, -10]
        ]
    ]

    unsorted_data = []

    for col_index, column in enumerate(columns):
        # Calculate backward steps from rightmost (col 3) to leftmost (col 1)
        steps_backward = (2 - col_index) * STEP_UNITS_PER_COLUMN
        step_mm = steps_backward * STEP_DISTANCE_MM

        for floor_index, angles_deg in enumerate(column):
            base_angle_deg, *rel_angles_deg = angles_deg
            rel_angles_rad = [math.radians(a) for a in rel_angles_deg]
            joint_positions = forward_kinematics(rel_angles_rad, LENGTHS, BASE_POS)
            if joint_positions:
                x, y = joint_positions[-1]
                x -= step_mm  # subtract since robot starts from front
                unsorted_data.append({
                    "column": col_index + 1,
                    "floor": floor_index + 1,
                    "steps_backward": steps_backward,
                    "target_xy_mm": [round(x, 2), round(y, 2)],
                    "base_angle": base_angle_deg
                    # "joint_angles_deg": rel_angles_deg
                })

    # Sort by floor first, then column
    sorted_data = sorted(unsorted_data, key=lambda d: (d["floor"], d["column"]))

    print(f"{'Block':<6}{'Col':<6}{'Floor':<7}{'StepsBack':<12}{'Final X':<12}{'Final Y':<12}")
    print("-" * 70)

    for block_num, item in enumerate(sorted_data, 1):
        x, y = item["target_xy_mm"]
        print(f"{block_num:<6}{item['column']:<6}{item['floor']:<7}{item['steps_backward']:<12}{x:<12.2f}{y:<12.2f}")
        item["block"] = block_num

    with open("block_output.json", "w") as json_file:
        json.dump(sorted_data, json_file, indent=2)

if __name__ == "__main__":
    main()
