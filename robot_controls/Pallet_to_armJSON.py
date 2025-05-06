import json

STEP_UNITS_PER_ROW = 6
STEP_DISTANCE_MM = 6.67  # robot step size in mm
BASE_X = 251.27  # arm X position over row 1 (near robot)
BASE_Y = -45.03  # arm Y position at floor 1
BASE_ANGLE = -90

def main():
    with open("configurations.json") as f:
        data = json.load(f)

    config = data["configurations"][0]
    block_dims = data["block_dimensions"]
    clearance = data["clearance"]

    spacing_x = block_dims[0] + clearance[0]  # row spacing
    spacing_y = block_dims[1] + clearance[1]  # column spacing
    spacing_z = block_dims[2] + clearance[2]  # floor spacing

    min_x = min(pos[0] for _, pos in config)
    min_y = min(pos[1] for _, pos in config)
    min_z = min(pos[2] for _, pos in config)

    output_blocks = []

    for i, (label, pos) in enumerate(config):
        # Convert cm → mm
        x_mm = pos[0] * 10
        y_mm = pos[1] * 10
        z_mm = pos[2] * 10

        row = round((x_mm - min_x * 10) / (spacing_x * 10)) + 1
        column = round((y_mm - min_y * 10) / (spacing_y * 10)) + 1
        floor = round((z_mm - min_z * 10) / (spacing_z * 10)) + 1

        steps_backward = round((x_mm - min_x * 10) / (spacing_x * 10)) * STEP_UNITS_PER_ROW

        delta_y = y_mm - min_y * 10
        delta_z = z_mm - min_z * 10

        target_x = BASE_X - delta_y
        target_y = BASE_Y + delta_z

        output_blocks.append({
            "block": i + 1,
            "column": column,
            "row": row,
            "floor": floor,
            "steps_backward": steps_backward,
            "target_xy": [round(target_x, 2), round(target_y, 2)],
            "base_angle": BASE_ANGLE
        })

        print(f"{i+1:<5}{column:<8}{row:<6}{floor:<7}{steps_backward:<10}{round(target_x,2):<10}{round(target_y,2):<10}")

    with open("block_output_computed.json", "w") as f:
        json.dump(output_blocks, f, indent=2)

if __name__ == "__main__":
    main()
