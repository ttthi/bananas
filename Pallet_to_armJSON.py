import json

COLUMNS = 3
ROWS = 3
FLOORS = 3
STEP_UNITS_PER_COLUMN = 5
STEP_DISTANCE_MM = 6.67
ROW_SPACING_MM = 41
COLUMN_SPACING = 41
FLOOR_SPACING_MM = 38

# Fixed base target for top-left (col=1, row=1, floor=1)
BASE_X = 251.27
BASE_Y = -45.03
BASE_ANGLE = -90

def main():
    blocks = []
    block_num = 1
    print(f"{'Block':<6}{'Col':<6}{'Row':<6}{'Floor':<7}{'StepsBack':<12}{'Target X':<12}{'Target Y':<12}")
    print("-" * 72)

    for floor in range(1, FLOORS + 1):
        for row in range(1, ROWS + 1):

            for col in range(1, COLUMNS + 1):
                steps_back = (COLUMNS - col) * STEP_UNITS_PER_COLUMN
                x = BASE_X - (row - 1) * ROW_SPACING_MM
                y = BASE_Y + (floor - 1) * FLOOR_SPACING_MM

                blocks.append({
                    "block": block_num,
                    "column": col,
                    "row": row,
                    "floor": floor,
                    "steps_backward": steps_back,
                    "target_xy": [round(x, 2), round(y, 2)],
                    "base_angle": BASE_ANGLE
                })

                print(f"{block_num:<6}{col:<6}{row:<6}{floor:<7}{steps_back:<12}{x:<12.2f}{y:<12.2f}")
                block_num += 1

    with open("block_output_computed.json", "w") as f:
        json.dump(blocks, f, indent=2)

if __name__ == "__main__":
    main()
