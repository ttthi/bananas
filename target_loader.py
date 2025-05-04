import json
import math


def load_targets_from_json(filename):
    with open(filename, "r") as f:
        data = json.load(f)

    targets = []
    for entry in data:
        base_angle = entry["base_angle_deg"]
        x, y = entry["target_xy_mm"]
        targets.append({
            "block": entry["block"],
            "column": entry["column"],
            "floor": entry["floor"],
            "steps_forward": entry["steps_forward"],
            "base_angle": base_angle,
            "target_xy": (x, y)
        })
    return targets
