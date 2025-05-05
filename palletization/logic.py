# logic.py

import json
from configurations import BLOCK_TYPES, generate_configurations
from user_input import get_user_selected_blocks, get_block_metadata

def save_configurations(configs, pallet_size, block_meta, filename="configurations.json"):
    output = {
        "pallet_size": list(pallet_size),
        "block_dimensions": block_meta["dimensions"],
        "clearance": block_meta["clearance"],
        "configurations": [
            [(block_type, list(position)) for block_type, position in config]
            for config in configs
        ]
    }
    with open(filename, 'w') as f:
        json.dump(output, f, indent=4)


def main():
    # Pallet size (cm)
    pallet_size = (10, 10, 15)  # width, length, height

    selected_blocks = get_user_selected_blocks()
    block_meta = get_block_metadata()

    while True:
        try:
            user_requested_blocks = int(input("How many blocks do you want to place? "))
            if user_requested_blocks > 0:
                break
            else:
                print("Please enter a positive integer.")
        except ValueError:
            print("Invalid input. Please enter an integer.")

    configs_list, total_configurations = generate_configurations(selected_blocks, pallet_size, user_requested_blocks)

    if total_configurations == 0:
        print("No valid configurations could be generated. Exiting.")
        return

    print(f"Generated {total_configurations} configurations.")
    save_configurations(configs_list, pallet_size, block_meta)
    print("Configurations saved to configurations.json.")

if __name__ == "__main__":
    main()