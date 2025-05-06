# user_input.py

def get_user_selected_blocks():
    print("Select block types from the following:")
    print("1. cube, 3.8cm")
    choice = input("Enter the number corresponding to the block type: ").strip()
    if choice == '1':
        return ['cube, 3.8cm']
    else:
        print("Invalid choice. Defaulting to cube.")
        return ['cube, 3.8cm']

def get_block_metadata():
    print("Specify block physical properties.")
    while True:
        try:
            width = float(input("Block width (cm): "))
            length = float(input("Block length (cm): "))
            height = float(input("Block height (cm): "))
            break
        except ValueError:
            print("Invalid input. Enter numeric values.")

    while True:
        try:
            clearance_x = float(input("Required clearance on X-axis (cm): "))
            clearance_y = float(input("Required clearance on Y-axis (cm): "))
            clearance_z = float(input("Required clearance on Z-axis (cm): "))
            break
        except ValueError:
            print("Invalid input. Enter numeric values.")

    return {
        "dimensions": [width, length, height],
        "clearance": [clearance_x, clearance_y, clearance_z]
    }
