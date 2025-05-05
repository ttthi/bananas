# configurations.py

# Define the real-world block size
BLOCK_BASE_SIZE_CM = 3.8  # 3.8 cm
MARGIN_CM = 0.3     # 0.3 cm margin

# Effective size of 1 cube block with margin
EFFECTIVE_CUBE_SIZE = BLOCK_BASE_SIZE_CM + MARGIN_CM

BLOCK_TYPES = {
    'cube, 3.8cm': {
        'size': (EFFECTIVE_CUBE_SIZE, EFFECTIVE_CUBE_SIZE, BLOCK_BASE_SIZE_CM),
        'color': (1, 0, 0),  # Red
    },
    # You can define more block types here...

#    'rectangle': {
#        'size': (2 * EFFECTIVE_CUBE_SIZE, EFFECTIVE_CUBE_SIZE, EFFECTIVE_CUBE_SIZE),
#        'color': (0, 1, 0),  # Green
#    }
    # You can define more block types here...
}

def generate_configurations(selected_blocks, pallet_size, user_requested_blocks):
    configurations_list = []

    width, length, height = pallet_size  # width (X), length (Y), height (Z)

    for block_type in selected_blocks:
        config = []

        block_size = BLOCK_TYPES[block_type]['size']
        block_width = block_size[0]
        block_length = block_size[1]
        block_height = block_size[2]

        # How many blocks fit along Width (X), Length (Y), and Height (Z)
        blocks_x = int(width // block_width)
        blocks_y = int(length // block_length)
        blocks_z = int(height // block_height)

        total_possible_blocks = blocks_x * blocks_y * blocks_z
        print(f"Max {block_type} blocks that fit: {total_possible_blocks}")

        blocks_to_place = min(user_requested_blocks, total_possible_blocks)

        start_x = block_width / 2
        start_y = block_length / 2
        start_z = 0 + block_height / 2

        placed = 0
        for z in range(blocks_z):  # floor by floor
            for y in range(blocks_y):  # row by row (depth)
                for x in range(blocks_x):  # left to right (width)
                    if placed >= blocks_to_place:
                        break
                    pos_x = start_x + x * block_width
                    pos_y = start_y + y * block_length
                    pos_z = start_z + z * block_height
                    config.append((block_type, (pos_x, pos_y, pos_z)))
                    placed += 1
                if placed >= blocks_to_place:
                    break
            if placed >= blocks_to_place:
                break

        configurations_list.append(config)

    total_configurations = len(configurations_list)
    return configurations_list, total_configurations
