# user_input.py

def get_user_selected_blocks():
    print("Available block types:")
    print("  1. cube, 3.8cm")
    #print("  2. Rectangle")

    block_mapping = {
        "1": "cube, 3.8cm",
        #"2": "rectangle",
    }

    selected = []
    while True:
        choices = input("Enter the numbers of the block types you want to use (comma-separated, e.g., 1, 2...): ")
        try:
            indices = [choice.strip() for choice in choices.split(',')]
            for idx in indices:
                if idx in block_mapping:
                    selected.append(block_mapping[idx])
                else:
                    print(f"Invalid choice: {idx}")
                    break
            else:
                break  # all were valid
        except Exception as e:
            print(f"Error: {e}. Try again.")

    return selected
