# 3D_tetris.py
# Original simulation by Leo, made sometime at the end of 2024
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import configurations
from configurations import BLOCK_TYPES
from user_input import get_user_selected_blocks

pygame.init()
display = (800, 600)
pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
gluPerspective(45, (display[0] / display[1]), 0.1, 100.0)
glTranslatef(0.0, -1.5, -30)

# Pallet size
pallet_size = (10, 10, 10)

# 1) Ask user for blocks
selected_blocks = get_user_selected_blocks()

# 2) Generate configurations
configurations_list, total_configurations = configurations.generate_configurations(selected_blocks, pallet_size)

if total_configurations == 0:
    print("No valid configurations could be generated. Exiting.")
    pygame.quit()
    exit()

# 3) Ask how many configurations the user wants to see
print(f"\nWe have generated {total_configurations} possible configurations.")
while True:
    try:
        user_request = int(input(f"How many configurations do you want to see? (1 - {total_configurations}): "))
        if 1 <= user_request <= total_configurations:
            break
        else:
            print(f"Please enter an integer between 1 and {total_configurations}.")
    except ValueError:
        print("Invalid input. Please enter an integer.")

# We'll show up to user_request
max_configs_to_show = user_request
configs_to_show = configurations_list[:max_configs_to_show]

# 4) Ask view mode
view_choice = input(
    "\nHow do you want to view the configurations?\n"
    "  b = one by one (incrementally)\n"
    "  f = full (all blocks at once)\n"
    "Enter choice (b/f): "
).strip().lower()
if view_choice not in ['b', 'f']:
    view_choice = 'b'  # default to incremental if invalid

# Prepare data for each config
num_blocks_placed_list = [0]*max_configs_to_show
block_counts_list = []
total_blocks_list = []

for config in configs_to_show:
    block_counts = {}
    for b_type in BLOCK_TYPES:
        block_counts[b_type] = {'placed': 0, 'total': 0}

    # Count how many of each block is in this configuration
    for block_type, _ in config:
        block_counts[block_type]['total'] += 1
    
    block_counts_list.append(block_counts)
    total_blocks_list.append(len(config))

# Camera
camera_angle_x = 0
camera_angle_y = 0
zoom_level = -30

# Font
pygame.font.init()
font = pygame.font.SysFont('Arial', 18, True)

def render_text(text, x, y):
    text_surface = font.render(text, True, (255, 255, 255), (0, 0, 0))
    text_data = pygame.image.tostring(text_surface, "RGBA", True)
    glWindowPos2d(x, y)
    glDrawPixels(text_surface.get_width(), text_surface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, text_data)

def draw_cube(size, position, color, transparency=0.6):
    glPushMatrix()
    glTranslatef(*position)
    glColor4f(*color, transparency)

    size_x, size_y, size_z = size
    hx, hy, hz = size_x / 2, size_y / 2, size_z / 2

    vertices = [
        ( hx, -hy, -hz), ( hx,  hy, -hz),
        (-hx,  hy, -hz), (-hx, -hy, -hz),
        ( hx, -hy,  hz), ( hx,  hy,  hz),
        (-hx, -hy,  hz), (-hx,  hy,  hz)
    ]
    
    faces = [
        (0, 1, 2, 3),
        (3, 2, 7, 6),
        (6, 7, 5, 4),
        (4, 5, 1, 0),
        (1, 5, 7, 2),
        (4, 0, 3, 6)
    ]
    
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    
    glBegin(GL_QUADS)
    for face in faces:
        for vertex in face:
            glVertex3fv(vertices[vertex])
    glEnd()
    
    glDisable(GL_BLEND)
    glPopMatrix()

def draw_pallet():
    glColor3f(0.5, 0.3, 0.1)
    glBegin(GL_QUADS)
    glVertex3f(-5, -0.1, -5)
    glVertex3f( 5, -0.1, -5)
    glVertex3f( 5, -0.1,  5)
    glVertex3f(-5, -0.1,  5)
    glEnd()
    
    max_height = 10
    glColor3f(0.8, 0.8, 0.8)
    glBegin(GL_LINES)
    corners = [(-5, -5), (5, -5), (5, 5), (-5, 5), (-5, -5)]
    for x, z in corners:
        glVertex3f(x, -0.1, z)
        glVertex3f(x, max_height, z)
    glEnd()
    
    glBegin(GL_LINE_LOOP)
    glVertex3f(-5, max_height, -5)
    glVertex3f( 5, max_height, -5)
    glVertex3f( 5, max_height,  5)
    glVertex3f(-5, max_height,  5)
    glEnd()

def display_configuration(config, num_blocks):
    for i in range(min(num_blocks, len(config))):
        block_type, pos = config[i]
        size = BLOCK_TYPES[block_type]['size']
        color = BLOCK_TYPES[block_type]['color']
        draw_cube(size, pos, color)

current_config_index = 0
running = True

# If user chose full view for the first config
if view_choice == 'f':
    num_blocks_placed_list[0] = total_blocks_list[0]
    for (btype, _) in configs_to_show[0]:
        block_counts_list[0][btype]['placed'] += 1

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
            
            # SPACE => next block (only if 'b')
            elif event.key == pygame.K_SPACE and view_choice == 'b':
                if num_blocks_placed_list[current_config_index] < total_blocks_list[current_config_index]:
                    num_blocks_placed_list[current_config_index] += 1
                    block_type, _ = configs_to_show[current_config_index][
                        num_blocks_placed_list[current_config_index] - 1
                    ]
                    block_counts_list[current_config_index][block_type]['placed'] += 1
            
            # R => reset
            elif event.key == pygame.K_r:
                num_blocks_placed_list[current_config_index] = 0
                for btype in BLOCK_TYPES:
                    block_counts_list[current_config_index][btype]['placed'] = 0
                # If full mode, instantly fill them
                if view_choice == 'f':
                    num_blocks_placed_list[current_config_index] = total_blocks_list[current_config_index]
                    for (btype, _) in configs_to_show[current_config_index]:
                        block_counts_list[current_config_index][btype]['placed'] = \
                            block_counts_list[current_config_index][btype]['total']
            
            # Arrow keys => rotate
            elif event.key == pygame.K_LEFT:
                camera_angle_y -= 10
            elif event.key == pygame.K_RIGHT:
                camera_angle_y += 10
            elif event.key == pygame.K_UP:
                camera_angle_x -= 10
            elif event.key == pygame.K_DOWN:
                camera_angle_x += 10
            
            # N => next configuration
            elif event.key == pygame.K_n:
                if current_config_index < max_configs_to_show - 1:
                    current_config_index += 1
                    # If user chose full view for new config
                    if view_choice == 'f':
                        num_blocks_placed_list[current_config_index] = total_blocks_list[current_config_index]
                        for (btype, _) in configs_to_show[current_config_index]:
                            block_counts_list[current_config_index][btype]['placed'] += 1
                else:
                    running = False
        
        elif event.type == pygame.MOUSEWHEEL:
            zoom_level += event.y
            zoom_level = max(min(zoom_level, -5), -50)

    glLoadIdentity()
    gluPerspective(45, (display[0] / display[1]), 0.1, 100.0)
    glTranslatef(0.0, -1.5, zoom_level)
    glRotatef(camera_angle_x, 1, 0, 0)
    glRotatef(camera_angle_y, 0, 1, 0)

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    draw_pallet()

    # Draw current config
    
    current_config = configs_to_show[current_config_index]
    display_configuration(current_config, num_blocks_placed_list[current_config_index])

    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    gluOrtho2D(0, display[0], 0, display[1])
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()

    total_blocks = total_blocks_list[current_config_index]
    placed_blocks = num_blocks_placed_list[current_config_index]
    render_text(
        f"Configuration {current_config_index+1}/{max_configs_to_show} | Blocks Placed: {placed_blocks}/{total_blocks}",
        10,
        display[1] - 30
    )

    # Show block counts

    y_offset = display[1] - 60
    for btype in BLOCK_TYPES:
        placed = block_counts_list[current_config_index][btype]['placed']
        total = block_counts_list[current_config_index][btype]['total']
        size = BLOCK_TYPES[btype]['size']
        size_str = f"{size[0]}x{size[1]}x{size[2]}"
        render_text(f"{btype.capitalize()} (size {size_str}): {placed}/{total}", 10, y_offset)
        y_offset -= 20

   

    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)
    glPopMatrix()

    pygame.display.flip()
    pygame.time.wait(10)

pygame.quit()
