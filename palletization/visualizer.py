# visualizer.py

import json
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from configurations import BLOCK_TYPES


def load_configurations(filename="configurations.json"):
    with open(filename, 'r') as f:
        data = json.load(f)
    pallet_size = tuple(data["pallet_size"])
    configurations = [[(block_type, tuple(pos)) for block_type, pos in config] for config in data["configurations"]]
    return pallet_size, configurations


def draw_cube(size, position, color, transparency=0.6):
    glPushMatrix()

    # Unpack: now position = (x, z, y)
    pos_x, pos_z, pos_y = position
    glTranslatef(pos_x, pos_y, pos_z)

    glColor4f(*color, transparency)

    size_x, size_y, size_z = size
    hx, hy, hz = size_x / 2, size_y / 2, size_z / 2

    vertices = [
        (hx, -hy, -hz), (hx, hy, -hz),
        (-hx, hy, -hz), (-hx, -hy, -hz),
        (hx, -hy, hz), (hx, hy, hz),
        (-hx, -hy, hz), (-hx, hy, hz)
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


def draw_pallet(pallet_size):
    width, length, height = pallet_size

    glColor3f(0.5, 0.3, 0.1)
    glBegin(GL_QUADS)
    glVertex3f(0, -0.1, 0)
    glVertex3f(width, -0.1, 0)
    glVertex3f(width, -0.1, length)
    glVertex3f(0, -0.1, length)
    glEnd()

    glColor3f(0.8, 0.8, 0.8)
    glBegin(GL_LINES)
    corners = [(0, 0), (width, 0), (width, length), (0, length), (0, 0)]
    for x, y in corners:
        glVertex3f(x, -0.1, y)
        glVertex3f(x, height, y)
    glEnd()

    glBegin(GL_LINE_LOOP)
    glVertex3f(0, height, 0)
    glVertex3f(width, height, 0)
    glVertex3f(width, height, length)
    glVertex3f(0, height, length)
    glEnd()


def main():
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    gluPerspective(45, (display[0] / display[1]), 0.1, 100.0)

    pallet_size, configs = load_configurations()
    current_config_index = 0
    zoom_level = -30
    camera_angle_x = 0
    camera_angle_y = 0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_n:
                    current_config_index = (current_config_index + 1) % len(configs)
                elif event.key == pygame.K_LEFT:
                    camera_angle_y -= 10
                elif event.key == pygame.K_RIGHT:
                    camera_angle_y += 10
                elif event.key == pygame.K_UP:
                    camera_angle_x -= 10
                elif event.key == pygame.K_DOWN:
                    camera_angle_x += 10
            elif event.type == pygame.MOUSEWHEEL:
                zoom_level += event.y
                zoom_level = max(min(zoom_level, -5), -50)

        glLoadIdentity()
        gluPerspective(45, (display[0] / display[1]), 0.1, 100.0)
        glTranslatef(0.0, -1.5, zoom_level)
        glRotatef(camera_angle_x, 1, 0, 0)
        glRotatef(camera_angle_y, 0, 1, 0)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        draw_pallet(pallet_size)

        config = configs[current_config_index]
        for block_type, position in config:
            size = BLOCK_TYPES[block_type]['size']
            color = BLOCK_TYPES[block_type]['color']
            draw_cube(size, position, color)

        pygame.display.flip()
        pygame.time.wait(10)

    pygame.quit()


if __name__ == "__main__":
    main()
