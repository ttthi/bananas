import pygame
import math
import sys
from Sender import Sender
from IKsolver import *
from robot_config import (
    STABILIZE_LAST, LENGTHS, REL_ANGLES, NUM_JOINTS, BASE_ANGLE,
    MIN_ANGLES, MAX_ANGLES, MAX_ITER, DST_THRESHOLD, BASE_POS
)

# shit UI program to control the robot remotely

# Replace the IP with the ip from the robot
handler = Sender(target_ip="192.168.0.63", target_port=5001)
handler.start()

pygame.init()

WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()
pygame.display.set_caption("Bananas robot controls")

font = pygame.font.SysFont(None, 24)
dst_value = 1

def transformXY(x, y):
    #Helper funcntion to tranform the coordintes when drawing.
    return (x + 200 , (HEIGHT - y) - 200)

def detransformXY(x, y):
    #Does the opposite of transformXY
    return (x - 200 , (HEIGHT - y) - 200)

def list_to_str(l):
    return ",".join(str(e) for e in l)

def update_arm():
    try:
        msg = "set:" + str(BASE_ANGLE) + "," + list_to_str(map(lambda x: int(math.degrees(x)), REL_ANGLES))
        print(msg)
        handler.send_tcp_data(msg)

        with open("pose.txt", "w") as f:
            f.write(str(BASE_ANGLE) + "," + list_to_str(map(lambda x: int(math.degrees(x)), REL_ANGLES)))

    except KeyboardInterrupt:
        handler.stop()

def main():
    global BASE_ANGLE
    global REL_ANGLES
    running = True

    try:
        with open("pose.txt", "r") as f:
            parts = f.read().strip().split(",")
            BASE_ANGLE = int(parts[0])
            REL_ANGLES = [math.radians(int(p)) for p in parts[1:]]
            print(f"Loaded saved pose: BASE={BASE_ANGLE}, REL_ANGLES={REL_ANGLES}")
    except Exception as e:
        print(f"No saved pose found or failed to load: {e}")

    joint_positions = forward_kinematics(REL_ANGLES, LENGTHS, BASE_POS)

    while running:
        clock.tick(60)
        screen.fill((30, 30, 30))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_RIGHT:
                    if BASE_ANGLE < 0:
                        BASE_ANGLE += 90
                    update_arm()
                elif event.key == pygame.K_LEFT:
                    if BASE_ANGLE > -90:
                        BASE_ANGLE -= 90
                    update_arm()
                elif event.key == pygame.K_UP:
                    try:
                        handler.send_tcp_data("open")
                    except KeyboardInterrupt:
                        handler.stop()
                elif event.key == pygame.K_DOWN:
                    try:
                        handler.send_tcp_data("close")
                    except KeyboardInterrupt:
                        handler.stop()
                elif event.key == pygame.K_f:
                    try:
                        handler.send_tcp_data(f"forward:{dst_value}")
                    except KeyboardInterrupt:
                        handler.stop()
                elif event.key == pygame.K_b:
                    try:
                        handler.send_tcp_data(f"backward:{dst_value}")
                    except KeyboardInterrupt:
                        handler.stop()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                target_pos = detransformXY(mx, my)
                print(target_pos)

                if STABILIZE_LAST:
                    t_angles = ccd_inverse_kinematics(
                        REL_ANGLES[:-1], LENGTHS, BASE_POS, (target_pos[0] - LENGTHS[-1], target_pos[1]),
                        MAX_ITER,
                        MIN_ANGLES, MAX_ANGLES, DST_THRESHOLD
                    )

                    t_angles.append(-get_absolute_angles(t_angles)[-1])
                    REL_ANGLES = t_angles
                else:
                    REL_ANGLES = ccd_inverse_kinematics(
                        REL_ANGLES, LENGTHS, BASE_POS, target_pos,
                        MAX_ITER,
                        MIN_ANGLES, MAX_ANGLES, DST_THRESHOLD
                    )

                update_arm()

                joint_positions = forward_kinematics(REL_ANGLES, LENGTHS, BASE_POS)

        mx, my = pygame.mouse.get_pos()

        BASE_POS_S = transformXY(BASE_POS[0], BASE_POS[1])
        B = (BASE_POS_S[0], BASE_POS_S[1]+99)
        pygame.draw.line(screen, (200, 200, 200), BASE_POS_S, B, 3)
        pygame.draw.rect(screen,(0, 0, 0),(B[0] - 120, B[1], 160, 85),3)

        last_pos = BASE_POS
        for pos in joint_positions:
            pygame.draw.line(screen, (200, 200, 200), transformXY(last_pos[0], last_pos[1]), transformXY(pos[0], pos[1]), 3)
            pygame.draw.circle(screen, (0, 255, 0), transformXY(int(pos[0]), int(pos[1])), 5)
            last_pos = pos

        pygame.draw.circle(screen, (255, 0, 0), transformXY(BASE_POS[0], BASE_POS[1]), 8)

        pygame.draw.circle(screen, (0, 100, 255), (int(mx), int(my)), 5)

        dst_indicator = font.render(f"dst: {dst_value}", True, (0, 255, 0))
        screen.blit(dst_indicator, (25, 25))

        dst_indicator = font.render(f"f/b: Forward/Back, Left/Right: Rotate L/R, Up/Down: Open/Close", True, (255, 255, 255))
        screen.blit(dst_indicator, (200, 25))

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()