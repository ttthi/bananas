import sys
import time
import math
import subprocess
import threading
import os #works on Windows
import subprocess
from Sender import Sender
from robot_config import (
    STABILIZE_LAST, LENGTHS, REL_ANGLES, MIN_ANGLES,
    MAX_ANGLES, BASE_POS, MAX_ITER, DST_THRESHOLD, BASE_ANGLE
)

# IP & port (could be moved to config if reused often)
ROBOT_IP = "192.168.0.63"
ROBOT_PORT = 5001

# Known good poses
PICKUP_ANGLES = [math.radians(16), math.radians(-5), math.radians(-10)]
LIFTED_ANGLES = [math.radians(86), math.radians(-53), math.radians(-32)]

def confirm_step(text):
    answer = input(f"{text} (Enter to continue, 'q' to quit): ").strip().lower()
    if answer == 'q':
        print("Aborted.")
        sys.exit(0)

def send_arm_pose(sender, base_angle, rel_angles):
    degs = [int(math.degrees(a)) for a in rel_angles]
    command = f"set:{base_angle},{','.join(map(str, degs))}"
    print(command)
    sender.send_tcp_data(command)

    # Save pose to file for GUI sync
    with open("pose.txt", "w") as f:
        f.write(f"{base_angle}," + ",".join(map(str, degs)))

def main():
    sender = Sender(ROBOT_IP, ROBOT_PORT)
    sender.start()

    print("=== Box Stacking Demo ===")
    confirm_step("Start demo")

    stack_count = 0
    while True:
        print(f"\n=== Starting stack cycle #{stack_count + 1} ===")

        # Step 0: Move to initial position
        print("Step 0: Initial Pos")
        send_arm_pose(sender, base_angle=0, rel_angles=LIFTED_ANGLES)
        sender.send_tcp_data("open")
        confirm_step("Robot in initial pos?")

        # Step 1: Move to box position
        print("Step 1: Moving arm to pick up position")
        sender.send_tcp_data("forward:5")
        send_arm_pose(sender, base_angle=0, rel_angles=PICKUP_ANGLES)
        confirm_step("Arm over box?")

        # Step 2: Close claw
        print("Step 2: Closing claw")
        sender.send_tcp_data("close")
        confirm_step("Claw closed?")

        # Step 3: Lift box
        print("Step 3: Lifting box")
        send_arm_pose(sender, base_angle=0, rel_angles=LIFTED_ANGLES)
        confirm_step("Box lifted?")

        # Step 4: Move robot back
        print("Step 4: Moving robot backward")
        sender.send_tcp_data("backward:5")
        confirm_step("Robot moved back?")

        # Step 5: Rotate base
        print("Step 5: Rotating base left")
        send_arm_pose(sender, base_angle=-90, rel_angles=LIFTED_ANGLES)
        confirm_step("Base rotated?")

        # Step 6: Lower arm to place
        print("Step 6: Lowering box to place it")
        send_arm_pose(sender, base_angle=-90, rel_angles=PICKUP_ANGLES)
        confirm_step("Arm lowered?")

        # Prompt user
        answer = input("Is the arm lowered correctly? (y = yes, n = open GUI): ").strip().lower()
        if answer == 'n':
            sender.stop()
            print("Launching manual control GUI for adjustment...")

            gui_path = os.path.join(os.getcwd(), "Remotecontrol.py")
            subprocess.Popen(["start", "python", gui_path], shell=True)

            input("Press Enter when you're done adjusting and ready to continue...")
            sender.start()

        # Step 7: Open claw
        print("Step 7: Opening claw")
        sender.send_tcp_data("open")
        confirm_step("Box placed?")

        # Step 8: Lift claw
        print("Step 8: Lifting box")
        send_arm_pose(sender, base_angle=90, rel_angles=LIFTED_ANGLES)
        confirm_step("Claw lifted?")

        # Ask to continue or quit
        user = input("Continue stacking another box? (Enter = yes, q = quit): ").strip().lower()
        if user == "q":
            print("Exiting demo loop.")
            break

        # Reset base angle if needed
        BASE_ANGLE = 0
        stack_count += 1

    print("Demo complete.")
    sender.stop()

if __name__ == "__main__":
    main()

