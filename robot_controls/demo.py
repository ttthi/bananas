import sys
import time
import math
import subprocess
import os
import json
from IKsolver import ccd_inverse_kinematics, get_absolute_angles
from Sender import Sender
from robot_config import (
    STABILIZE_LAST, LENGTHS, REL_ANGLES, MIN_ANGLES,
    MAX_ANGLES, BASE_POS, MAX_ITER, DST_THRESHOLD, BASE_ANGLE
)

ROBOT_IP = "192.168.0.63"
ROBOT_PORT = 5001

def calculate_arm_pose(target_xy):
    if STABILIZE_LAST:
        adj_target = (target_xy[0] - LENGTHS[-1], target_xy[1])
        angles = ccd_inverse_kinematics(
            REL_ANGLES[:-1], LENGTHS, BASE_POS, adj_target,
            MAX_ITER, MIN_ANGLES, MAX_ANGLES, DST_THRESHOLD
        )
        angles.append(-get_absolute_angles(angles)[-1])
    else:
        angles = ccd_inverse_kinematics(
            REL_ANGLES, LENGTHS, BASE_POS, target_xy,
            MAX_ITER, MIN_ANGLES, MAX_ANGLES, DST_THRESHOLD
        )
    return angles

def confirm_step(text):
    answer = input(f"{text} (Enter to continue, 'q' to quit): ").strip().lower()
    if answer == 'q':
        print("Aborted.")
        sys.exit(0)

def send_arm_pose(sender, base_angle, rel_angles):
    global REL_ANGLES, BASE_ANGLE
    BASE_ANGLE = base_angle
    REL_ANGLES[:] = rel_angles
    degs = [int(math.degrees(a)) for a in rel_angles]
    command = f"set:{base_angle},{','.join(map(str, degs))}"
    print(command)
    sender.send_tcp_data(command)

    with open("pose.txt", "w") as f:
        f.write(f"{base_angle}," + ",".join(map(str, degs)))

def main():
    sender = Sender(ROBOT_IP, ROBOT_PORT)
    sender.start()

    with open("block_output_computed.json") as f:
        targets = json.load(f)

    print("=== Box Stacking Demo (Using JSON Targets) ===")
    confirm_step("Start demo")

    #TODO: change into (x, y)
    PICKUP_ANGLES = [math.radians(-16), math.radians(-14), math.radians(31)]
    LIFTED_ANGLES = [math.radians(86), math.radians(-53), math.radians(-32)]

    for stack_count, target in enumerate(targets):
        backward_steps = target["steps_backward"]

        print(f"\n=== Starting stack cycle #{stack_count + 1} (Block {target['block']}) ===")

        # Step 0: Move to initial position
        print("Step 0: Initial Pos")
        send_arm_pose(sender, base_angle=0, rel_angles=LIFTED_ANGLES)
        sender.send_tcp_data("open")
        sender.send_tcp_data("open")
        sender.send_tcp_data("open")

        while True: #TODO: use arduino pos instead for more accurate results
            move = input("Adjust robot position? (F=forward, B=backward, Enter=confirm): ").strip().lower()
            if move == 'f':
                sender.send_tcp_data("forward:1")
            elif move == 'b':
                sender.send_tcp_data("backward:1")
            elif move == '':
                break
            else:
                print("Invalid input. Press 'F', 'B' or Enter.")

        # Step 1: Move arm
        print("Step 1: Moving arm to pick up position")
        send_arm_pose(sender, base_angle=0, rel_angles=PICKUP_ANGLES)
        confirm_step("Claw over box?")

        # Step 2: Close claw
        print("Step 2: Closing claw")
        sender.send_tcp_data("close")
        time.sleep(2.5)

        # Step 3: Lift box
        print("Step 3: Lifting box")
        send_arm_pose(sender, base_angle=0, rel_angles=LIFTED_ANGLES)

        # Step 4: Move robot back
        print("Step 4: Moving robot backward")
        print(f"Moving robot backward {backward_steps} steps for clearance")
        sender.send_tcp_data(f"backward:{backward_steps}")
        time.sleep(1.5)

        # Step 5: Rotate base to target base angle
        print(f"Step 5: Rotating base to {target['base_angle']}°")
        send_arm_pose(sender, base_angle=target["base_angle"], rel_angles=LIFTED_ANGLES)
        time.sleep(10)

        # Step 6: Lower arm using target coordinates
        print(f"Step 6: Lowering box to ({target['target_xy'][0]}, {target['target_xy'][1]})")
        rel_angles = calculate_arm_pose(target["target_xy"])
        send_arm_pose(sender, base_angle=target["base_angle"], rel_angles=rel_angles)

        # Manual override if needed
        answer = input("Is the arm lowered correctly? (y = yes, n = open GUI): ").strip().lower()
        if answer == 'n':
            sender.stop()
            print("Launching manual control GUI for adjustment...")
            gui_path = os.path.join(os.getcwd(), "Remotecontrol.py")
            subprocess.Popen(["start", "python", gui_path], shell=True)
            input("Press Enter when you're done adjusting and ready to continue...")
            sender.start()
        #confirm_step("Box placed?")

        # Step 7: Open claw
        print("Step 7: Opening claw")
        sender.send_tcp_data("open")

        # Step 8: Lift claw before rotating or moving again
        print("Step 8: Lifting claw")
        send_arm_pose(sender, base_angle=target["base_angle"], rel_angles=LIFTED_ANGLES)
        confirm_step("Arm lifted?")

        # Step 9: Move robot forward to start position
        print(f"Step 9: Moving robot forward {backward_steps} steps to reset")
        sender.send_tcp_data(f"forward:{backward_steps}")

        # Ask to continue or quit
        user = input("Continue stacking another box? (Enter = yes, q = quit): ").strip().lower()
        if user == "q":
            print("Exiting demo loop.")
            break

    print("Demo complete.")
    sender.stop()

if __name__ == "__main__":
    main()
