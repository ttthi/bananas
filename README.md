#  Bananas Robot Controls

This project implements a fully automated stacking system for blocks using a robot arm and mobile base. Blocks are placed on a defined grid with calculated coordinates, robot movements, and inverse kinematics. The robot operates in a loop to pick and place blocks as defined by a configuration file.

##  Project Structure

```
├── demo.py                    # Main automation logic for robot stacking
├── Remotecontrol.py           # Manual GUI for robot arm positioning
├── IKsolver.py                # CCD-based inverse kinematics solver
├── Sender.py                  # TCP communication with robot hardware
├── robot_config.py            # Arm limits, segment lengths, base angle
├── Pallet_to_armJSON.py       # Converts configurations.json → block_output_computed.json
├── configurations.json        # Input block positions in cm (raw format)
├── block_output_computed.json # Output format used by demo.py (in mm)
├── pose.txt                   # Stores current pose for GUI sync
```

##  How It Works

###  `demo.py`
1. Starts TCP connection to robot.
2. Loads block positions from `block_output_computed.json`.
3. Loops over blocks:
   - Moves arm to pickup position.
   - Closes claw.
   - Lifts the block.
   - Moves backwards by `steps_backward`.
   - Rotates base.
   - Uses inverse kinematics to place the block.
   - Returns to initial position.
4. Prompts user only when placing or picking up blocks, to be replaced with drone interrupts later.

###  `Pallet_to_armJSON.py`
- Converts palletization output `configurations.json` (in cm) into robot-usable JSON (in mm).
- Calculates:
  - `steps_backward`: Based on row position (X-axis) (arduino distance calculation not yet integrated).
  - `target_xy`: Based on Y/Z offset from minimum block.
- Outputs `block_output_computed.json` in the following format:

```json
[
  {
    "block": 1,
    "column": 1,
    "row": 1,
    "floor": 1,
    "steps_backward": 10,
    "target_xy": [251.27, -45.03],
    "base_angle": -90
  },
  ...
]
```

##  Setup Instructions

1. Connect your robot and computer to the same network, suggested to ssh into the robot.
2. Update the robot IP in `demo.py`:
   ```python
   ROBOT_IP = "X.X.X.X"
   ```
3. Test the robot out:
   ```
   python Remotecontrol.py
   ```
4. Run the inverse kinematics converter:
   ```
   python Pallet_to_armJSON.py
   ```
5. Launch the stacking demo:
   ```
   python demo.py
   ```
---