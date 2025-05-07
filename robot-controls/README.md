# Robot Setup & Development Guide

## Setup

### Step 1: Connect to a Monitor and Keyboard
Connect a monitor and keyboard directly to the robot. Alternatively, you can connect via USB using an SSH client.

### Step 2: Boot and Login
When the LCD at the back of the robot powers on and displays text, the robot is fully booted. Use the following credentials:
- **Username**: `jetbot`
- **Password**: `jetbot`

### Step 3: Connect to Wi-Fi
After logging in, connect to Wi-Fi using:
```bash
sudo nmcli device wifi connect "<your_wifi_name>" password "<your_wifi_password>"
```

The robot should automatically start. You can confirm this by observing logs in the console or by checking if the gripper on the arm moves. If it does not start, run:
```bash
sudo systemctl start robot-startup.service
```

### Step 4 (Optional): SSH into the Robot from Another Device
For easier development and debugging, you can SSH into the robot once it's on the network:

1. Note the robot's IP address displayed on the LCD. If none is shown, reboot the robot.
2. From another device, run:
   ```bash
   ssh jetbot@<robot_ip_address>
   ```
3. Log in with the same `jetbot` credentials.

---
### Step 5: Troubleshooting Wi-Fi
If connecting to Wi-Fi using nmcli didn't work, try clearing known connections:
```bash
sudo rm /etc/NetworkManager/system-connections/*.nmconnection"
```

After this restart NetworkManager
```bash
sudo systemctl restart NetworkManager
```

## Usage

### Method 1: SSH Control

Once an SSH connection has been established with the robot, you can control it via the Python console. Navigate to the `BananasRobot` directory and execute the following commands:

```python
from BananasRobotMain import BananasRobot
robot = BananasRobot()
```

You can now issue various commands using the `robot` object. For more information on available methods and capabilities, refer to the `BananasRobotMain.py` file.

---

### Method 2: TCP Text Interface

On startup, the robot automatically runs the `update_and_run.sh` script, which:

1. Updates the codebase.
2. Starts the `run.py` script in the background, which in turn creates a `robot` object.

The `robot` object also provides a basic TCP-based text interface for controlling the robot. By default, it listens for incoming TCP connections on **port 5001** and processes textual commands sent over those connections. Supported commands include:

- `set:a1,a2,a3,a4` — Set the angles of the arm joints.
- `forward:dst` — Move the robot forward by `dst` centimeters.
- `backward:dst` — Move the robot backward by `dst` centimeters.
- `open` — Open the claw.
- `close` — Close the claw.

A basic Python GUI for controlling the robot over TCP is available at:  
[https://github.com/mmjm27/Bananas-robot-controls](https://github.com/mmjm27/Bananas-robot-controls)

---

## Development
Once the robot is set up and connected to the internet, it automatically fetches and merges this repository with its local repository on each reboot.

To manually fetch updates and restart the robot without rebooting:
1. Stop the robot service:
   ```bash
   sudo systemctl stop robot-startup.service
   ```
2. Start it again:
   ```bash
   sudo systemctl start robot-startup.service
   ```

---

## Troubleshooting

### `robot-start.service` Doesn't Execute
If the BananasRobot software fails to start or update, it may be a permission issue with `update_and_run.sh`.

- Restore permissions:
  ```bash
  chmod +x update_and_run.sh
  ```
- Check the service status:
  ```bash
  sudo systemctl status robot-startup.service
  ```

### PyTorch Library Segmentation Fault
Limited disk space or corruption may cause PyTorch to fail:

1. Remove PyTorch:
   ```bash
   pip3 uninstall torch torchvision torchaudio
   ```
2. Reinstall:
   ```bash
   pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
   ```

---

For additional issues or improvements, feel free to update this README.
