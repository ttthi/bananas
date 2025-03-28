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
sudo systemctl start robot-start.service
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

## Development
Once the robot is set up and connected to the internet, it automatically fetches and merges this repository with its local repository on each reboot.

To manually fetch updates and restart the robot without rebooting:
1. Stop the robot service:
   ```bash
   sudo systemctl stop robot-start.service
   ```
2. Start it again:
   ```bash
   sudo systemctl start robot-start.service
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
  sudo systemctl status robot-start.service
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
