# Setup

## Step 1: Connect to a Monitor and a Keyboard

Connecting via usb with an ssh client also possible

## Step 2: Boot and login

You know the robot is fully booted when the LCD at the back of the robot powers on and display text. Username and Password are both "jetbot"

## Step 3: Connect to wifi

Once logged in connect to network by running the following command: sudo nmcli device wifi connect "your wifi name" password "your wifi password".
The Robot should now start automatically, indicated by the logs in the console or the gripper on the arm moving. If not you can run the command 
sudo systemctl start robot-start.service in the home directory

## Step 4 (Optional): Connect to the robot from an other device with ssh

This is helpful for development and debuggin as the robot is easier to use from an other device. Once booted the robot should display an ip address at the back of the robot
if not reboot the robot. Once the ip address is know run the following command on your other device: ssh jetbot@the_ip_at_the_back and login normally

# Development

Once the setup is complete, the development of robot codebase can be done on another device. When the robot reboots and connects to the internet it automatically fetches and merges this repository with
its local repository. If you want to make changes while the robot is running, without having to reboot, you can run the manually start the fetch and start process by doing the following:
1. Stop the robot from running: sudo systemctl stop robot-start.service 2. Restart the robot: sudo systemctl start robot-start.service

# Troubleshooting

## robot-start.service wont execute

This means that the BananasRobot software wont update its code base or start. This is most likely because changes were made to the update_and_run.sh shellscript. This resets the permissions of the shellscript and requires them to me manually reset by running this chmod +x update_and_run.sh
By running sudo systemctl status robot-start.service you can also inspect the logs of the systemservice inorder to debug any errors related to it.

## Pytorch library gives an segmentation fault

Pytorch library might get corrupted due to limited diskspace?? This wont let the BananasRobot software to start and lead to an import error while importing the pytorch library. To fix this you need to: 1. delete pytorch: pip3 uninstall torch torchvision torchaudio
2. Reinstall: pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
