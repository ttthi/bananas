#!/bin/bash

# A shell script for setting permissions and starting the robot
cd ~/BananasRobot
git pull origin main

# Enable communication via the top left USB port (where the arduino nano is connected)
sudo chmod 666 /dev/ttyUSB0

# Start the robot
python3 run.py