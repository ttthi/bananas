#!/bin/bash

# Move to the project directory
cd ~/BananasRobot || exit

# Force sync with the remote repo (no merge conflicts)
echo "Updating repo from remote..."
git fetch origin
git reset --hard origin/main

# Ensure permissions for Arduino Nano (top-left USB port)
echo "Setting permissions for /dev/ttyUSB0..."
sudo chmod 666 /dev/ttyUSB0

# Start the robot
echo "Starting robot script..."
python3 run.py