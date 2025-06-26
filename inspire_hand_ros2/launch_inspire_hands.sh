#!/bin/bash

# Launch inspire hands with dual setup

echo "=== Launching Inspire Hands ==="

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch dual hand setup
echo "Starting dual inspire hand setup..."
echo "Make sure both hands are connected to different USB ports"
echo "Default: Left hand on /dev/ttyUSB0, Right hand on /dev/ttyUSB1"
echo ""

ros2 launch inspire_hand_demo inspire_hand_dual.launch.py

