#!/bin/bash

# TeleVision Inspire Hand Bridge Launcher

echo "=== TeleVision Inspire Hand Bridge ==="
echo "Starting bridge system..."

# Source ROS2 environment (use system Python)
source /opt/ros/humble/setup.bash
source install/setup.bash

# Check TeleVision dependencies
echo "Checking TeleVision dependencies..."
python3 -c "import vuer" 2>/dev/null || {
    echo "TeleVision dependencies not found. Please run setup script first."
    echo "Run: ./setup_television_bridge.sh"
    exit 1
}

# Check if inspire hands are connected
echo "Checking for inspire hand devices..."
if ! ls /dev/ttyUSB* &> /dev/null && ! ls /dev/ttyACM* &> /dev/null; then
    echo "Warning: No USB serial devices found. Make sure your inspire hands are connected."
    echo "Available devices:"
    ls /dev/tty* | grep -E "(USB|ACM)" || echo "None found"
fi

# Start the bridge
echo "Starting TeleVision Inspire Bridge..."
python3 television_inspire_bridge.py

