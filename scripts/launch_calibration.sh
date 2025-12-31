#!/bin/bash
# Launch calibration GUI

echo "=== Launching LiDAR-Camera Calibration GUI ==="

# Source ROS 2
source /opt/ros/jazzy/setup.bash
source ~/glitter/install/setup.bash

# Set Python path
export PYTHONPATH="$HOME/glitter/glitter:$PYTHONPATH"

# Launch calibration GUI (use system Python, not conda)
echo "Starting calibration GUI..."
/usr/bin/python3 ~/glitter/glitter/core/calibration.py

echo "Calibration GUI closed."
