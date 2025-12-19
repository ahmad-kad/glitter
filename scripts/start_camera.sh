#!/bin/bash
# Start camera node

cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
source activate_env.sh 2>/dev/null || true

echo "Starting Pi Camera..."
python3 src/camera/pi_camera_node.py
