#!/bin/bash
# Start Pi Camera node

cd "$(dirname "$0")/../.."

source /opt/ros/jazzy/setup.bash
source activate_env.sh 2>/dev/null || true

echo "Starting Pi Camera node..."
echo "Publishing to: /camera/image_raw"
echo ""
echo "To view camera:"
echo "  ros2 run rqt_image_view rqt_image_view /camera/image_raw"
echo ""
echo "Press Ctrl+C to stop"
echo ""

python3 src/camera/pi_camera_node.py










