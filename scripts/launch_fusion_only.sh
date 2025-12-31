#!/bin/bash
# Launch only the fusion node (assumes sensors are already running)

echo "=== Launching LiDAR-Camera Fusion Node ==="

# Source ROS 2
source /opt/ros/jazzy/setup.bash
source ~/glitter/install/setup.bash

# Set Python path to include the glitter package
export PYTHONPATH="$HOME/glitter/glitter:$PYTHONPATH"

# Launch fusion node directly with Python (use system Python, not conda)
echo "Starting fusion node..."
/usr/bin/python3 ~/glitter/glitter/core/fusion.py \
    --ros-args \
    --param camera_matrix:="[615.0, 0.0, 310.0, 0.0, 615.0, 240.0, 0.0, 0.0, 1.0]" \
    --param extrinsic_trans:="[0.0, 0.0, 0.1]" \
    --param extrinsic_rot:="[0.0, 0.0, 0.0]" \
    --param lidar_topic:="/unilidar/cloud" \
    --param geometric_prioritization:=false \
    --param max_points_per_frame:=0 \
    --param geometric_weight:=0.7

echo "Fusion node stopped."
