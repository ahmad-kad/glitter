#!/bin/bash
# Launch static TF broadcaster for LiDAR frame
# This creates the unilidar_lidar frame so RViz can use it

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo "Launching static TF broadcaster for unilidar_lidar frame..."
echo "This will create the frame so RViz can use it as Fixed Frame"
echo "Press Ctrl+C to stop"
echo ""

# Publish static transform: map -> unilidar_lidar
# Arguments: x y z yaw pitch roll frame_id child_frame_id
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map unilidar_lidar

