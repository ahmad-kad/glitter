#!/bin/bash
# Fix RViz frame issue by creating static TF transform
# This allows RViz to use base_link or unilidar_lidar as fixed frame

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

echo "Creating static TF transforms for RViz..."
echo ""

# Create map -> base_link -> unilidar_lidar chain
# This allows RViz to use either frame as fixed frame

echo "Publishing: map -> base_link"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link &
TF1_PID=$!

sleep 1

echo "Publishing: base_link -> unilidar_lidar"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link unilidar_lidar &
TF2_PID=$!

sleep 1

echo "Publishing: base_link -> livox_frame (if needed)"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link livox_frame &
TF3_PID=$!

echo ""
echo "TF transforms created!"
echo "  - map -> base_link"
echo "  - base_link -> unilidar_lidar"
echo "  - base_link -> livox_frame"
echo ""
echo "You can now use 'base_link' or 'unilidar_lidar' as Fixed Frame in RViz"
echo ""
echo "Press Ctrl+C to stop all transforms"

# Wait for interrupt
trap "kill $TF1_PID $TF2_PID $TF3_PID 2>/dev/null; exit" INT TERM
wait










