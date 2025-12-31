#!/bin/bash

# Manual step-by-step launch script for RealSense 435 + Unitree L2
# Use this for debugging or when the combined launch fails

set -e

echo "=== Manual Launch: RealSense 435 + Unitree L2 ==="
echo ""

# Source ROS 2
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# Function to check if a process is running
check_process() {
    pgrep -f "$1" > /dev/null
}

echo "Step 1: Starting Unitree L2 LiDAR..."
echo "Command: ros2 launch unitree_lidar_ros2 launch.py"
echo "Press Enter to continue..."
read

# Launch L2 LiDAR in background
ros2 launch unitree_lidar_ros2 launch.py &
LIDAR_PID=$!
echo "✓ L2 LiDAR launched (PID: $LIDAR_PID)"
sleep 3

echo ""
echo "Step 2: Starting RealSense Camera..."
echo "Command: ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true enable_pointcloud:=true align_depth:=true"
echo "Press Enter to continue..."
read

# Launch RealSense camera in background
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    enable_depth:=true \
    enable_pointcloud:=true \
    align_depth:=true \
    color_width:=640 \
    color_height:=480 \
    color_fps:=30.0 \
    depth_width:=640 \
    depth_height:=480 \
    depth_fps:=30.0 \
    pointcloud_texture_index:=0 \
    ordered_pc:=false \
    enable_sync:=true \
    diagnostics_period:=0.0 \
    base_frame_id:=camera_link &
CAMERA_PID=$!
echo "✓ RealSense camera launched (PID: $CAMERA_PID)"
sleep 3

echo ""
echo "Step 3: Starting TF Transforms..."
echo "Command: ros2 launch config/tf/static_transforms.launch.py"
echo "Press Enter to continue..."
read

# Launch TF transforms in background
ros2 launch config/tf/static_transforms.launch.py &
TF_PID=$!
echo "✓ TF transforms launched (PID: $TF_PID)"
sleep 2

echo ""
echo "Step 4: Checking topics..."
echo "Available topics:"
ros2 topic list | grep -E "(camera|unilidar|tf)"
echo ""

echo "Step 5: Starting RViz..."
echo "Command: rviz2 -d config/realsense_l2_combined.rviz"
echo "Press Enter to continue..."
read

# Launch RViz
rviz2 -d config/realsense_l2_combined.rviz &
RVIZ_PID=$!
echo "✓ RViz launched (PID: $RVIZ_PID)"

echo ""
echo "=== All components launched ==="
echo "PIDs:"
echo "  L2 LiDAR: $LIDAR_PID"
echo "  RealSense: $CAMERA_PID"
echo "  TF: $TF_PID"
echo "  RViz: $RVIZ_PID"
echo ""
echo "Press Ctrl+C to stop all processes"

# Wait for user interrupt
trap 'echo ""; echo "Stopping all processes..."; kill $LIDAR_PID $CAMERA_PID $TF_PID $RVIZ_PID 2>/dev/null; exit 0' INT
wait
