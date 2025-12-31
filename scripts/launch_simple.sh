#!/bin/bash
# Simple launch script for RealSense + L2 sensors

set -e

echo "=== Simple RealSense 435 + Unitree L2 Launch ==="

# Source ROS 2
source /opt/ros/jazzy/setup.bash
source ~/glitter/install/setup.bash

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $LIDAR_PID $CAMERA_PID $TF_PID 2>/dev/null || true
    wait $LIDAR_PID $CAMERA_PID $TF_PID 2>/dev/null || true
    echo "All processes stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "Starting Unitree L2 LiDAR..."
ros2 launch unitree_lidar_ros2 launch.py > /tmp/lidar.log 2>&1 &
LIDAR_PID=$!
echo "LiDAR PID: $LIDAR_PID"

sleep 2

echo "Starting RealSense Camera..."
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
    base_frame_id:=camera_link > /tmp/camera.log 2>&1 &
CAMERA_PID=$!
echo "Camera PID: $CAMERA_PID"

sleep 2

echo "Starting TF transforms..."
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 unilidar_lidar camera_link > /tmp/tf.log 2>&1 &
TF_PID=$!
echo "TF PID: $TF_PID"

echo ""
echo "Sensors launched successfully!"
echo "Topics:"
echo "  - LiDAR: /unilidar/cloud"
echo "  - Camera RGB: /camera/camera/color/image_raw"
echo "  - Camera Depth: /camera/camera/depth/image_rect_raw"
echo "  - Camera PointCloud: /camera/camera/depth/color/points"
echo ""
echo "Press Ctrl+C to stop all sensors"

# Wait for user interrupt
wait


