#!/bin/bash
# Check LiDAR data and RViz configuration

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo "=== LiDAR Data Check ==="
echo ""

echo "1. Checking topics..."
ros2 topic list | grep -E "(unilidar|cloud|lidar)"

echo ""
echo "2. Checking topic frequency..."
timeout 3 ros2 topic hz /unilidar/cloud 2>&1 | head -5

echo ""
echo "3. Checking topic info..."
ros2 topic info /unilidar/cloud

echo ""
echo "4. Checking point cloud data (first message)..."
timeout 5 ros2 topic echo /unilidar/cloud --once --field header,width,height,point_step,row_step 2>&1 | head -20

echo ""
echo "5. Checking TF frames..."
ros2 run tf2_ros tf2_echo unilidar_lidar map 2>&1 | head -5 || echo "TF not available (may be normal)"

echo ""
echo "=== RViz Configuration Check ==="
echo ""
echo "Fixed Frame should be: unilidar_lidar"
echo "Topic should be: /unilidar/cloud"
echo ""
echo "In RViz:"
echo "1. Global Options → Fixed Frame → Set to 'unilidar_lidar'"
echo "2. Displays → Raw_L2_LiDAR → Topic → Set to '/unilidar/cloud'"
echo "3. Make sure the display is enabled (checkbox checked)"

