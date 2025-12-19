#!/bin/bash
# Diagnose RViz subscription issues

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo "=== LiDAR Topic Diagnostics ==="
echo ""

echo "1. Topic Status:"
ros2 topic info /unilidar/cloud -v | grep -E "(Publisher|Subscription)"

echo ""
echo "2. Topic Frequency:"
timeout 3 ros2 topic hz /unilidar/cloud 2>&1 | head -3

echo ""
echo "3. Sample Message Header:"
timeout 3 ros2 topic echo /unilidar/cloud --once 2>&1 | grep -E "(frame_id|width|height|point_step)" | head -5

echo ""
echo "4. RViz Node Subscriptions:"
ros2 node info /rviz2 2>&1 | grep -A 10 "Subscribers:" | head -15

echo ""
echo "5. All PointCloud2 Topics:"
ros2 topic list | grep -i cloud

echo ""
echo "=== RViz Configuration Check ==="
echo ""
echo "In RViz, verify:"
echo "1. Global Options → Fixed Frame = 'unilidar_lidar'"
echo "2. Displays → Raw_L2_LiDAR → Enabled = ✓"
echo "3. Displays → Raw_L2_LiDAR → Topic = '/unilidar/cloud'"
echo "4. Displays → Raw_L2_LiDAR → Size (Pixels) = 3-5"
echo ""
echo "If subscription count is 0, RViz is not connected!"
echo "Try: Close RViz and relaunch with: ./launch_lidar_rviz.sh"

