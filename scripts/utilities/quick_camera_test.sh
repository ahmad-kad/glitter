#!/bin/bash
# Quick camera test - launch and verify

source /opt/ros/jazzy/setup.bash

echo "Testing camera_ros..."
echo ""

# Launch camera in background
ros2 run camera_ros camera_ros_node &
CAMERA_PID=$!

echo "Camera launched (PID: $CAMERA_PID)"
echo "Waiting 3 seconds for initialization..."
sleep 3

# Check topics
echo ""
echo "Checking topics:"
ros2 topic list 2>/dev/null | grep camera

# Check if publishing
echo ""
echo "Checking publishing rate:"
timeout 3 ros2 topic hz /camera/image_raw 2>&1 | head -10

# Check topic info
echo ""
echo "Topic info:"
ros2 topic info /camera/image_raw 2>/dev/null

# Try to view image
echo ""
echo "To view image, run in another terminal:"
echo "  ros2 run rqt_image_view rqt_image_view /camera/image_raw"
echo ""
echo "Press Ctrl+C to stop camera"
echo ""

# Wait for interrupt
trap "kill $CAMERA_PID 2>/dev/null; exit" INT TERM
wait $CAMERA_PID










