#!/bin/bash
# Quick script to source ROS 2 and workspace
# Usage: source source_ros.sh

# Source ROS 2 Jazzy
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✓ ROS 2 Jazzy sourced"
else
    echo "✗ ROS 2 Jazzy not found at /opt/ros/jazzy"
    return 1 2>/dev/null || exit 1
fi

# Source ROS workspace
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
    echo "✓ ROS workspace sourced"
else
    echo "⚠ ROS workspace not found at ~/ros2_ws/install/setup.bash"
    echo "  Build workspace first: cd ~/ros2_ws && colcon build"
fi

# Verify
if command -v ros2 &> /dev/null; then
    echo "✓ ROS 2 ready (ros2 command available)"
    echo ""
    echo "Quick commands:"
    echo "  ros2 topic list          # List topics"
    echo "  ros2 topic echo <topic>  # Echo topic"
    echo "  ros2 node list            # List nodes"
else
    echo "✗ ros2 command not found"
    return 1 2>/dev/null || exit 1
fi

