#!/bin/bash

# Combined launch script for RealSense 435 + Unitree L2 setup
# Starts both sensors and TF transforms for RViz visualization

set -e

echo "=== Launching RealSense 435 + Unitree L2 Combined Setup ==="

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Source workspace if it exists
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Check if RealSense camera is connected
echo "Checking for RealSense camera..."
if ! lsusb | grep -q "8086:0b07"; then
    echo "⚠ Warning: RealSense 435 not detected (USB ID: 8086:0b07)"
    echo "Please connect the camera before proceeding"
fi

# Check if L2 LiDAR is reachable
echo "Checking L2 LiDAR connectivity..."
if ! ping -c 1 -W 1 192.168.1.150 >/dev/null 2>&1 && ! ping -c 1 -W 1 192.168.1.62 >/dev/null 2>&1; then
    echo "⚠ Warning: Unitree L2 LiDAR not reachable"
    echo "Expected IPs: 192.168.1.150 or 192.168.1.62"
fi

# Create combined launch file
cat > /tmp/realsense_l2_combined.launch.py << 'EOF'
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to launch RViz'
    )

    rviz_config = DeclareLaunchArgument(
        'rviz_config', default_value='',
        description='RViz configuration file path'
    )

    # Include Unitree L2 LiDAR launch
    # Note: Adjust this path based on your actual L2 driver installation
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('unitree_lidar_ros2'),
                'launch.py'
            ])
        ])
    )

    # Include RealSense camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_pointcloud': 'true',
            'align_depth': 'true',
            'color_width': '640',
            'color_height': '480',
            'color_fps': '30.0',
            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '30.0',
            'pointcloud_texture_index': '0',
            'ordered_pc': 'false',
            'enable_sync': 'true',
            'diagnostics_period': '0.0',
            'base_frame_id': 'camera_link',
        }.items()
    )

    # Static transforms (with delay to ensure sensors are ready)
    transforms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join('/home/kaddo/glitter/config/tf/static_transforms.launch.py')
        ]),
        launch_arguments={
            'x_offset': '0.0',    # Adjust based on your physical setup
            'y_offset': '0.0',    # Adjust based on your physical setup
            'z_offset': '0.1',    # Camera 10cm above LiDAR
            'yaw_offset': '0.0',  # Adjust based on your physical setup
        }.items()
    )

    # RViz (optional)
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rviz2'),
                'launch',
                'rviz2.launch.py'
            ])
        ]),
        launch_arguments={
            'config': LaunchConfiguration('rviz_config'),
        }.items(),
        condition=LaunchConfiguration('use_rviz')
    )

    return LaunchDescription([
        use_rviz,
        rviz_config,
        lidar_launch,
        TimerAction(period=2.0, actions=[realsense_launch]),  # Delay RealSense start
        TimerAction(period=4.0, actions=[transforms]),        # Delay transforms
        TimerAction(period=6.0, actions=[rviz]),              # Delay RViz
    ])
EOF

echo "Launching combined RealSense + L2 setup..."

# Set default RViz config if not specified
RVIZ_CONFIG="${RVIZ_CONFIG:-config/realsense_l2_combined.rviz}"
export RVIZ_CONFIG

# Launch the combined system
ros2 launch /tmp/realsense_l2_combined.launch.py \
    rviz_config:="$RVIZ_CONFIG" \
    use_rviz:=true

# Cleanup
rm -f /tmp/realsense_l2_combined.launch.py
