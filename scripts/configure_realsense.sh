#!/bin/bash

# Configure RealSense 435 camera for optimal performance with L2 LiDAR
# Sets appropriate resolution, frame rates, and depth settings

set -e

echo "=== Configuring RealSense 435 Camera ==="

# Check if RealSense camera is connected
echo "Checking for RealSense camera..."
if lsusb | grep -q "8086:0b07"; then
    echo "✓ RealSense 435 detected"
else
    echo "⚠ RealSense 435 not detected. Please connect the camera and try again."
    echo "Expected USB ID: 8086:0b07 (Intel Corp.)"
    exit 1
fi

# Test basic camera functionality
echo "Testing camera with realsense-viewer..."
timeout 5 realsense-viewer --no-gui || true

# Create RealSense launch configuration
echo "Creating RealSense launch configuration..."

# Create config directory if it doesn't exist
mkdir -p config/realsense

# Create RealSense parameters file
cat > config/realsense/realsense_params.yaml << 'EOF'
/camera/camera:
  ros__parameters:
    # Device configuration
    serial_no: ''  # Use first available camera
    usb_port_id: ''  # Auto-detect
    device_type: ''  # Auto-detect

    # Color stream configuration
    enable_color: true
    color_width: 640
    color_height: 480
    color_fps: 30.0
    color_format: 'RGB8'

    # Depth stream configuration
    enable_depth: true
    depth_width: 640
    depth_height: 480
    depth_fps: 30.0

    # Infrared streams (for depth computation)
    enable_infra1: false
    enable_infra2: false

    # Point cloud configuration
    enable_pointcloud: true
    pointcloud_texture_index: 0  # Use color texture
    ordered_pc: false  # Unordered point cloud for better performance

    # Alignment
    align_depth: true  # Align depth to color frame
    align_depth_filter_size: 1

    # Filters
    enable_decimation_filter: false
    enable_hdr_merge: false
    enable_sequence_id_filter: false
    enable_spatial_filter: false
    enable_temporal_filter: false
    enable_hole_filling_filter: false

    # Advanced settings
    json_file_path: ''  # Use default settings

    # TF frames
    base_frame_id: 'camera_link'
    depth_frame_id: 'camera_depth_frame'
    color_frame_id: 'camera_color_frame'
    infra1_frame_id: 'camera_infra1_frame'
    infra2_frame_id: 'camera_infra2_frame'
    pointcloud_frame_id: 'camera_depth_optical_frame'

    # Performance settings
    enable_sync: true  # Synchronize color and depth
    enable_auto_exposure: true

    # Diagnostic settings
    diagnostics_period: 0.0  # Disable diagnostics for performance

    # QoS settings for ROS2
    qos_sensor_data: 0  # SYSTEM_DEFAULT
    qos_history_depth: 5
EOF

echo "✓ RealSense parameters configured"

# Create launch file for RealSense
cat > config/realsense/realsense_launch.py << 'EOF'
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    serial_no = DeclareLaunchArgument(
        'serial_no', default_value='',
        description='RealSense camera serial number'
    )

    base_frame_id = DeclareLaunchArgument(
        'base_frame_id', default_value='camera_link',
        description='Base frame ID for camera'
    )

    # Include the main realsense launch file
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'serial_no': LaunchConfiguration('serial_no'),
            'base_frame_id': LaunchConfiguration('base_frame_id'),
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
        }.items()
    )

    return LaunchDescription([
        serial_no,
        base_frame_id,
        realsense_launch,
    ])
EOF

echo "✓ RealSense launch file created"

# Make launch file executable
chmod +x config/realsense/realsense_launch.py

echo ""
echo "=== RealSense Configuration Complete ==="
echo "Configuration files created:"
echo "  - config/realsense/realsense_params.yaml"
echo "  - config/realsense/realsense_launch.py"
echo ""
echo "To test the camera:"
echo "  ros2 launch config/realsense/realsense_launch.py"
echo ""
echo "Expected topics:"
echo "  - /camera/color/image_raw (640x480 @ 30fps)"
echo "  - /camera/depth/image_rect_raw (640x480 @ 30fps)"
echo "  - /camera/depth/color/points (aligned point cloud)"
echo "  - /tf (transform tree)"
