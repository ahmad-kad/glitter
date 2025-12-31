#!/bin/bash

# Setup TF coordinate frames for RealSense 435 + Unitree L2 alignment
# This creates static transforms between camera and LiDAR coordinate frames

set -e

echo "=== Setting up TF Coordinate Frames ==="

# Create TF configuration directory
mkdir -p config/tf

# Create TF launch file for static transforms
cat > config/tf/static_transforms.launch.py << 'EOF'
#!/usr/bin/env python3

import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare transform parameters
    # These represent the physical offset between RealSense camera and L2 LiDAR
    # Adjust these values based on your physical setup

    # Translation from L2 LiDAR to RealSense camera (in meters)
    x_offset = DeclareLaunchArgument(
        'x_offset', default_value='0.0',  # Forward/backward
        description='X offset from L2 LiDAR to camera (meters)'
    )

    y_offset = DeclareLaunchArgument(
        'y_offset', default_value='0.0',  # Left/right
        description='Y offset from L2 LiDAR to camera (meters)'
    )

    z_offset = DeclareLaunchArgument(
        'z_offset', default_value='0.1',  # Up/down (10cm above LiDAR)
        description='Z offset from L2 LiDAR to camera (meters)'
    )

    # Rotation from L2 LiDAR to RealSense camera (in degrees, then converted to radians)
    roll_offset = DeclareLaunchArgument(
        'roll_offset', default_value='0.0',  # Rotation around X
        description='Roll offset from L2 LiDAR to camera (degrees)'
    )

    pitch_offset = DeclareLaunchArgument(
        'pitch_offset', default_value='0.0',  # Rotation around Y
        description='Pitch offset from L2 LiDAR to camera (degrees)'
    )

    yaw_offset = DeclareLaunchArgument(
        'yaw_offset', default_value='0.0',  # Rotation around Z
        description='Yaw offset from L2 LiDAR to camera (degrees)'
    )

    # Static transform publisher: L2 LiDAR frame -> Camera base frame
    lidar_to_camera_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_to_camera_broadcaster',
        arguments=[
            '--x', LaunchConfiguration('x_offset'),
            '--y', LaunchConfiguration('y_offset'),
            '--z', LaunchConfiguration('z_offset'),
            '--roll', LaunchConfiguration('roll_offset'),
            '--pitch', LaunchConfiguration('pitch_offset'),
            '--yaw', LaunchConfiguration('yaw_offset'),
            '--frame-id', 'unilidar_lidar',  # Parent frame (L2 LiDAR)
            '--child-frame-id', 'camera_link'  # Child frame (RealSense base)
        ],
        parameters=[{
            'use_sim_time': False
        }]
    )

    # Optional: Transform from world/map frame to L2 LiDAR frame
    # Uncomment and adjust if you need a world reference frame
    # world_to_lidar_transform = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='world_to_lidar_broadcaster',
    #     arguments=[
    #         '--x', '0.0',
    #         '--y', '0.0',
    #         '--z', '0.0',
    #         '--roll', '0.0',
    #         '--pitch', '0.0',
    #         '--yaw', '0.0',
    #         '--frame-id', 'map',  # World frame
    #         '--child-frame-id', 'unilidar_lidar'  # L2 LiDAR frame
    #     ]
    # )

    return LaunchDescription([
        x_offset, y_offset, z_offset,
        roll_offset, pitch_offset, yaw_offset,
        lidar_to_camera_transform,
        # world_to_lidar_transform,  # Uncomment if needed
    ])
EOF

echo "✓ TF static transforms launch file created"

# Make launch file executable
chmod +x config/tf/static_transforms.launch.py

# Create a calibration helper script
cat > config/tf/calibrate_transforms.py << 'EOF'
#!/usr/bin/env python3
"""
Helper script to determine transform between RealSense camera and L2 LiDAR.
Run this after both sensors are publishing data to measure their relative positions.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation

class TransformCalibrator(Node):
    def __init__(self):
        super().__init__('transform_calibrator')

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/unilidar/cloud', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.camera_callback, 10)

        self.lidar_received = False
        self.camera_received = False

        self.get_logger().info("Transform calibrator started. Waiting for sensor data...")

    def lidar_callback(self, msg):
        if not self.lidar_received:
            self.get_logger().info("✓ L2 LiDAR data received")
            self.lidar_received = True
        self.check_calibration_ready()

    def camera_callback(self, msg):
        if not self.camera_received:
            self.get_logger().info("✓ RealSense camera data received")
            self.camera_received = True
        self.check_calibration_ready()

    def check_calibration_ready(self):
        if self.lidar_received and self.camera_received:
            self.get_logger().info("Both sensors are publishing data.")
            self.get_logger().info("To calibrate transforms:")
            self.get_logger().info("1. Place a known object in view of both sensors")
            self.get_logger().info("2. Note the position in both coordinate frames")
            self.get_logger().info("3. Update the transform parameters in static_transforms.launch.py")
            self.get_logger().info("4. Or use manual measurement of sensor positions")

def main(args=None):
    rclpy.init(args=args)
    calibrator = TransformCalibrator()

    try:
        rclpy.spin(calibrator)
    except KeyboardInterrupt:
        pass
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

echo "✓ TF calibration helper script created"

# Make calibration script executable
chmod +x config/tf/calibrate_transforms.py

# Create configuration documentation
cat > config/tf/README.md << 'EOF'
# TF Coordinate Frame Setup

This directory contains the coordinate frame transformations for aligning RealSense camera and Unitree L2 LiDAR data.

## Coordinate Frames

- `map` / `odom`: World reference frame (optional)
- `unilidar_lidar`: Unitree L2 LiDAR base frame
- `camera_link`: RealSense camera base frame
- `camera_color_frame`: RealSense color camera optical frame
- `camera_depth_frame`: RealSense depth camera optical frame

## Transform Parameters

The static transform from L2 LiDAR to RealSense camera is defined by:

- **Translation** (x, y, z): Physical offset between sensor centers
- **Rotation** (roll, pitch, yaw): Angular offset between sensor orientations

### Default Values
- x_offset: 0.0m (forward/backward)
- y_offset: 0.0m (left/right)
- z_offset: 0.1m (camera 10cm above LiDAR)
- roll/pitch/yaw: 0.0° (aligned orientations)

## Calibration Process

1. **Measure Physical Setup**: Determine real-world distance and angle between sensors
2. **Run Sensors**: Start both L2 LiDAR and RealSense camera
3. **Use Calibration Script**: `ros2 run calibrate_transforms.py` to verify data flow
4. **Visual Inspection**: Check alignment in RViz
5. **Fine-tune**: Adjust transform parameters as needed

## Usage

```bash
# Launch static transforms
ros2 launch config/tf/static_transforms.launch.py

# With custom offsets (example)
ros2 launch config/tf/static_transforms.launch.py \
  x_offset:=0.2 \
  y_offset:=0.0 \
  z_offset:=0.15 \
  yaw_offset:=5.0
```

## Troubleshooting

- **No TF data**: Check that static_transform_publisher is running
- **Misaligned data**: Verify physical measurements and transform parameters
- **Frame not found**: Ensure both sensors are publishing TF data
EOF

echo "✓ TF configuration documentation created"

echo ""
echo "=== TF Setup Complete ==="
echo "Files created:"
echo "  - config/tf/static_transforms.launch.py"
echo "  - config/tf/calibrate_transforms.py"
echo "  - config/tf/README.md"
echo ""
echo "Next steps:"
echo "1. Measure physical offset between RealSense and L2 LiDAR"
echo "2. Update transform parameters in static_transforms.launch.py"
echo "3. Launch: ros2 launch config/tf/static_transforms.launch.py"
