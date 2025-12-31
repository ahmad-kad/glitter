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
