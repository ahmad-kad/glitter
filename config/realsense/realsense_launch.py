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
