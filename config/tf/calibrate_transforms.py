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
