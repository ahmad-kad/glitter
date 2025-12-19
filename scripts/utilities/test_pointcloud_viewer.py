#!/usr/bin/env python3
"""
Simple Point Cloud Viewer Test
Visualizes point cloud data to verify it's working
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import sys

class PointCloudViewer(Node):
    def __init__(self):
        super().__init__('pointcloud_test_viewer')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud',
            self.cloud_callback,
            10
        )
        self.message_count = 0
        self.get_logger().info('Waiting for point cloud data on /unilidar/cloud...')
        self.get_logger().info('Press Ctrl+C to stop')

    def cloud_callback(self, msg):
        self.message_count += 1
        
        # Extract points
        try:
            points = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True))
            
            if len(points) > 0:
                # Convert to numpy array properly
                points_list = []
                for pt in points:
                    if len(pt) >= 4:
                        points_list.append([pt[0], pt[1], pt[2], pt[3]])
                
                if len(points_list) > 0:
                    points_array = np.array(points_list)
                    
                    # Calculate statistics
                    x_min, x_max = points_array[:, 0].min(), points_array[:, 0].max()
                    y_min, y_max = points_array[:, 1].min(), points_array[:, 1].max()
                    z_min, z_max = points_array[:, 2].min(), points_array[:, 2].max()
                    intensity_min, intensity_max = points_array[:, 3].min(), points_array[:, 3].max()
                else:
                    x_min = x_max = y_min = y_max = z_min = z_max = intensity_min = intensity_max = 0
                
                print(f"\n=== Message #{self.message_count} ===")
                print(f"Frame: {msg.header.frame_id}")
                print(f"Points: {len(points)}")
                print(f"X range: [{x_min:.2f}, {x_max:.2f}] m")
                print(f"Y range: [{y_min:.2f}, {y_max:.2f}] m")
                print(f"Z range: [{z_min:.2f}, {z_max:.2f}] m")
                print(f"Intensity range: [{intensity_min:.1f}, {intensity_max:.1f}]")
                print(f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
                
                # Show sample points
                if len(points) > 0:
                    print(f"\nSample points (first 3):")
                    for i, pt in enumerate(points[:3]):
                        print(f"  Point {i+1}: x={pt[0]:.3f}, y={pt[1]:.3f}, z={pt[2]:.3f}, intensity={pt[3]:.1f}")
                
                print("\n✓ Point cloud data is valid and contains points!")
                print("  If you see this, the LiDAR is working correctly.")
                print("  The issue is only with RViz subscription.")
                
            else:
                print(f"\n⚠ Message #{self.message_count}: No valid points found")
                
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudViewer()
    
    try:
        # Process messages for a few seconds
        rclpy.spin_once(node, timeout_sec=10.0)
        rclpy.spin_once(node, timeout_sec=10.0)
        rclpy.spin_once(node, timeout_sec=10.0)
        
        if node.message_count == 0:
            print("\n✗ No messages received after 30 seconds")
            print("  Check:")
            print("  1. LiDAR driver is running")
            print("  2. Topic exists: ros2 topic list | grep cloud")
            print("  3. Data is publishing: ros2 topic hz /unilidar/cloud")
        else:
            print(f"\n✓ Successfully received {node.message_count} message(s)")
            print("  Point cloud is working! Fix RViz subscription to see it visually.")
            
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

