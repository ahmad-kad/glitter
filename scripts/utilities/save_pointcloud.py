#!/usr/bin/env python3
"""
Save Point Cloud to PCD File
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import sys
from pathlib import Path

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

class PointCloudSaver(Node):
    def __init__(self, output_file="pointcloud.pcd"):
        super().__init__('pointcloud_saver')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud',
            self.cloud_callback,
            10
        )
        self.output_file = output_file
        self.saved = False
        self.get_logger().info(f'Waiting for point cloud to save to {output_file}...')

    def cloud_callback(self, msg):
        if self.saved:
            return
            
        try:
            # Extract points
            points = list(point_cloud2.read_points(
                msg, 
                field_names=('x', 'y', 'z', 'intensity'), 
                skip_nans=True
            ))
            
            if len(points) == 0:
                self.get_logger().warn("No points in cloud")
                return
            
            # Convert to numpy array
            points_array = np.array([[pt[0], pt[1], pt[2]] for pt in points])
            
            if HAS_OPEN3D:
                # Save with Open3D
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points_array)
                
                # Add colors from intensity
                colors = np.array([[pt[3], pt[3], pt[3]] for pt in points]) / 255.0
                pcd.colors = o3d.utility.Vector3dVector(colors)
                
                o3d.io.write_point_cloud(self.output_file, pcd)
                self.get_logger().info(f"✓ Saved {len(points)} points to {self.output_file}")
            else:
                # Save as simple text file
                with open(self.output_file.replace('.pcd', '.txt'), 'w') as f:
                    f.write("# Point Cloud Data\n")
                    f.write("# Format: x y z intensity\n")
                    for pt in points:
                        f.write(f"{pt[0]} {pt[1]} {pt[2]} {pt[3]}\n")
                self.get_logger().info(f"✓ Saved {len(points)} points to {self.output_file.replace('.pcd', '.txt')}")
            
            self.saved = True
            self.get_logger().info("Done! Exiting...")
            rclpy.shutdown()
            
        except Exception as e:
            self.get_logger().error(f"Error saving point cloud: {e}")

def main(args=None):
    output_file = sys.argv[1] if len(sys.argv) > 1 else "pointcloud.pcd"
    
    rclpy.init(args=args)
    node = PointCloudSaver(output_file)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

