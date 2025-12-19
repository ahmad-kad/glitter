#!/usr/bin/env python3
"""
Open3D Point Cloud Visualizer
Alternative to RViz for viewing point clouds
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import sys

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("Open3D not installed. Install with: pip install open3d")
    sys.exit(1)

class Open3DViewer(Node):
    def __init__(self):
        super().__init__('open3d_viewer')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud',
            self.cloud_callback,
            10
        )
        self.pcd = None
        self.vis = None
        self.message_count = 0
        self.get_logger().info('Waiting for point cloud data...')
        self.get_logger().info('Press Ctrl+C to stop')

    def cloud_callback(self, msg):
        self.message_count += 1
        
        try:
            # Extract points
            points = list(point_cloud2.read_points(
                msg, 
                field_names=('x', 'y', 'z', 'intensity'), 
                skip_nans=True
            ))
            
            if len(points) == 0:
                return
            
            # Convert to numpy array
            points_array = np.array([[pt[0], pt[1], pt[2]] for pt in points])
            colors_array = np.array([[pt[3], pt[3], pt[3]] for pt in points]) / 255.0  # Normalize intensity to 0-1
            
            # Create Open3D point cloud
            self.pcd = o3d.geometry.PointCloud()
            self.pcd.points = o3d.utility.Vector3dVector(points_array)
            self.pcd.colors = o3d.utility.Vector3dVector(colors_array)
            
            print(f"\n=== Message #{self.message_count} ===")
            print(f"Points: {len(points)}")
            print(f"Frame: {msg.header.frame_id}")
            
            # Update or create visualization
            if self.vis is None:
                self.vis = o3d.visualization.Visualizer()
                self.vis.create_window("LiDAR Point Cloud Viewer", width=1280, height=720)
                self.vis.add_geometry(self.pcd)
                
                # Set view
                view_ctl = self.vis.get_view_control()
                view_ctl.set_front([0, 0, -1])
                view_ctl.set_lookat([0, 0, 0])
                view_ctl.set_up([0, -1, 0])
                view_ctl.set_zoom(0.5)
                
                print("\n✓ Open3D viewer opened!")
                print("Controls:")
                print("  - Mouse drag: Rotate")
                print("  - Shift + Mouse drag: Pan")
                print("  - Scroll: Zoom")
                print("  - Close window to exit")
            else:
                self.vis.update_geometry(self.pcd)
            
            # Update visualization
            self.vis.poll_events()
            self.vis.update_renderer()
            
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")

def main(args=None):
    if not HAS_OPEN3D:
        print("Please install Open3D: pip install open3d")
        return
    
    rclpy.init(args=args)
    node = Open3DViewer()
    
    try:
        # Keep running until window is closed
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.vis is not None:
                if not node.vis.poll_events():
                    break
                node.vis.update_renderer()
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        if node.vis is not None:
            node.vis.destroy_window()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

