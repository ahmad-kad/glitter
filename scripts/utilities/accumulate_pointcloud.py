#!/usr/bin/env python3
"""
Point Cloud Accumulator for Room Reconstruction
Accumulates multiple LiDAR scans into a single point cloud
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import time
from collections import deque
import sys

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

class PointCloudAccumulator(Node):
    def __init__(self, max_points=1000000, decay_time=60.0, voxel_size=0.05):
        super().__init__('pointcloud_accumulator')
        
        # Parameters
        self.max_points = max_points
        self.decay_time = decay_time  # seconds
        self.voxel_size = voxel_size  # meters for downsampling
        
        # Storage
        self.accumulated_points = []  # List of (points, timestamp)
        self.total_points = 0
        
        # Subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud',
            self.cloud_callback,
            10
        )
        
        # Publisher for accumulated cloud
        self.publisher = self.create_publisher(
            PointCloud2,
            '/unilidar/accumulated_cloud',
            10
        )
        
        # Timer to publish accumulated cloud
        self.create_timer(1.0, self.publish_accumulated)  # Publish every second
        
        # Timer to clean old points
        self.create_timer(5.0, self.clean_old_points)  # Clean every 5 seconds
        
        self.get_logger().info('Point Cloud Accumulator started')
        self.get_logger().info(f'Max points: {self.max_points:,}')
        self.get_logger().info(f'Decay time: {self.decay_time}s')
        self.get_logger().info(f'Voxel size: {self.voxel_size}m')
        self.get_logger().info('Accumulating scans... Press Ctrl+C to save')

    def cloud_callback(self, msg):
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
            points_array = np.array([[pt[0], pt[1], pt[2], pt[3]] for pt in points])
            
            # Store with timestamp
            current_time = time.time()
            self.accumulated_points.append((points_array, current_time))
            self.total_points += len(points_array)
            
            # Log progress
            if len(self.accumulated_points) % 10 == 0:
                self.get_logger().info(
                    f'Accumulated {len(self.accumulated_points)} scans, '
                    f'{self.total_points:,} total points'
                )
            
            # Limit total points
            if self.total_points > self.max_points:
                self.downsample()
                
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def clean_old_points(self):
        """Remove points older than decay_time"""
        current_time = time.time()
        removed_scans = 0
        
        # Filter out old points
        self.accumulated_points = [
            (pts, ts) for pts, ts in self.accumulated_points
            if (current_time - ts) < self.decay_time
        ]
        
        # Recalculate total
        self.total_points = sum(len(pts) for pts, _ in self.accumulated_points)
        
        if removed_scans > 0:
            self.get_logger().info(f'Removed {removed_scans} old scans')

    def downsample(self):
        """Downsample accumulated points using voxel grid"""
        if not HAS_OPEN3D:
            # Simple downsampling: keep every Nth point
            keep_every = max(1, self.total_points // self.max_points)
            for i, (pts, ts) in enumerate(self.accumulated_points):
                self.accumulated_points[i] = (pts[::keep_every], ts)
            self.total_points = sum(len(pts) for pts, _ in self.accumulated_points)
            self.get_logger().info(f'Downsampled to {self.total_points:,} points')
            return
        
        try:
            # Combine all points
            all_points = np.vstack([pts[:, :3] for pts, _ in self.accumulated_points])
            all_intensities = np.hstack([pts[:, 3] for pts, _ in self.accumulated_points])
            
            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(all_points)
            
            # Voxel downsampling
            pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
            
            # Get downsampled points
            downsampled_points = np.asarray(pcd.points)
            
            # Rebuild with intensities (approximate)
            # For simplicity, just keep first point from each scan
            self.accumulated_points = [(pts[:len(downsampled_points)//len(self.accumulated_points)], ts) 
                                     for pts, ts in self.accumulated_points[:len(self.accumulated_points)//2]]
            
            self.total_points = sum(len(pts) for pts, _ in self.accumulated_points)
            self.get_logger().info(f'Downsampled to {self.total_points:,} points')
            
        except Exception as e:
            self.get_logger().warn(f'Downsampling failed: {e}')

    def publish_accumulated(self):
        """Publish accumulated point cloud"""
        if len(self.accumulated_points) == 0:
            return
        
        try:
            # Combine all points
            all_points = np.vstack([pts for pts, _ in self.accumulated_points])
            
            # Create PointCloud2 message
            fields = [
                point_cloud2.PointField(name='x', offset=0, datatype=7, count=1),
                point_cloud2.PointField(name='y', offset=4, datatype=7, count=1),
                point_cloud2.PointField(name='z', offset=8, datatype=7, count=1),
                point_cloud2.PointField(name='intensity', offset=16, datatype=7, count=1),
            ]
            
            # Convert to list of tuples
            points_list = [(pt[0], pt[1], pt[2], pt[3]) for pt in all_points]
            
            cloud_msg = point_cloud2.create_cloud_xyz32(
                self.get_clock().now().to_msg(),
                [(pt[0], pt[1], pt[2]) for pt in all_points]
            )
            
            # Add intensity field manually
            # For simplicity, publish as xyz only for now
            cloud_msg.header.frame_id = 'unilidar_lidar'
            cloud_msg.width = len(all_points)
            cloud_msg.height = 1
            
            self.publisher.publish(cloud_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing accumulated cloud: {e}')

    def save_accumulated(self, filename='accumulated_room.pcd'):
        """Save accumulated point cloud to file"""
        if len(self.accumulated_points) == 0:
            self.get_logger().warn('No points to save')
            return False
        
        try:
            # Combine all points
            all_points = np.vstack([pts[:, :3] for pts, _ in self.accumulated_points])
            all_intensities = np.hstack([pts[:, 3] for pts, _ in self.accumulated_points])
            
            if HAS_OPEN3D:
                # Save with Open3D
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(all_points)
                
                # Normalize intensities to 0-1 for colors
                colors = np.column_stack([
                    all_intensities / 255.0,
                    all_intensities / 255.0,
                    all_intensities / 255.0
                ])
                pcd.colors = o3d.utility.Vector3dVector(colors)
                
                o3d.io.write_point_cloud(filename, pcd)
                self.get_logger().info(f'✓ Saved {len(all_points):,} points to {filename}')
            else:
                # Save as text file
                txt_file = filename.replace('.pcd', '.txt')
                with open(txt_file, 'w') as f:
                    f.write("# Accumulated Point Cloud\n")
                    f.write("# Format: x y z intensity\n")
                    for i, pt in enumerate(all_points):
                        f.write(f"{pt[0]} {pt[1]} {pt[2]} {all_intensities[i]}\n")
                self.get_logger().info(f'✓ Saved {len(all_points):,} points to {txt_file}')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error saving point cloud: {e}')
            return False

def main(args=None):
    import argparse
    parser = argparse.ArgumentParser(description='Accumulate LiDAR point clouds for room reconstruction')
    parser.add_argument('--max-points', type=int, default=1000000, help='Maximum points to accumulate')
    parser.add_argument('--decay-time', type=float, default=60.0, help='Point decay time in seconds')
    parser.add_argument('--voxel-size', type=float, default=0.05, help='Voxel size for downsampling (meters)')
    parser.add_argument('--output', type=str, default='accumulated_room.pcd', help='Output filename')
    args = parser.parse_args()
    
    rclpy.init(args=None)
    node = PointCloudAccumulator(
        max_points=args.max_points,
        decay_time=args.decay_time,
        voxel_size=args.voxel_size
    )
    
    try:
        print("\n" + "="*50)
        print("Point Cloud Accumulator Running")
        print("="*50)
        print(f"Accumulating scans...")
        print(f"Press Ctrl+C to save and exit")
        print("="*50 + "\n")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n\nSaving accumulated point cloud...")
        node.save_accumulated(args.output)
        print(f"Saved to: {args.output}")
        
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

