#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
import time
from tf2_ros import Buffer, TransformListener
from utils import ColorUtils, PointCloudUtils, LoggingUtils, HAS_ROS_NUMPY
from typing import Dict, Tuple, Optional, List
from collections import defaultdict
import math

# Import constants from utils
from utils import DOWNSAMPLE_FREQUENCY


class BoundedOctreeMap:
    """
    Bounded octree for efficient point cloud mapping with memory management.

    Advantages over Open3D accumulation:
    - Bounded memory usage (configurable max points)
    - Spatial indexing for fast queries
    - Adaptive resolution based on distance
    - Better coverage through intelligent culling
    """

    def __init__(self, max_points: int = 500000, voxel_size: float = 0.05,
                 max_depth: int = 8):
        self.max_points = max_points
        self.base_voxel_size = voxel_size
        self.max_depth = max_depth

        # Octree structure: key = (depth, x, y, z), value = (points, colors, count)
        self.octree: Dict[Tuple[int, int, int, int], Tuple[List[np.ndarray], List[np.ndarray], int]] = defaultdict(
            lambda: ([], [], 0)
        )

        # Statistics
        self.total_points_added = 0
        self.current_points = 0
        self.nodes_created = 0

    def add_points(self, points: np.ndarray, colors: np.ndarray, pose: Optional[np.ndarray] = None):
        """Add points to the bounded octree with intelligent culling"""
        if pose is not None:
            # Transform points to world frame
            points_hom = np.column_stack((points, np.ones(len(points))))
            points_world = (pose @ points_hom.T).T[:, :3]
        else:
            points_world = points

        # Adaptive voxel sizing based on distance from origin
        distances = np.linalg.norm(points_world, axis=1)
        adaptive_voxel_sizes = self._compute_adaptive_voxel_sizes(distances)

        # Insert points into octree
        for i, (point, color) in enumerate(zip(points_world, colors)):
            voxel_size = adaptive_voxel_sizes[i]
            self._insert_point(point, color, voxel_size)

        self.total_points_added += len(points)

        # Memory management: cull distant/old points if over limit
        self._cull_if_needed()

    def _compute_adaptive_voxel_sizes(self, distances: np.ndarray) -> np.ndarray:
        """Compute adaptive voxel sizes based on distance (farther = larger voxels)"""
        # Linear scaling: base_size at 1m, 4x larger at 10m+
        scale_factor = np.maximum(1.0, distances / 2.0)
        scale_factor = np.minimum(scale_factor, 4.0)  # Cap at 4x
        return self.base_voxel_size * scale_factor

    def _insert_point(self, point: np.ndarray, color: np.ndarray, voxel_size: float):
        """Insert a point into the appropriate octree node"""
        # Compute voxel coordinates at multiple depths
        for depth in range(1, self.max_depth + 1):
            scale = 1.0 / (voxel_size * (2 ** (depth - 1)))
            voxel_coord = tuple(int(coord * scale) for coord in point)
            key = (depth,) + voxel_coord

            points_list, colors_list, count = self.octree[key]

            if count == 0:
                self.nodes_created += 1

            # Store point with averaging for occupied voxels
            if count < 8:  # Store up to 8 points per voxel before averaging
                points_list.append(point.copy())
                colors_list.append(color.copy())
                self.octree[key] = (points_list, colors_list, count + 1)
            else:
                # Average existing points and colors
                avg_point = np.mean(points_list, axis=0)
                avg_color = np.mean(colors_list, axis=0)
                self.octree[key] = ([avg_point], [avg_color], 1)

            self.current_points += 1

    def _cull_if_needed(self):
        """Cull points if memory limit exceeded"""
        if self.current_points <= self.max_points:
            return

        # Cull strategy: remove deepest level nodes first (lowest resolution)
        nodes_to_remove = []
        points_to_remove = 0

        # Sort nodes by depth (deepest first)
        sorted_nodes = sorted(self.octree.keys(), key=lambda x: x[0], reverse=True)

        for node_key in sorted_nodes:
            if self.current_points - points_to_remove <= self.max_points * 0.8:
                break

            depth, x, y, z = node_key
            if depth > 2:  # Don't remove root levels
                _, _, count = self.octree[node_key]
                points_to_remove += count
                nodes_to_remove.append(node_key)

        # Remove selected nodes
        for node_key in nodes_to_remove:
            del self.octree[node_key]
            self.nodes_created -= 1

        self.current_points -= points_to_remove

    def to_point_cloud(self) -> o3d.geometry.PointCloud:
        """Convert octree to Open3D point cloud for saving/visualization"""
        all_points = []
        all_colors = []

        for (points_list, colors_list, _) in self.octree.values():
            if points_list:  # Only add non-empty nodes
                all_points.extend(points_list)
                all_colors.extend(colors_list)

        if not all_points:
            # Return empty point cloud
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector([])
            cloud.colors = o3d.utility.Vector3dVector([])
            return cloud

        # Convert to numpy arrays
        points_array = np.array(all_points)
        colors_array = np.array(all_colors)

        # Create Open3D cloud
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points_array)
        cloud.colors = o3d.utility.Vector3dVector(colors_array)

        return cloud

    def get_stats(self) -> Dict:
        """Get octree statistics"""
        return {
            'total_points_added': self.total_points_added,
            'current_points': self.current_points,
            'max_points': self.max_points,
            'nodes_created': self.nodes_created,
            'memory_efficiency': self.current_points / max(1, self.total_points_added),
            'utilization': self.current_points / self.max_points
        }


class MapBuilder(Node):
    def __init__(self):
        super().__init__('map_builder')

        # Setup logging with error handling
        try:
            self.logger = LoggingUtils.setup_logging("map_builder.log")
            self.logger.info("Map Builder Node initialized")
            self.logger.info(f"ROS NumPy available: {HAS_ROS_NUMPY}")
        except Exception as e:
            self.get_logger().error(f"Failed to setup logging: {e}")
            raise

        # Declare and get parameters
        try:
            self.declare_parameter('save_path', 'colored_map.pcd')
            self.declare_parameter('target_frame', 'map')
            self.declare_parameter('voxel_size', 0.05)
            self.declare_parameter('max_points', 500000)  # Memory safety limit
            self.declare_parameter('memory_warning_threshold', 400000)  # Warning threshold

            self.save_path = self.get_parameter('save_path').value
            self.target_frame = self.get_parameter('target_frame').value
            self.voxel_size = self.get_parameter('voxel_size').value
            self.max_points = self.get_parameter('max_points').value
            self.memory_warning_threshold = self.get_parameter('memory_warning_threshold').value

            # Validate parameters
            if not isinstance(self.save_path, str) or not self.save_path:
                raise ValueError("save_path must be a non-empty string")
            if not isinstance(self.target_frame, str) or not self.target_frame:
                raise ValueError("target_frame must be a non-empty string")
            if not isinstance(self.voxel_size, (int, float)) or self.voxel_size <= 0:
                raise ValueError("voxel_size must be a positive number")
            if not isinstance(self.max_points, int) or self.max_points <= 0:
                raise ValueError("max_points must be a positive integer")
            if not isinstance(self.memory_warning_threshold, int) or self.memory_warning_threshold <= 0:
                raise ValueError("memory_warning_threshold must be a positive integer")

        except Exception as e:
            self.logger.error(f"Failed to setup parameters: {e}")
            raise

        # Setup TF buffer and listener
        try:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        except Exception as e:
            self.logger.error(f"Failed to setup TF components: {e}")
            raise

        # Setup subscriber
        try:
            self.pcd_sub = self.create_subscription(
                PointCloud2,
                '/unilidar/colored_cloud',
                self.cloud_callback,
                10
            )
        except Exception as e:
            self.logger.error(f"Failed to setup subscriber: {e}")
            raise

        # Initialize bounded octree map
        try:
            self.octree_map = BoundedOctreeMap(
                max_points=self.max_points,
                voxel_size=self.voxel_size
            )
            # Keep Open3D cloud for compatibility with saving
            self.global_pcd = o3d.geometry.PointCloud()
        except Exception as e:
            self.logger.error(f"Failed to initialize octree map: {e}")
            raise

        self.frame_count = 0
        self.total_points_accumulated = 0

        self.logger.info(f"Map Builder Started. Accumulating to {self.target_frame}...")
        self.logger.info("Press Ctrl+C to save and exit.")

    def cloud_callback(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time())
        except Exception as e:
            self.get_logger().warning(f"Could not transform to {self.target_frame}: {e}", throttle_duration_sec=2)
            return

        xyz, rgb_packed = PointCloudUtils.extract_xyz_rgb(msg)
        if len(xyz) == 0:
            return

        q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
        t = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]

        R = o3d.geometry.get_rotation_matrix_from_quaternion(q)
        xyz_transformed = (R @ xyz.T).T + t

        colors_rgb = ColorUtils.unpack_rgb(rgb_packed)
        colors = colors_rgb.astype(np.float64) / 255.0

        # Use bounded octree for intelligent point management
        pose_matrix = np.eye(4)
        pose_matrix[:3, :3] = R
        pose_matrix[:3, 3] = t

        self.octree_map.add_points(xyz_transformed, colors, pose_matrix)
        self.total_points_accumulated += len(xyz_transformed)

        # Convert octree to Open3D cloud for compatibility
        self.global_pcd = self.octree_map.to_point_cloud()

        # Periodic downsampling for final output quality
        self.frame_count += 1
        if self.frame_count % DOWNSAMPLE_FREQUENCY == 0:
            # Additional downsampling for final map quality
            if len(self.global_pcd.points) > 0:
                self.global_pcd = self.global_pcd.voxel_down_sample(voxel_size=self.voxel_size)

            # Get octree statistics
            octree_stats = self.octree_map.get_stats()

            LoggingUtils.log_frame_stats(self.logger, self.frame_count, {
                "accumulated_points": len(self.global_pcd.points),
                "total_processed": self.total_points_accumulated,
                "octree_nodes": octree_stats['nodes_created'],
                "memory_efficiency": f"{octree_stats['memory_efficiency']:.2f}",
                "octree_utilization": f"{octree_stats['utilization']:.2f}"
            }, DOWNSAMPLE_FREQUENCY)

    def save_map(self):
        self.logger.info(f"Saving map with {len(self.global_pcd.points)} points to {self.save_path}...")
        self.logger.info(f"Total frames processed: {self.frame_count}")
        self.logger.info(f"Total points processed: {self.total_points_accumulated}")

        start_time = time.time()
        success = o3d.io.write_point_cloud(self.save_path, self.global_pcd)
        save_time = time.time() - start_time

        if success:
            self.logger.info(f"Map saved successfully in {save_time:.3f}s")
        else:
            self.logger.error("Failed to save point cloud!")

def main(args=None):
    rclpy.init(args=args)
    node = MapBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_map()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()