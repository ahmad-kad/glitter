#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
import time
from tf2_ros import Buffer, TransformListener
from utils import ColorUtils, PointCloudUtils, LoggingUtils, HAS_ROS_NUMPY

# Import constants from utils
from utils import DOWNSAMPLE_FREQUENCY

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

            self.save_path = self.get_parameter('save_path').value
            self.target_frame = self.get_parameter('target_frame').value
            self.voxel_size = self.get_parameter('voxel_size').value

            # Validate parameters
            if not isinstance(self.save_path, str) or not self.save_path:
                raise ValueError("save_path must be a non-empty string")
            if not isinstance(self.target_frame, str) or not self.target_frame:
                raise ValueError("target_frame must be a non-empty string")
            if not isinstance(self.voxel_size, (int, float)) or self.voxel_size <= 0:
                raise ValueError("voxel_size must be a positive number")

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

        # Initialize point cloud storage
        try:
            self.global_pcd = o3d.geometry.PointCloud()
        except Exception as e:
            self.logger.error(f"Failed to initialize point cloud: {e}")
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

        new_cloud = o3d.geometry.PointCloud()
        new_cloud.points = o3d.utility.Vector3dVector(xyz_transformed)
        new_cloud.colors = o3d.utility.Vector3dVector(colors)

        self.global_pcd += new_cloud
        self.total_points_accumulated += len(new_cloud.points)

        self.frame_count += 1
        if self.frame_count % DOWNSAMPLE_FREQUENCY == 0:
            self.global_pcd = self.global_pcd.voxel_down_sample(voxel_size=self.voxel_size)
            LoggingUtils.log_frame_stats(self.logger, self.frame_count, {
                "accumulated_points": len(self.global_pcd.points),
                "total_processed": self.total_points_accumulated
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