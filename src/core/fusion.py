#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import message_filters
import numpy as np
import time
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped
import tf2_ros
from utils import (
    TransformUtils,
    PointCloudUtils,
    ColorUtils,
    LoggingUtils,
    CameraUtils,
    L2_DEFAULT_TOPIC,
    HAS_ROS_NUMPY
)

# Constants
FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]

class LidarCameraFusion(Node):
    def __init__(self):
        super().__init__('lidar_camera_fusion')

        # Setup logging with error handling
        try:
            self.logger = LoggingUtils.setup_logging("fusion.log")
            self.logger.info("LiDAR-Camera Fusion Node initialized")
            self.logger.info(f"ROS NumPy available: {HAS_ROS_NUMPY}")
        except Exception as e:
            self.get_logger().error(f"Failed to setup logging: {e}")
            raise

        # Declare parameters with L2/L1 compatibility
        try:
            default_intrinsics = CameraUtils.default_pi_camera_matrix()
            self.declare_parameter('camera_matrix', default_intrinsics.flatten().tolist())
            self.declare_parameter('extrinsic_trans', [0.0, 0.0, 0.0])
            self.declare_parameter('extrinsic_rot', [0.0, 0.0, 0.0])
            self.declare_parameter('lidar_topic', L2_DEFAULT_TOPIC)
            self.declare_parameter('geometric_prioritization', False)  # Enable edge/corner prioritization
            self.declare_parameter('max_points_per_frame', None)  # Limit points per frame
            self.declare_parameter('geometric_weight', 0.7)  # Weight for geometric features
        except Exception as e:
            self.logger.error(f"Failed to declare parameters: {e}")
            raise

        # Setup subscribers with configurable topic
        try:
            lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
            self.lidar_sub = message_filters.Subscriber(self, PointCloud2, lidar_topic)
            self.cam_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
            self.ts = message_filters.ApproximateTimeSynchronizer([self.lidar_sub, self.cam_sub], 10, 0.1)
            self.ts.registerCallback(self.fusion_callback)
        except Exception as e:
            self.logger.error(f"Failed to setup subscribers: {e}")
            raise

        # Setup publishers
        try:
            self.pub_colored_cloud = self.create_publisher(PointCloud2, '/unilidar/colored_cloud', 10)
            self.bridge = CvBridge()

            # Diagnostic and performance publishers
            self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics/fusion', 10)
            self.frame_rate_pub = self.create_publisher(Float64, '/fusion/performance/frame_rate', 10)
            self.processing_time_pub = self.create_publisher(Float64, '/fusion/performance/processing_time', 10)
            self.points_processed_pub = self.create_publisher(Float64, '/fusion/performance/points_processed', 10)

            # TF broadcaster for coordinate frames
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        except Exception as e:
            self.logger.error(f"Failed to setup publishers: {e}")
            raise

        self.frame_count = 0
        self.total_processing_time = 0.0

        # Performance tracking
        self.last_frame_time = time.time()
        self.frame_times = []
        self.processing_times = []

        self.logger.info("Fusion Node Started. Waiting for synced topics...")

    def get_transform_matrix(self):
        """Get camera intrinsic and extrinsic transformation matrices.

        Returns:
            Tuple of (K, RT) where K is 3x3 intrinsics and RT is 4x4 extrinsics

        Raises:
            RuntimeError: If parameters cannot be retrieved or are invalid
        """
        try:
            # Retrieve parameters with validation
            trans_param = self.get_parameter('extrinsic_trans')
            rot_param = self.get_parameter('extrinsic_rot')
            cam_param = self.get_parameter('camera_matrix')

            t = trans_param.get_parameter_value().double_array_value
            r = rot_param.get_parameter_value().double_array_value
            k_flat = cam_param.get_parameter_value().double_array_value

            # Validate parameter lengths
            if len(t) != 3:
                raise ValueError(f"extrinsic_trans must have 3 elements, got {len(t)}")
            if len(r) != 3:
                raise ValueError(f"extrinsic_rot must have 3 elements, got {len(r)}")
            if len(k_flat) != 9:
                raise ValueError(f"camera_matrix must have 9 elements, got {len(k_flat)}")

            K = np.array(k_flat).reshape(3, 3)
            RT = TransformUtils.build_extrinsic_matrix(t, r)

            return K, RT

        except Exception as e:
            self.logger.error(f"Failed to get transform matrices: {e}")
            raise RuntimeError(f"Parameter retrieval failed: {e}")

    def fusion_callback(self, lidar_msg, img_msg):
        frame_start_time = time.time()
        current_time = time.time()
        self.frame_count += 1

        # Calculate frame rate
        if self.frame_count > 1:
            frame_interval = current_time - self.last_frame_time
            self.frame_times.append(frame_interval)
            if len(self.frame_times) > 30:  # Keep last 30 frame intervals
                self.frame_times.pop(0)
            avg_frame_rate = 1.0 / (sum(self.frame_times) / len(self.frame_times))
        else:
            avg_frame_rate = 0.0
        self.last_frame_time = current_time

        # Performance timing variables
        timing = {}

        try:
            # Time image conversion
            img_convert_start = time.time()
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            height, width, _ = cv_image.shape
            timing['image_conversion'] = time.time() - img_convert_start
        except Exception as e:
            self.logger.error(f"CV Bridge error: {e}")
            return

        # Time parameter retrieval
        param_start = time.time()
        K, RT = self.get_transform_matrix()
        timing['parameter_retrieval'] = time.time() - param_start

        # Broadcast TF transform (camera relative to LiDAR)
        self._broadcast_tf_transform(RT, lidar_msg.header.stamp)

        # Time point cloud extraction
        pc_extract_start = time.time()
        points_np, _ = PointCloudUtils.extract_xyz_rgb(lidar_msg, ('x', 'y', 'z'))
        timing['point_cloud_extraction'] = time.time() - pc_extract_start

        if len(points_np) == 0:
            return

        # Time coordinate transformations
        coord_start = time.time()
        points_hom = PointCloudUtils.to_homogeneous(points_np)
        points_cam = (RT @ points_hom.T).T
        timing['coordinate_transform'] = time.time() - coord_start

        # Time depth filtering
        depth_start = time.time()
        depth_mask = points_cam[:, 2] > 0.1
        points_cam = points_cam[depth_mask]
        points_3d = points_np[depth_mask]
        timing['depth_filtering'] = time.time() - depth_start

        if len(points_cam) == 0:
            return

        # Optional geometric prioritization (prioritize edges/corners)
        geom_start = time.time()
        try:
            geometric_prioritization = self.get_parameter('geometric_prioritization').get_parameter_value().bool_value
            if geometric_prioritization:
                max_points = self.get_parameter('max_points_per_frame').get_parameter_value().integer_value
                geometric_weight = self.get_parameter('geometric_weight').get_parameter_value().double_value

                # Apply geometric prioritization to select most valuable points
                points_3d = TransformUtils.prioritize_points_by_geometry(
                    points_3d, cv_image, K, RT,
                    max_points=max_points if max_points > 0 else None,
                    geometric_weight=geometric_weight
                )

                # Re-project selected points to camera coordinates
                points_hom_selected = PointCloudUtils.to_homogeneous(points_3d)
                points_cam = (RT @ points_hom_selected.T).T

                self.logger.debug(f"Geometric prioritization: selected {len(points_3d)} high-value points")

        except Exception as e:
            self.logger.warning(f"Geometric prioritization failed, using all points: {e}")

        timing['geometric_prioritization'] = time.time() - geom_start

        # Time 2D projection
        proj_start = time.time()
        u, v = TransformUtils.project_3d_to_2d(points_cam[:, :3], K, np.eye(4))
        bounds_mask = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        u_valid, v_valid = u[bounds_mask], v[bounds_mask]
        points_final = points_3d[bounds_mask]
        timing['projection_2d'] = time.time() - proj_start

        # Time color sampling
        color_start = time.time()
        colors = cv_image[v_valid.astype(int), u_valid.astype(int)]
        rgb_float = ColorUtils.pack_rgb(colors)
        output_data = np.column_stack((points_final, rgb_float))
        timing['color_sampling'] = time.time() - color_start

        colored_msg = point_cloud2.create_cloud(lidar_msg.header, FIELDS, output_data)
        self.pub_colored_cloud.publish(colored_msg)

        # Performance tracking
        total_processing_time = time.time() - frame_start_time
        self.total_processing_time += total_processing_time
        self.processing_times.append(total_processing_time)
        if len(self.processing_times) > 30:  # Keep last 30 processing times
            self.processing_times.pop(0)

        # Publish performance metrics
        self.frame_rate_pub.publish(Float64(data=avg_frame_rate))
        self.processing_time_pub.publish(Float64(data=total_processing_time))
        self.points_processed_pub.publish(Float64(data=len(points_final)))

        # Publish diagnostic information
        self._publish_diagnostics(points_np, points_3d, points_final, total_processing_time, avg_frame_rate)

        # Log detailed timing statistics every 30 frames
        if self.frame_count % 30 == 0:
            total_timing = sum(timing.values())
            self.logger.info(f"Pipeline Timing Breakdown (Frame {self.frame_count}):")
            for stage, duration in timing.items():
                percentage = (duration / total_timing) * 100
                self.logger.info(f"  {stage}: {duration:.3f}ms ({percentage:.1f}%)")

        # Log statistics every 30 frames
        LoggingUtils.log_frame_stats(self.logger, self.frame_count, {
            "input_points": len(points_np),
            "visible": len(points_3d),
            "colored": len(points_final),
            "total_time_ms": total_processing_time * 1000
        })

    def _publish_diagnostics(self, input_points: np.ndarray, visible_points: np.ndarray,
                           colored_points: np.ndarray, processing_time: float, frame_rate: float):
        """Publish diagnostic information for monitoring."""
        try:
            # Create diagnostic array
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()

            # Main fusion status
            fusion_status = DiagnosticStatus()
            fusion_status.name = "LiDAR-Camera Fusion"
            fusion_status.hardware_id = "fusion_pipeline"

            # Determine status level
            if processing_time > 0.1:  # Slow processing
                fusion_status.level = DiagnosticStatus.WARN
                fusion_status.message = "Processing time above 100ms threshold"
            elif len(colored_points) < 100:  # Few visible points
                fusion_status.level = DiagnosticStatus.WARN
                fusion_status.message = "Low point count in fused output"
            elif frame_rate < 5.0:  # Low frame rate
                fusion_status.level = DiagnosticStatus.WARN
                fusion_status.message = "Frame rate below 5 Hz"
            else:
                fusion_status.level = DiagnosticStatus.OK
                fusion_status.message = "Fusion operating normally"

            # Add key-value pairs
            fusion_status.values = [
                KeyValue(key="frame_count", value=str(self.frame_count)),
                KeyValue(key="input_points", value=str(len(input_points))),
                KeyValue(key="visible_points", value=str(len(visible_points))),
                KeyValue(key="colored_points", value=str(len(colored_points))),
                KeyValue(key="processing_time_ms", value=f"{processing_time * 1000:.1f}"),
                KeyValue(key="frame_rate_hz", value=f"{frame_rate:.1f}"),
                KeyValue(key="point_efficiency", value=f"{len(colored_points) / max(len(input_points), 1) * 100:.1f}%"),
            ]

            diag_array.status = [fusion_status]
            self.diag_pub.publish(diag_array)

        except Exception as e:
            self.logger.warning(f"Failed to publish diagnostics: {e}")

    def _broadcast_tf_transform(self, RT: np.ndarray, timestamp):
        """Broadcast TF transform from LiDAR to camera frame."""
        try:
            # Extract rotation matrix and translation from 4x4 transform
            R = RT[:3, :3]
            t = RT[:3, 3]

            # Convert rotation matrix to quaternion
            # This is a simplified conversion - in production you'd want a more robust method
            qw = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2
            qx = (R[2,1] - R[1,2]) / (4 * qw)
            qy = (R[0,2] - R[2,0]) / (4 * qw)
            qz = (R[1,0] - R[0,1]) / (4 * qw)

            # Create transform message
            transform = TransformStamped()
            transform.header.stamp = timestamp
            transform.header.frame_id = "livox_frame"  # LiDAR frame
            transform.child_frame_id = "camera_frame"  # Camera frame

            transform.transform.translation.x = float(t[0])
            transform.transform.translation.y = float(t[1])
            transform.transform.translation.z = float(t[2])

            transform.transform.rotation.x = float(qx)
            transform.transform.rotation.y = float(qy)
            transform.transform.rotation.z = float(qz)
            transform.transform.rotation.w = float(qw)

            self.tf_broadcaster.sendTransform(transform)

        except Exception as e:
            self.logger.warning(f"Failed to broadcast TF transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()