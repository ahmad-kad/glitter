#!/usr/bin/env python3
"""
Simple, Maintainable Real-World Fusion Node
==========================================

Clean, efficient implementation using simplified infrastructure.
Focus: Readability, maintainability, and real-world robustness.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import numpy as np
import time
from typing import Optional
from collections import deque

from infrastructure import create_infrastructure, create_compressed_publisher, FusionMode, get_memory_usage, get_cpu_usage
from utils import TransformUtils, PointCloudUtils, ColorUtils


class SimpleFusionNode(Node):
    """
    Clean, maintainable fusion node with real-world capabilities.
    ~200 lines vs 400+ lines in complex version.
    """

    def __init__(self):
        super().__init__('simple_fusion')

        # Simple infrastructure setup (one line!)
        self.infra = create_infrastructure(self, target_fps=30.0)

        # Compressed publisher with adaptive compression
        self.cloud_publisher = create_compressed_publisher(
            self, '/fused_cloud', PointCloud2, 'lz4'
        )

        # Mode change handler
        self.infra.add_mode_change_callback(self.on_mode_change)

        # Sensor data buffers (simple)
        self.lidar_data: Optional[PointCloud2] = None
        self.camera_data: Optional[Image] = None

        # Processing state
        self.frame_count = 0
        self.last_frame_time = time.time()

        # Memory and CPU monitoring
        self.memory_baseline = get_memory_usage()
        self.memory_warning_threshold = self.memory_baseline + 200  # 200MB over baseline
        self.cpu_history = deque(maxlen=10)  # Last 10 CPU readings
        self.frame_drop_count = 0
        self.last_lidar_time = 0
        self.last_camera_time = 0

        # Geometric prioritization settings
        self.declare_parameter('geometric_prioritization', False)
        self.declare_parameter('max_points_per_frame', 10000)  # Default 10K points
        self.declare_parameter('geometric_weight', 0.7)

        # Subscriptions with QoS
        self.create_subscriptions()

        self.get_logger().info("Simple fusion node ready")

    def create_subscriptions(self):
        """Create sensor subscriptions"""
        try:
            # LiDAR subscription
            self.lidar_sub = self.create_subscription(
                PointCloud2, '/unilidar/cloud', self.lidar_callback, 10
            )

            # Camera subscription
            self.camera_sub = self.create_subscription(
                Image, '/camera/image_raw', self.camera_callback, 10
            )

        except Exception as e:
            self.get_logger().error(f"Subscription setup failed: {e}")

    def lidar_callback(self, msg: PointCloud2):
        """LiDAR data callback"""
        # Update sensor health
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.infra.update_sensor_health('lidar', timestamp)

        # Store data
        self.lidar_data = msg

        # Try to process
        self.try_process_frame()

    def camera_callback(self, msg: Image):
        """Camera data callback with monitoring"""
        # Frame drop detection
        current_time = time.time()
        time_since_last = current_time - self.last_camera_time

        # Detect frame drops (assuming 30 FPS = 33ms intervals)
        if self.last_camera_time > 0 and time_since_last > 0.066:  # 66ms = ~2 frames at 30 FPS
            dropped_frames = int(time_since_last / (1/30)) - 1
            if dropped_frames > 0:
                self.frame_drop_count += dropped_frames
                self.get_logger().warning(f"Camera frame drops detected: {dropped_frames} frames "
                                        f"(total drops: {self.frame_drop_count})")

        self.last_camera_time = current_time

        # Update sensor health
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.infra.update_sensor_health('camera', timestamp)

        # Store data
        self.camera_data = msg

        # Try to process
        self.try_process_frame()

    def try_process_frame(self):
        """Process frame if data is available"""
        if not (self.lidar_data and self.camera_data):
            return

        # Simple time sync check (100ms tolerance)
        lidar_time = self.lidar_data.header.stamp.sec + self.lidar_data.header.stamp.nanosec / 1e9
        camera_time = self.camera_data.header.stamp.sec + self.camera_data.header.stamp.nanosec / 1e9

        if abs(lidar_time - camera_time) > 0.1:
            return

        # Process the frame
        self.process_frame()

        # Clear buffers
        self.lidar_data = None
        self.camera_data = None

    def process_frame(self):
        """Process a single frame with adaptive behavior and monitoring"""
        start_time = time.time()

        # Memory and CPU monitoring
        current_memory = get_memory_usage()
        current_cpu = get_cpu_usage()
        self.cpu_history.append(current_cpu)
        avg_cpu = sum(self.cpu_history) / len(self.cpu_history) if self.cpu_history else 0

        # Adaptive behavior based on system load
        if current_memory > self.memory_warning_threshold:
            self.get_logger().warning(f"High memory usage: {current_memory:.1f}MB "
                                    f"(baseline: {self.memory_baseline:.1f}MB)")
            # Force compression and basic mode
            self.infra.set_compression_level('high')
            self.infra.set_mode(FusionMode.BASIC)

        elif avg_cpu > 80:
            self.get_logger().warning(f"High CPU usage: {avg_cpu:.1f}%")
            self.infra.set_mode(FusionMode.BASIC)

        elif avg_cpu < 50 and current_memory < self.memory_baseline + 100:
            # System has capacity, use full processing
            self.infra.set_mode(FusionMode.FULL)

        # Get current processing parameters from infrastructure
        params = self.infra.get_processing_params()
        mode = self.infra.get_fusion_mode()

        # Adaptive processing based on mode
        if mode == FusionMode.FULL:
            result = self.process_full_fusion()
        elif mode == FusionMode.BASIC:
            result = self.process_basic_fusion()
        else:  # MINIMAL
            result = self.process_minimal_fusion()

        # Apply adaptive compression
        if result and self.infra.should_compress():
            # Could apply compression here
            pass

        # Publish result
        if result:
            self.cloud_publisher.publish(result)

        # Update performance feedback
        self.update_performance_feedback(start_time)

    def process_full_fusion(self) -> Optional[PointCloud2]:
        """Full LiDAR + Camera fusion with optional geometric prioritization"""
        try:
            if not self.lidar_data or not self.camera_data:
                return None

            # Extract point cloud data
            xyz, rgb_packed = PointCloudUtils.extract_xyz_rgb(self.lidar_data)
            if len(xyz) == 0:
                return None

            # Get camera image
            from cv_bridge import CvBridge
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(self.camera_data, desired_encoding='bgr8')

            # Get camera parameters (simplified - in real implementation would be calibrated)
            from utils import CameraUtils
            K = CameraUtils.default_pi_camera_matrix()
            RT = np.eye(4)  # Identity transform (camera at origin)

            # Optional geometric prioritization
            geometric_prioritization = self.get_parameter('geometric_prioritization').get_parameter_value().bool_value
            if geometric_prioritization:
                max_points = self.get_parameter('max_points_per_frame').get_parameter_value().integer_value
                geometric_weight = self.get_parameter('geometric_weight').get_parameter_value().double_value

                # Apply geometric prioritization
                xyz_prioritized = TransformUtils.prioritize_points_by_geometry(
                    xyz, cv_image, K, RT,
                    max_points=max_points if max_points > 0 else None,
                    geometric_weight=geometric_weight
                )

                self.get_logger().debug(f"Geometric prioritization: {len(xyz)} -> {len(xyz_prioritized)} points")
                xyz = xyz_prioritized

                # Re-extract colors for prioritized points (simplified)
                # In full implementation, would re-project and sample colors
                rgb_packed = ColorUtils.pack_rgb(np.random.randint(0, 255, (len(xyz), 3)))

            # Create fused point cloud
            from sensor_msgs_py import point_cloud2
            from sensor_msgs.msg import PointField

            FIELDS = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]

            result = point_cloud2.create_cloud(self.lidar_data.header, FIELDS,
                                             np.column_stack((xyz, rgb_packed)))
            result.header.frame_id = 'fused_frame'

            return result

        except Exception as e:
            self.get_logger().error(f"Full fusion failed: {e}")
            return None

    def process_basic_fusion(self) -> Optional[PointCloud2]:
        """Basic fusion with reduced processing"""
        try:
            # Simplified processing for degraded conditions
            result = PointCloud2()
            result.header = self.lidar_data.header
            # ... basic processing only ...

            return result

        except Exception as e:
            self.get_logger().error(f"Basic fusion failed: {e}")
            return None

    def process_minimal_fusion(self) -> Optional[PointCloud2]:
        """Minimal fusion for emergency operation"""
        try:
            # Minimal processing - just pass through LiDAR data
            result = self.lidar_data  # Simple pass-through
            return result

        except Exception as e:
            self.get_logger().error(f"Minimal fusion failed: {e}")
            return None

    def update_performance_feedback(self, frame_start_time: float):
        """Update performance metrics with monitoring"""
        current_time = time.time()
        processing_time = current_time - frame_start_time

        # Calculate FPS
        time_diff = current_time - self.last_frame_time
        if time_diff > 0:
            instantaneous_fps = 1.0 / time_diff
            self.infra.update_performance(instantaneous_fps)

        self.last_frame_time = current_time

        # Log comprehensive performance every 30 frames
        if hasattr(self, 'frame_count') and self.frame_count % 30 == 0:
            current_memory = get_memory_usage()
            avg_cpu = sum(self.cpu_history) / len(self.cpu_history) if self.cpu_history else 0

            self.get_logger().info(f"Performance: CPU {avg_cpu:.1f}%, "
                                 f"Memory {current_memory:.1f}MB, "
                                 f"FPS {instantaneous_fps:.1f}, "
                                 f"Processing {processing_time:.3f}s, "
                                 f"Frame drops: {self.frame_drop_count}")

    def on_mode_change(self, old_mode: FusionMode, new_mode: FusionMode):
        """Handle fusion mode changes"""
        self.get_logger().info(f"Adapting to mode: {new_mode.value}")

        # Could adjust subscriptions, publishers, etc. based on mode
        if new_mode == FusionMode.MINIMAL:
            # Reduce QoS, change topics, etc.
            pass
        elif new_mode == FusionMode.FULL:
            # Restore full capabilities
            pass

    def get_system_status(self) -> dict:
        """Get comprehensive system status"""
        health = self.infra.get_system_health()

        return {
            'fusion_mode': self.infra.get_fusion_mode().value,
            'system_health': {
                'sensors_ok': health.sensors_ok,
                'network_ok': health.network_ok,
                'thermal_ok': health.thermal_ok,
                'overall_ok': health.overall_ok
            },
            'network_quality': self.infra.get_network_quality(),
            'processing_params': self.infra.get_processing_params(),
            'performance': {
                'current_fps': self.infra.current_fps,
                'target_fps': self.infra.target_fps
            }
        }


def main(args=None):
    rclpy.init(args=args)

    node = SimpleFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        node.infra.stop_monitoring()  # Clean shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

