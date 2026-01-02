#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from sensor_msgs_py import point_cloud2
import message_filters
from cv_bridge import CvBridge
import numpy as np
import struct
import json
import os
from datetime import datetime

class LidarCameraFusion(Node):
    """
    ROS 2 Node for real-time LiDAR-Camera sensor fusion.
    Located at: /home/durian/glitter/glitter/src/core/fusion.py
    """
    def __init__(self):
        super().__init__('lidar_camera_fusion')
        
        # 1. Parameters & State
        self.bridge = CvBridge()
        self.camera_model = None # Will be populated by CameraInfo or estimated from image
        
        # Extrinsic Calibration: Transform from LiDAR to Camera Link
        # Default: LiDAR is 10cm above camera
        self.declare_parameter('extrinsic_xyz', [0.1, 0.0, 0.0]) # Translation
        self.declare_parameter('extrinsic_rpy', [0.0, 0.0, 0.0]) # Rotation (radians)
        
        # Track if we've warned about missing CameraInfo
        self._camera_info_warned = False

        # 2. Subscribers with Approximation Sync
        # We use a small slop (0.05s) to ensure colors match the motion
        self.info_sub = self.create_subscription(
            CameraInfo, 
            '/camera/camera/color/camera_info', 
            self.info_callback, 
            10
        )
        
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, '/unilidar/cloud')
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        
        # Debug: Track individual message reception
        self._lidar_count = 0
        self._image_count = 0
        
        # Create individual subscribers for debugging
        self.debug_lidar_sub = self.create_subscription(
            PointCloud2, '/unilidar/cloud', 
            lambda msg: self._debug_lidar_callback(msg), 10
        )
        self.debug_image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            lambda msg: self._debug_image_callback(msg), 10
        )
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_sub, self.image_sub], 
            queue_size=10, 
            slop=0.5  # Increased tolerance to 0.5s for better synchronization
        )
        self.ts.registerCallback(self.sync_callback)
        # #region agent log
        try:
            log_data = {
                "sessionId": "debug-session",
                "runId": "run1",
                "hypothesisId": "E",
                "location": "fusion.py:__init__:synchronizer_setup",
                "message": "ApproximateTimeSynchronizer configured",
                "data": {
                    "slop_seconds": 0.5,
                    "queue_size": 10,
                    "lidar_topic": "/unilidar/cloud",
                    "image_topic": "/camera/camera/color/image_raw"
                },
                "timestamp": int(datetime.now().timestamp() * 1000)
            }
            with open("/home/durian/glitter/glitter/.cursor/debug.log", "a") as f:
                f.write(json.dumps(log_data) + "\n")
        except: pass
        # #endregion

        # 3. Publisher
        self.pc_pub = self.create_publisher(PointCloud2, '/unilidar/colored_cloud', 10)
        
        self.get_logger().info("Fusion Node Initialized. Will use CameraInfo if available, otherwise estimate from image.")

    def info_callback(self, msg):
        """Store camera intrinsics from the camera_info topic."""
        if self.camera_model is None:
            self.camera_model = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("Camera Intrinsics (K Matrix) Received and loaded.")
    
    def _debug_lidar_callback(self, msg):
        """Debug callback to verify LiDAR messages are arriving."""
        # #region agent log
        try:
            log_data = {
                "sessionId": "debug-session",
                "runId": "run1",
                "hypothesisId": "B",
                "location": "fusion.py:_debug_lidar_callback",
                "message": "LiDAR message received",
                "data": {
                    "count": self._lidar_count + 1,
                    "timestamp_nsec": msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec,
                    "point_count": msg.width * msg.height if hasattr(msg, 'width') and hasattr(msg, 'height') else 0
                },
                "timestamp": int(datetime.now().timestamp() * 1000)
            }
            with open("/home/durian/glitter/glitter/.cursor/debug.log", "a") as f:
                f.write(json.dumps(log_data) + "\n")
        except: pass
        # #endregion
        self._lidar_count += 1
        if self._lidar_count % 30 == 0:
            self.get_logger().info(f"LiDAR messages received: {self._lidar_count}")
    
    def _debug_image_callback(self, msg):
        """Debug callback to verify image messages are arriving."""
        # #region agent log
        try:
            log_data = {
                "sessionId": "debug-session",
                "runId": "run1",
                "hypothesisId": "B",
                "location": "fusion.py:_debug_image_callback",
                "message": "Image message received",
                "data": {
                    "count": self._image_count + 1,
                    "timestamp_nsec": msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec,
                    "width": msg.width,
                    "height": msg.height
                },
                "timestamp": int(datetime.now().timestamp() * 1000)
            }
            with open("/home/durian/glitter/glitter/.cursor/debug.log", "a") as f:
                f.write(json.dumps(log_data) + "\n")
        except: pass
        # #endregion
        self._image_count += 1
        if self._image_count % 30 == 0:
            self.get_logger().info(f"Image messages received: {self._image_count}")

    def _estimate_camera_model(self, img_width, img_height):
        """Estimate camera intrinsics from image dimensions if CameraInfo unavailable."""
        # RealSense D435i typical intrinsics at common resolutions
        # Using pinhole camera model: fx, fy ≈ focal_length * pixel_size
        # For RealSense: fx ≈ fy ≈ 0.6 * min(width, height) is a reasonable estimate
        fx = fy = 0.6 * min(img_width, img_height)
        cx = img_width / 2.0
        cy = img_height / 2.0
        
        K = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        
        return K

    def sync_callback(self, lidar_msg, img_msg):
        """Callback triggered when LiDAR and Image timestamps are synchronized."""
        # #region agent log
        try:
            lidar_ts = lidar_msg.header.stamp.sec * 1e9 + lidar_msg.header.stamp.nanosec
            img_ts = img_msg.header.stamp.sec * 1e9 + img_msg.header.stamp.nanosec
            ts_diff_ms = abs(lidar_ts - img_ts) / 1e6
            log_data = {
                "sessionId": "debug-session",
                "runId": "run1",
                "hypothesisId": "A",
                "location": "fusion.py:sync_callback:entry",
                "message": "Sync callback triggered",
                "data": {
                    "lidar_timestamp_nsec": lidar_ts,
                    "image_timestamp_nsec": img_ts,
                    "timestamp_diff_ms": ts_diff_ms,
                    "lidar_points": lidar_msg.width * lidar_msg.height if hasattr(lidar_msg, 'width') else 0,
                    "image_size": f"{img_msg.width}x{img_msg.height}"
                },
                "timestamp": int(datetime.now().timestamp() * 1000)
            }
            with open("/home/durian/glitter/glitter/.cursor/debug.log", "a") as f:
                f.write(json.dumps(log_data) + "\n")
        except Exception as e:
            try:
                error_log = {
                    "sessionId": "debug-session",
                    "runId": "run1",
                    "hypothesisId": "C",
                    "location": "fusion.py:sync_callback:entry:error",
                    "message": "Error in sync_callback entry logging",
                    "data": {"error": str(e)},
                    "timestamp": int(datetime.now().timestamp() * 1000)
                }
                with open("/home/durian/glitter/glitter/.cursor/debug.log", "a") as f:
                    f.write(json.dumps(error_log) + "\n")
            except: pass
        # #endregion
        
        # Start timing for performance monitoring
        start_time = self.get_clock().now()

        # --- STEP 1: Process Image ---
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            h, w = cv_img.shape[:2]
        except Exception as e:
            # #region agent log
            try:
                error_log = {
                    "sessionId": "debug-session",
                    "runId": "run1",
                    "hypothesisId": "C",
                    "location": "fusion.py:sync_callback:image_conversion_error",
                    "message": "Image conversion failed",
                    "data": {"error": str(e), "error_type": type(e).__name__},
                    "timestamp": int(datetime.now().timestamp() * 1000)
                }
                with open("/home/durian/glitter/glitter/.cursor/debug.log", "a") as f:
                    f.write(json.dumps(error_log) + "\n")
            except: pass
            # #endregion
            self.get_logger().error(f"Image conversion error: {e}")
            return
        
        # If CameraInfo not received, estimate from image dimensions
        if self.camera_model is None:
            self.camera_model = self._estimate_camera_model(w, h)
            if not self._camera_info_warned:
                self.get_logger().warn(
                    f"CameraInfo not available. Using estimated intrinsics: "
                    f"fx={self.camera_model[0,0]:.1f}, fy={self.camera_model[1,1]:.1f}, "
                    f"cx={self.camera_model[0,2]:.1f}, cy={self.camera_model[1,2]:.1f}"
                )
                self._camera_info_warned = True

        # --- STEP 2: Extract LiDAR Points (Vectorized) ---
        # Efficiently read x, y, z fields as a numpy array
        try:
            # Convert to regular numpy array (not structured array) for matrix operations
            points_list = list(point_cloud2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True))
            points = np.array([list(p) for p in points_list], dtype=np.float32)
            # #region agent log
            try:
                log_data = {
                    "sessionId": "debug-session",
                    "runId": "run1",
                    "hypothesisId": "D",
                    "location": "fusion.py:sync_callback:point_extraction",
                    "message": "Point extraction completed",
                    "data": {
                        "points_count": len(points),
                        "points_shape": list(points.shape) if points.size > 0 else []
                    },
                    "timestamp": int(datetime.now().timestamp() * 1000)
                }
                with open("/home/durian/glitter/glitter/.cursor/debug.log", "a") as f:
                    f.write(json.dumps(log_data) + "\n")
            except: pass
            # #endregion
            if points.size == 0:
                # #region agent log
                try:
                    log_data = {
                        "sessionId": "debug-session",
                        "runId": "run1",
                        "hypothesisId": "D",
                        "location": "fusion.py:sync_callback:empty_points",
                        "message": "Empty point cloud detected",
                        "data": {},
                        "timestamp": int(datetime.now().timestamp() * 1000)
                    }
                    with open("/home/durian/glitter/glitter/.cursor/debug.log", "a") as f:
                        f.write(json.dumps(log_data) + "\n")
                except: pass
                # #endregion
                return
        except Exception as e:
            # #region agent log
            try:
                error_log = {
                    "sessionId": "debug-session",
                    "runId": "run1",
                    "hypothesisId": "C",
                    "location": "fusion.py:sync_callback:point_extraction_error",
                    "message": "Point extraction failed",
                    "data": {"error": str(e), "error_type": type(e).__name__},
                    "timestamp": int(datetime.now().timestamp() * 1000)
                }
                with open("/home/durian/glitter/glitter/.cursor/debug.log", "a") as f:
                    f.write(json.dumps(error_log) + "\n")
            except: pass
            # #endregion
            self.get_logger().error(f"Point extraction error: {e}")
            return

        # --- STEP 3: Transform LiDAR to Camera Optical Frame ---
        # Unitree L2: Z is UP, X is FORWARD.
        # RealSense Optical: Z is FORWARD, Y is DOWN, X is RIGHT.
        R_align = np.array([
            [0, -1,  0], # New X is -Old Y
            [0,  0, -1], # New Y is -Old Z
            [1,  0,  0]  # New Z is  Old X (Forward)
        ])
        
        # Apply transformation to the point cloud
        pts_cam = points @ R_align.T 
        
        # --- STEP 4: Projection ---
        # Filter points behind camera (Z < 0.1m)
        mask = pts_cam[:, 2] > 0.1
        pts_cam = pts_cam[mask]
        original_pts = points[mask]

        if pts_cam.size == 0:
            return

        # Project 3D to 2D image coordinates: [u, v, 1]^T = K * [x, y, z]^T / z
        u_v = (self.camera_model @ pts_cam.T).T
        u_v[:, 0] /= u_v[:, 2] # x / z
        u_v[:, 1] /= u_v[:, 2] # y / z
        
        u = u_v[:, 0].astype(int)
        v = u_v[:, 1].astype(int)

        # --- STEP 5: Boundary Check & Color Sampling ---
        valid_idx = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        u, v = u[valid_idx], v[valid_idx]
        final_pts = original_pts[valid_idx]
        
        # Vectorized color sampling from the image
        colors = cv_img[v, u] 

        # --- STEP 6: Pack into PointCloud2 ---
        # Pack RGB into a single 32-bit float for ROS PointCloud2 compatibility (RGB8)
        processed_points = []
        for i in range(len(final_pts)):
            b, g, r = colors[i]
            # Pack 4 bytes (B, G, R, A) into one unsigned integer
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
            processed_points.append([final_pts[i][0], final_pts[i][1], final_pts[i][2], rgb])

        # Define fields for the output PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]

        # Create and publish the message
        out_msg = point_cloud2.create_cloud(lidar_msg.header, fields, processed_points)
        self.pc_pub.publish(out_msg)

        # Performance monitoring (log every few seconds)
        duration = (self.get_clock().now() - start_time).nanoseconds / 1e6
        # #region agent log
        try:
            log_data = {
                "sessionId": "debug-session",
                "runId": "run1",
                "hypothesisId": "A",
                "location": "fusion.py:sync_callback:exit",
                "message": "Sync callback completed successfully",
                "data": {
                    "processed_points": len(processed_points),
                    "duration_ms": duration,
                    "original_points": len(points),
                    "valid_projected": len(final_pts)
                },
                "timestamp": int(datetime.now().timestamp() * 1000)
            }
            with open("/home/durian/glitter/glitter/.cursor/debug.log", "a") as f:
                f.write(json.dumps(log_data) + "\n")
        except: pass
        # #endregion
        if self.frame_count_mod(30):
            self.get_logger().info(f"Fusion Success: {len(processed_points)} points colored in {duration:.2f}ms")

    def frame_count_mod(self, n):
        # Simple helper for periodic logging
        return self.get_clock().now().nanoseconds % n == 0

def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Fusion node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()