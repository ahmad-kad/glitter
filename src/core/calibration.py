#!/usr/bin/env python3
"""Manual calibration GUI for LiDAR-camera extrinsic parameters."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import message_filters
import numpy as np
import cv2
from cv_bridge import CvBridge
from typing import Tuple, List
from utils import TransformUtils, CameraUtils, PointCloudUtils, LoggingUtils, HAS_ROS_NUMPY

# Import constants from utils
from utils import (CALIBRATION_TRACKBAR_RANGE, CALIBRATION_TRACKBAR_OFFSET,
                   CALIBRATION_ANGLE_OFFSET, CALIBRATION_DEPTH_THRESHOLD)

class ManualCalibrator(Node):
    def __init__(self):
        super().__init__('manual_calibrator')

        # Setup logging with error handling
        try:
            self.logger = LoggingUtils.setup_logging("calibration.log")
            self.logger.info("Manual Calibrator initialized")
            self.logger.info(f"ROS NumPy available: {HAS_ROS_NUMPY}")
        except Exception as e:
            self.get_logger().error(f"Failed to setup logging: {e}")
            raise

        # Setup camera intrinsic matrix
        try:
            self.K = CameraUtils.default_pi_camera_matrix()
        except Exception as e:
            self.logger.error(f"Failed to setup camera intrinsics: {e}")
            raise

        # Setup subscribers and synchronizer
        try:
            self.lidar_sub = message_filters.Subscriber(self, PointCloud2, '/unilidar/cloud')
            self.cam_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
            self.ts = message_filters.ApproximateTimeSynchronizer([self.lidar_sub, self.cam_sub], 10, 0.1)
            self.ts.registerCallback(self.callback)
            self.bridge = CvBridge()
        except Exception as e:
            self.logger.error(f"Failed to setup ROS components: {e}")
            raise
        self.frame_count = 0

        cv2.namedWindow("Calibration")
        for name in ["X (cm)", "Y (cm)", "Z (cm)"]:
            cv2.createTrackbar(name, "Calibration", 0, CALIBRATION_TRACKBAR_RANGE, self.nothing)
            cv2.setTrackbarPos(name, "Calibration", CALIBRATION_TRACKBAR_OFFSET)
        for name in ["Roll (deg)", "Pitch (deg)", "Yaw (deg)"]:
            cv2.createTrackbar(name, "Calibration", CALIBRATION_ANGLE_OFFSET,
                              CALIBRATION_TRACKBAR_RANGE + CALIBRATION_ANGLE_OFFSET, self.nothing)

    def nothing(self, x):
        pass

    def get_sliders(self) -> Tuple[List[float], List[float]]:
        """Get current slider positions and convert to physical units.

        Returns:
            Tuple of ([x, y, z], [roll, pitch, yaw]) in meters and radians

        Raises:
            RuntimeError: If OpenCV trackbar operations fail
        """
        try:
            # Get translation values (convert cm to meters)
            x = (cv2.getTrackbarPos("X (cm)", "Calibration") - CALIBRATION_TRACKBAR_OFFSET) / CALIBRATION_TRACKBAR_RANGE
            y = (cv2.getTrackbarPos("Y (cm)", "Calibration") - CALIBRATION_TRACKBAR_OFFSET) / CALIBRATION_TRACKBAR_RANGE
            z = (cv2.getTrackbarPos("Z (cm)", "Calibration") - CALIBRATION_TRACKBAR_OFFSET) / CALIBRATION_TRACKBAR_RANGE

            # Get rotation values (convert degrees to radians)
            r = np.radians(cv2.getTrackbarPos("Roll (deg)", "Calibration") - CALIBRATION_ANGLE_OFFSET)
            p = np.radians(cv2.getTrackbarPos("Pitch (deg)", "Calibration") - CALIBRATION_ANGLE_OFFSET)
            yaw = np.radians(cv2.getTrackbarPos("Yaw (deg)", "Calibration") - CALIBRATION_ANGLE_OFFSET)

            # Validate ranges
            trans_values = [x, y, z]
            rot_values = [r, p, yaw]

            if not all(np.isfinite(v) for v in trans_values + rot_values):
                raise ValueError("Slider values contain non-finite values")

            return trans_values, rot_values

        except Exception as e:
            self.logger.error(f"Failed to get slider values: {e}")
            raise RuntimeError(f"Slider value retrieval failed: {e}")

    def callback(self, lidar_msg, img_msg):
        self.frame_count += 1

        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except Exception as e:
            self.logger.warning(f"Failed to convert image: {e}")
            return

        t, r = self.get_sliders()
        RT = TransformUtils.build_extrinsic_matrix(t, r)
        points_np, _ = PointCloudUtils.extract_xyz_rgb(lidar_msg, ('x', 'y', 'z'))
        points_np = PointCloudUtils.downsample_points(points_np, step=5)

        if len(points_np) > 0:
            points_hom = PointCloudUtils.to_homogeneous(points_np)
            points_cam = (RT @ points_hom.T).T

            # Filter points in front of camera
            depth_mask = points_cam[:, 2] > 0.1
            points_cam = points_cam[depth_mask]
            points_visible = np.sum(depth_mask)

            if len(points_cam) > 0:
                u, v = TransformUtils.project_3d_to_2d(points_cam[:, :3], self.K, np.eye(4))
                h, w, _ = cv_img.shape
                u_filtered, v_filtered = PointCloudUtils.filter_points_in_image_bounds(u, v, w, h)

                # Draw projected points
                for i in range(len(u_filtered)):
                    depth = points_cam[i, 2]
                    color = (0, 255, 0) if depth < CALIBRATION_DEPTH_THRESHOLD else (0, 0, 255)
                    cv2.circle(cv_img, (u_filtered[i], v_filtered[i]), 1, color, -1)

                # Log frame statistics every 30 frames
                LoggingUtils.log_frame_stats(self.logger, self.frame_count, {
                    "points": len(points_np),
                    "visible": points_visible,
                    "in_bounds": len(u_filtered)
                })

        cv2.imshow("Calibration", cv_img)
        key = cv2.waitKey(1)
        if key == ord('s'):
            self.logger.info(f"Calibration parameters saved - Translation: {t}, Rotation: {r}")
            print(f"Translation: {t}\nRotation: {r}")

def main(args=None):
    rclpy.init(args=args)
    node = ManualCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()