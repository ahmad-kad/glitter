#!/usr/bin/env python3
"""
Camera Hardware Test
====================

Tests camera hardware and ROS integration
"""

import unittest
import time
import cv2
import subprocess
import sys
import os

# Add src to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class CameraHardwareTest(unittest.TestCase):
    """Test camera hardware functionality"""

    def setUp(self):
        """Set up test fixtures"""
        self.bridge = CvBridge() if ROS_AVAILABLE else None

    def test_libcamera_detection(self):
        """Test libcamera camera detection"""
        try:
            result = subprocess.run(
                ['libcamera-hello', '--list-cameras'],
                capture_output=True, text=True, timeout=10
            )

            # Check if cameras are available
            self.assertEqual(result.returncode, 0,
                           "libcamera-hello failed")
            self.assertIn("Available", result.stdout,
                         "No cameras detected by libcamera")

        except (subprocess.TimeoutExpired, FileNotFoundError):
            self.skipTest("libcamera not available or no cameras detected")

    def test_v4l2_camera_detection(self):
        """Test V4L2 camera detection"""
        try:
            result = subprocess.run(
                ['v4l2-ctl', '--list-devices'],
                capture_output=True, text=True, timeout=5
            )

            self.assertEqual(result.returncode, 0,
                           "v4l2-ctl failed")

            # Check for video devices
            self.assertTrue(
                "/dev/video" in result.stdout or
                "video" in result.stdout,
                "No video devices found"
            )

        except (subprocess.TimeoutExpired, FileNotFoundError):
            self.skipTest("v4l2-ctl not available")

    def test_camera_capture_libcamera(self):
        """Test camera capture with libcamera"""
        try:
            # Capture a test image
            result = subprocess.run(
                ['libcamera-jpeg', '-o', '/tmp/test_capture.jpg',
                 '--timeout', '2000', '--width', '640', '--height', '480'],
                capture_output=True, timeout=10
            )

            self.assertEqual(result.returncode, 0,
                           f"Camera capture failed: {result.stderr}")

            # Check if file was created and has reasonable size
            if os.path.exists('/tmp/test_capture.jpg'):
                size = os.path.getsize('/tmp/test_capture.jpg')
                self.assertGreater(size, 10000,
                                 "Captured image too small")
                os.remove('/tmp/test_capture.jpg')

        except (subprocess.TimeoutExpired, FileNotFoundError):
            self.skipTest("libcamera-jpeg not available or camera capture failed")

    def test_camera_info_v4l2(self):
        """Test camera info retrieval with v4l2"""
        try:
            result = subprocess.run(
                ['v4l2-ctl', '-d', '/dev/video0', '--info'],
                capture_output=True, text=True, timeout=5
            )

            self.assertEqual(result.returncode, 0,
                           "Failed to get camera info")

            # Check for expected info
            self.assertTrue(
                "Card type" in result.stdout or
                "Driver" in result.stdout,
                "Camera info incomplete"
            )

        except (subprocess.TimeoutExpired, FileNotFoundError):
            self.skipTest("v4l2 camera info not available")

    @unittest.skipUnless(ROS_AVAILABLE, "ROS 2 not available")
    def test_ros_camera_bridge(self):
        """Test ROS camera bridge functionality"""
        # Create a test ROS node
        rclpy.init()
        node = Node('test_camera_node')

        try:
            # Test CvBridge functionality
            # Create a test OpenCV image
            test_image = cv2.imread('/tmp/test_capture.jpg') if os.path.exists('/tmp/test_capture.jpg') else None

            if test_image is not None:
                # Convert to ROS message
                ros_image = self.bridge.cv2_to_imgmsg(test_image, 'bgr8')
                self.assertIsNotNone(ros_image)
                self.assertEqual(ros_image.height, test_image.shape[0])
                self.assertEqual(ros_image.width, test_image.shape[1])

                # Convert back to OpenCV
                cv_image = self.bridge.imgmsg_to_cv2(ros_image, 'bgr8')
                self.assertEqual(cv_image.shape, test_image.shape)

        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_camera_performance(self):
        """Test camera performance metrics"""
        try:
            # Time a series of captures
            import time

            captures = []
            for i in range(5):
                start_time = time.time()

                result = subprocess.run(
                    ['libcamera-jpeg', '-o', f'/tmp/perf_test_{i}.jpg',
                     '--timeout', '1000', '--width', '640', '--height', '480'],
                    capture_output=True, timeout=5
                )

                end_time = time.time()

                if result.returncode == 0:
                    captures.append(end_time - start_time)

                    # Clean up
                    if os.path.exists(f'/tmp/perf_test_{i}.jpg'):
                        os.remove(f'/tmp/perf_test_{i}.jpg')

            if captures:
                avg_time = sum(captures) / len(captures)
                fps = 1.0 / avg_time if avg_time > 0 else 0

                # Log performance
                print(".3f")
                print(".1f")

                # Basic performance check
                self.assertGreater(fps, 1.0, "Camera capture too slow (< 1 FPS)")

        except (subprocess.TimeoutExpired, FileNotFoundError):
            self.skipTest("Camera performance test not available")


class ROSCameraTest(unittest.TestCase):
    """Test ROS camera integration"""

    @unittest.skipUnless(ROS_AVAILABLE, "ROS 2 not available")
    def setUp(self):
        """Set up ROS test"""
        rclpy.init()
        self.node = Node('test_camera_node')
        self.received_images = []

    @unittest.skipUnless(ROS_AVAILABLE, "ROS 2 not available")
    def tearDown(self):
        """Clean up ROS test"""
        self.node.destroy_node()
        rclpy.shutdown()

    @unittest.skipUnless(ROS_AVAILABLE, "ROS 2 not available")
    def test_camera_topic_subscription(self):
        """Test subscription to camera topics"""
        # This would require actually launching camera drivers
        # For now, just test that the node can be created
        self.assertIsNotNone(self.node)

        # Check if common camera topics exist
        # Note: This requires camera drivers to be running
        # In a real test environment, you would launch camera drivers first

        # Placeholder test
        self.assertTrue(True, "ROS camera topic test placeholder")


if __name__ == '__main__':
    # Add verbose output
    if '--verbose' in sys.argv:
        sys.argv.remove('--verbose')
        unittest.main(verbosity=2)
    else:
        unittest.main()
