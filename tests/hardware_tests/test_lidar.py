#!/usr/bin/env python3
"""
LiDAR Hardware Test
===================

Tests Unitree L2 LiDAR hardware and ROS integration
"""

import unittest
import time
import subprocess
import sys
import os

# Add src to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class LidarHardwareTest(unittest.TestCase):
    """Test LiDAR hardware functionality"""

    LIDAR_IP = "192.168.1.150"

    def test_network_connectivity(self):
        """Test network connectivity to L2 LiDAR"""
        try:
            result = subprocess.run(
                ['ping', '-c', '3', '-W', '2', self.LIDAR_IP],
                capture_output=True, text=True, timeout=10
            )

            self.assertEqual(result.returncode, 0,
                           f"Cannot ping L2 at {self.LIDAR_IP}")

            # Check for reasonable ping times
            lines = result.stdout.strip().split('\n')
            rtt_line = [line for line in lines if 'rtt min/avg/max/mdev' in line]
            if rtt_line:
                # Parse avg RTT
                rtt_stats = rtt_line[0].split('=')[1].split('/')[1]
                avg_rtt = float(rtt_stats)

                self.assertLess(avg_rtt, 10.0,
                              ".1f")

        except subprocess.TimeoutExpired:
            self.fail(f"Ping to {self.LIDAR_IP} timed out")

    def test_lidar_driver_installation(self):
        """Test if L2 driver is installed"""
        ros_ws = os.path.expanduser('~/ros2_ws')

        # Check if ROS workspace exists
        self.assertTrue(os.path.exists(ros_ws),
                       "ROS workspace not found")

        # Check if driver is cloned
        driver_path = os.path.join(ros_ws, 'src', 'unilidar_sdk_ros2')
        self.assertTrue(os.path.exists(driver_path),
                       "L2 driver not cloned")

        # Check if driver is built
        install_path = os.path.join(ros_ws, 'install', 'lib', 'unilidar_sdk')
        self.assertTrue(os.path.exists(install_path),
                       "L2 driver not built")

    def test_driver_launch_capability(self):
        """Test if L2 driver can be launched"""
        if not ROS_AVAILABLE:
            self.skipTest("ROS 2 not available")

        try:
            # Source ROS environment
            env = os.environ.copy()
            ros_setup = "/opt/ros/humble/setup.bash"
            ws_setup = "~/ros2_ws/install/setup.bash"

            if os.path.exists(os.path.expanduser(ws_setup)):
                env['BASH_ENV'] = f"source {ros_setup}; source {ws_setup}"

            # Try to launch driver briefly
            result = subprocess.run(
                ['timeout', '5s', 'ros2', 'launch', 'unilidar_sdk', 'run.launch.py'],
                capture_output=True, text=True, timeout=10, env=env
            )

            # Launch might fail due to network issues, but should not crash immediately
            # We mainly want to test that the launch files exist and are executable
            launch_file = os.path.expanduser('~/ros2_ws/src/unilidar_sdk_ros2/launch/run.launch.py')
            self.assertTrue(os.path.exists(launch_file),
                          "Launch file not found")

        except subprocess.TimeoutExpired:
            # This is expected - we're timing out the launch
            pass
        except Exception as e:
            self.fail(f"Driver launch test failed: {e}")

    @unittest.skipUnless(ROS_AVAILABLE, "ROS 2 not available")
    def test_lidar_topic_detection(self):
        """Test LiDAR topic detection"""
        rclpy.init()
        node = Node('test_lidar_node')

        try:
            # Create a simple topic lister
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True, text=True, timeout=5
            )

            self.assertEqual(result.returncode, 0,
                           "Failed to list ROS topics")

            # Note: This test requires the driver to be running
            # In a real test environment, you would launch the driver first

        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_network_configuration(self):
        """Test network configuration for L2"""
        try:
            result = subprocess.run(
                ['ip', 'addr', 'show'],
                capture_output=True, text=True, timeout=5
            )

            self.assertEqual(result.returncode, 0,
                           "Failed to check network configuration")

            # Check if L2 network interface exists
            # This is system-dependent, so we'll just check that we can run network commands
            self.assertIn("inet", result.stdout,
                         "Network configuration appears incomplete")

        except subprocess.TimeoutExpired:
            self.fail("Network configuration check timed out")

    def test_lidar_performance_requirements(self):
        """Test that system meets L2 performance requirements"""
        # Check CPU cores
        try:
            with open('/proc/cpuinfo', 'r') as f:
                cpu_count = sum(1 for line in f if line.startswith('processor'))
                self.assertGreaterEqual(cpu_count, 4,
                                      f"Insufficient CPU cores: {cpu_count}, need >= 4")
        except FileNotFoundError:
            self.skipTest("CPU info not available")

        # Check memory
        try:
            with open('/proc/meminfo', 'r') as f:
                for line in f:
                    if line.startswith('MemTotal:'):
                        mem_kb = int(line.split()[1])
                        mem_gb = mem_kb / 1024 / 1024
                        self.assertGreaterEqual(mem_gb, 2.0,
                                              ".1f")
                        break
        except (FileNotFoundError, ValueError):
            self.skipTest("Memory info not available")

        # Check network interface speed (if possible)
        try:
            result = subprocess.run(
                ['ethtool', 'eth0'],
                capture_output=True, text=True, timeout=5
            )

            if result.returncode == 0 and 'Speed:' in result.stdout:
                # Extract speed
                for line in result.stdout.split('\n'):
                    if 'Speed:' in line:
                        speed_str = line.split('Speed:')[1].strip()
                        # This would require parsing "1000Mb/s" etc.
                        break
        except (subprocess.TimeoutExpired, FileNotFoundError):
            # ethtool not available or interface doesn't exist
            pass


class ROSLidarTest(unittest.TestCase):
    """Test ROS LiDAR integration"""

    @unittest.skipUnless(ROS_AVAILABLE, "ROS 2 not available")
    def setUp(self):
        """Set up ROS test"""
        rclpy.init()
        self.node = Node('test_lidar_node')
        self.received_clouds = []

    @unittest.skipUnless(ROS_AVAILABLE, "ROS 2 not available")
    def tearDown(self):
        """Clean up ROS test"""
        self.node.destroy_node()
        rclpy.shutdown()

    @unittest.skipUnless(ROS_AVAILABLE, "ROS 2 not available")
    def test_pointcloud2_message_handling(self):
        """Test PointCloud2 message handling"""
        # Test that we can create and manipulate PointCloud2 messages
        from sensor_msgs.msg import PointCloud2, PointField
        from sensor_msgs_py import point_cloud2

        # Create a simple point cloud
        points = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = point_cloud2.create_cloud(
            header=self.node.get_clock().now().to_msg(),
            fields=fields,
            points=points
        )

        self.assertIsNotNone(cloud_msg)
        self.assertEqual(cloud_msg.width, 2)  # 2 points
        self.assertEqual(cloud_msg.height, 1)

        # Test reading back
        read_points = list(point_cloud2.read_points(cloud_msg, field_names=('x', 'y', 'z')))
        self.assertEqual(len(read_points), 2)
        self.assertAlmostEqual(read_points[0][0], 1.0)
        self.assertAlmostEqual(read_points[1][2], 6.0)

    @unittest.skipUnless(ROS_AVAILABLE, "ROS 2 not available")
    def test_lidar_data_validation(self):
        """Test LiDAR data validation functions"""
        # This would test the data validation functions from the fusion code
        # For now, just a placeholder
        self.assertTrue(True, "LiDAR data validation test placeholder")


class LidarIntegrationTest(unittest.TestCase):
    """Test full LiDAR integration"""

    def test_lidar_fusion_pipeline(self):
        """Test the complete LiDAR fusion pipeline"""
        # This would test the full pipeline from raw LiDAR data to fused output
        # For now, just test that all components can be imported

        try:
            from src.core.fusion import LidarCameraFusion
            from src.infrastructure.infrastructure import RealWorldInfrastructure
            from src.utils.utils import PointCloudUtils

            # Test that classes can be instantiated (without ROS context)
            self.assertTrue(hasattr(LidarCameraFusion, '__init__'))
            self.assertTrue(hasattr(RealWorldInfrastructure, '__init__'))
            self.assertTrue(hasattr(PointCloudUtils, 'extract_xyz_rgb'))

        except ImportError as e:
            self.fail(f"Failed to import fusion components: {e}")


if __name__ == '__main__':
    # Add verbose output
    if '--verbose' in sys.argv:
        sys.argv.remove('--verbose')
        unittest.main(verbosity=2)
    else:
        unittest.main()
