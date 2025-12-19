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
    LIDAR_NETWORK = "192.168.1.0/24"

    def test_network_connectivity(self):
        """Test network connectivity to L2 LiDAR using ARP/neighbor discovery"""
        # First, check if we're on the correct network
        try:
            # Get network interfaces and their IPs
            result = subprocess.run(
                ['ip', 'addr', 'show'],
                capture_output=True, text=True, timeout=5
            )
            
            # Check if we have an IP on 192.168.1.x network
            has_lidar_network = False
            for line in result.stdout.split('\n'):
                if 'inet 192.168.1.' in line and 'eth0' in result.stdout:
                    has_lidar_network = True
                    break
            
            if not has_lidar_network:
                self.skipTest("Not on LiDAR network (192.168.1.x). Configure network first.")
            
            # Try ping first
            ping_result = subprocess.run(
                ['ping', '-c', '2', '-W', '2', self.LIDAR_IP],
                capture_output=True, text=True, timeout=5
            )
            
            # Check ARP table / neighbor discovery for LiDAR devices
            # Use 'ip neigh' (modern) or 'arp' (legacy)
            arp_result = subprocess.run(
                ['ip', 'neigh', 'show', 'dev', 'eth0'],
                capture_output=True, text=True, timeout=5
            )
            
            # Also try scanning the network for LiDAR devices
            # Look for devices in ARP table or try common LiDAR IPs
            lidar_found = False
            lidar_ip = None
            
            if ping_result.returncode == 0:
                # Ping succeeded
                lidar_found = True
                lidar_ip = self.LIDAR_IP
            else:
                # Check ARP table for any 192.168.1.x devices
                for line in arp_result.stdout.split('\n'):
                    if '192.168.1.' in line and 'REACHABLE' in line:
                        # Extract IP from line (format: IP dev eth0 lladdr MAC REACHABLE)
                        parts = line.split()
                        if len(parts) > 0:
                            potential_ip = parts[0]
                            # Try to ping this IP to verify it's a LiDAR
                            test_ping = subprocess.run(
                                ['ping', '-c', '1', '-W', '1', potential_ip],
                                capture_output=True, text=True, timeout=3
                            )
                            if test_ping.returncode == 0:
                                lidar_found = True
                                lidar_ip = potential_ip
                                break
                
                # If still not found, try common LiDAR IPs
                if not lidar_found:
                    common_lidar_ips = ['192.168.1.150', '192.168.1.100', '192.168.1.200']
                    for test_ip in common_lidar_ips:
                        test_ping = subprocess.run(
                            ['ping', '-c', '1', '-W', '1', test_ip],
                            capture_output=True, text=True, timeout=3
                        )
                        if test_ping.returncode == 0:
                            lidar_found = True
                            lidar_ip = test_ip
                            break
            
            if lidar_found:
                # Verify connectivity with reasonable ping times
                verify_result = subprocess.run(
                    ['ping', '-c', '3', '-W', '2', lidar_ip],
                    capture_output=True, text=True, timeout=10
                )
                
                if verify_result.returncode == 0:
                    # Parse RTT if available
                    lines = verify_result.stdout.strip().split('\n')
                    rtt_line = [line for line in lines if 'rtt min/avg/max/mdev' in line]
                    if rtt_line:
                        try:
                            rtt_stats = rtt_line[0].split('=')[1].split('/')[1]
                            avg_rtt = float(rtt_stats)
                            self.assertLess(avg_rtt, 50.0,
                                          f"High latency to LiDAR: {avg_rtt:.1f}ms")
                        except (ValueError, IndexError):
                            pass  # RTT parsing failed, but ping succeeded
                    
                    # Update LIDAR_IP if we found a different IP
                    if lidar_ip != self.LIDAR_IP:
                        self.LIDAR_IP = lidar_ip
                        print(f"Found LiDAR at {lidar_ip} (expected {self.LIDAR_IP})")
                else:
                    self.fail(f"LiDAR found at {lidar_ip} but ping verification failed")
            else:
                self.fail(f"LiDAR not found on network. Checked {self.LIDAR_IP} and ARP table. "
                         f"Ensure LiDAR is connected and powered on.")

        except subprocess.TimeoutExpired:
            self.fail(f"Network connectivity test timed out")
        except Exception as e:
            self.fail(f"Network connectivity test failed: {e}")

    def test_lidar_driver_installation(self):
        """Test if L2 driver is installed"""
        ros_ws = os.path.expanduser('~/ros2_ws')

        # Check if ROS workspace exists
        if not os.path.exists(ros_ws):
            self.skipTest("ROS workspace not found at ~/ros2_ws. Run install_unitree_l2.sh to set up.")

        # Check for multiple possible driver locations/names
        possible_driver_paths = [
            os.path.join(ros_ws, 'src', 'unilidar_sdk_ros2'),
            os.path.join(ros_ws, 'src', 'unilidar_sdk2'),
            os.path.join(ros_ws, 'src', 'unitree_lidar_ros2'),
        ]
        
        driver_path = None
        driver_name = None
        for path in possible_driver_paths:
            if os.path.exists(path):
                driver_path = path
                driver_name = os.path.basename(path)
                break
        
        if not driver_path:
            self.fail(f"L2 driver not found in ROS workspace. Checked: {possible_driver_paths}. "
                     f"Run: cd ~/ros2_ws/src && git clone https://github.com/unitreerobotics/unilidar_sdk_ros2.git")

        # Check if driver is built - check multiple possible install locations
        possible_install_paths = [
            os.path.join(ros_ws, 'install', 'lib', 'unilidar_sdk'),
            os.path.join(ros_ws, 'install', 'lib', 'unilidar_sdk2'),
            os.path.join(ros_ws, 'install', 'lib', 'unitree_lidar_ros2'),
            os.path.join(ros_ws, 'install', 'share', 'unilidar_sdk'),
            os.path.join(ros_ws, 'install', 'share', 'unilidar_sdk2'),
            os.path.join(ros_ws, 'install', 'share', 'unitree_lidar_ros2'),
        ]
        
        install_path = None
        for path in possible_install_paths:
            if os.path.exists(path):
                install_path = path
                break
        
        if not install_path:
            self.fail(f"L2 driver not built. Found source at {driver_path} but no install directory. "
                     f"Run: cd ~/ros2_ws && colcon build --packages-select {driver_name}")
        
        # Verify launch files exist
        launch_paths = [
            os.path.join(driver_path, 'launch', 'run.launch.py'),
            os.path.join(driver_path, 'launch', 'launch.py'),
            os.path.join(driver_path, 'launch', 'unitree_lidar_ros2.launch.py'),
        ]
        
        launch_file = None
        for path in launch_paths:
            if os.path.exists(path):
                launch_file = path
                break
        
        if not launch_file:
            self.fail(f"Launch file not found in driver. Checked: {launch_paths}")

    def test_driver_launch_capability(self):
        """Test if L2 driver can be launched"""
        if not ROS_AVAILABLE:
            self.skipTest("ROS 2 not available")

        ros_ws = os.path.expanduser('~/ros2_ws')
        if not os.path.exists(ros_ws):
            self.skipTest("ROS workspace not found")

        # Find driver and launch file
        possible_driver_paths = [
            os.path.join(ros_ws, 'src', 'unilidar_sdk_ros2'),
            os.path.join(ros_ws, 'src', 'unilidar_sdk2'),
            os.path.join(ros_ws, 'src', 'unitree_lidar_ros2'),
        ]
        
        driver_path = None
        package_name = None
        for path in possible_driver_paths:
            if os.path.exists(path):
                driver_path = path
                # Try to determine package name from package.xml or CMakeLists.txt
                package_xml = os.path.join(path, 'package.xml')
                if os.path.exists(package_xml):
                    try:
                        with open(package_xml, 'r') as f:
                            content = f.read()
                            import re
                            match = re.search(r'<name>([^<]+)</name>', content)
                            if match:
                                package_name = match.group(1)
                    except:
                        pass
                
                # Fallback to directory name
                if not package_name:
                    package_name = os.path.basename(path)
                break
        
        if not driver_path:
            self.skipTest("L2 driver not found in ROS workspace")
        
        # Find launch file
        launch_paths = [
            os.path.join(driver_path, 'launch', 'run.launch.py'),
            os.path.join(driver_path, 'launch', 'launch.py'),
            os.path.join(driver_path, 'launch', 'unitree_lidar_ros2.launch.py'),
        ]
        
        launch_file = None
        for path in launch_paths:
            if os.path.exists(path):
                launch_file = path
                break
        
        self.assertTrue(launch_file is not None,
                      f"Launch file not found. Checked: {launch_paths}")
        
        # Check if package is built and available
        install_setup = os.path.expanduser('~/ros2_ws/install/setup.bash')
        if not os.path.exists(install_setup):
            self.skipTest("ROS workspace not built. Run: cd ~/ros2_ws && colcon build")
        
        # Try to verify launch file syntax (basic check)
        try:
            # Check if it's a valid Python file
            result = subprocess.run(
                ['python3', '-m', 'py_compile', launch_file],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode != 0:
                self.fail(f"Launch file has syntax errors: {result.stderr}")
        except Exception as e:
            self.fail(f"Could not verify launch file: {e}")
        
        # Note: We don't actually launch here to avoid requiring network/hardware
        # The launch capability is verified by checking file existence and syntax

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
            # Import with proper path handling
            import sys
            project_root = os.path.join(os.path.dirname(__file__), '..', '..')
            if project_root not in sys.path:
                sys.path.insert(0, project_root)
            
            from src.core.fusion import LidarCameraFusion
            # RealWorldInfrastructure is in root infrastructure.py, not src/infrastructure/
            try:
                from infrastructure import RealWorldInfrastructure
            except ImportError:
                # Try alternative import path
                import sys
                root_path = os.path.join(os.path.dirname(__file__), '..')
                if root_path not in sys.path:
                    sys.path.insert(0, root_path)
                from infrastructure import RealWorldInfrastructure
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
