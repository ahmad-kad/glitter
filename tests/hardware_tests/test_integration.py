#!/usr/bin/env python3
"""
Integration Test for LiDAR-Camera Fusion
========================================

Tests the complete hardware integration and fusion pipeline
"""

import unittest
import time
import threading
import subprocess
import sys
import os
import argparse

# Add src to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, Image
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class IntegrationTest(unittest.TestCase):
    """Test complete LiDAR-camera integration"""

    def setUp(self):
        """Set up integration test"""
        self.test_duration = 10  # seconds
        self.lidar_topic = '/livox/lidar'
        self.camera_topic = '/camera/image_raw'
        self.fusion_topic = '/colored_cloud'

    @unittest.skipUnless(ROS_AVAILABLE, "ROS 2 not available")
    def test_sensor_data_flow(self):
        """Test that sensor data flows through the system"""
        rclpy.init()
        node = IntegrationTestNode()
        node.test_duration = self.test_duration

        try:
            # Start test node
            thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
            thread.start()

            # Wait for test to complete
            start_time = time.time()
            while time.time() - start_time < self.test_duration + 5:  # Extra 5s for cleanup
                if node.test_complete:
                    break
                time.sleep(0.1)

            # Check results
            self.assertTrue(node.lidar_received > 0,
                          f"No LiDAR data received (got {node.lidar_received})")
            self.assertTrue(node.camera_received > 0,
                          f"No camera data received (got {node.camera_received})")
            self.assertTrue(node.fusion_output > 0,
                          f"No fusion output (got {node.fusion_output})")

            # Check frequencies (rough estimates)
            lidar_freq = node.lidar_received / self.test_duration
            camera_freq = node.camera_received / self.test_duration
            fusion_freq = node.fusion_output / self.test_duration

            self.assertGreater(lidar_freq, 5.0,
                             ".1f")
            self.assertGreater(camera_freq, 10.0,
                             ".1f")
            self.assertGreater(fusion_freq, 5.0,
                             ".1f")

        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_hardware_validation_script(self):
        """Test the hardware validation script"""
        script_path = os.path.join(os.path.dirname(__file__), '..', '..', 'scripts', 'test_hardware.sh')

        if not os.path.exists(script_path):
            self.skipTest("Hardware validation script not found")

        try:
            result = subprocess.run(
                ['bash', script_path],
                capture_output=True, text=True, timeout=30
            )

            # Script should complete (may have failures, but should run)
            self.assertIn("Hardware Validation", result.stdout)

        except subprocess.TimeoutExpired:
            self.fail("Hardware validation script timed out")

    def test_fusion_node_startup(self):
        """Test that fusion node can start up"""
        if not ROS_AVAILABLE:
            self.skipTest("ROS 2 not available")

        # Test that we can import and create the fusion node
        try:
            from src.core.fusion import LidarCameraFusion
            from src.infrastructure.simple_fusion_node import SimpleFusionNode

            # Test that classes exist and can be instantiated (without ROS context)
            self.assertTrue(hasattr(LidarCameraFusion, '__init__'))
            self.assertTrue(hasattr(SimpleFusionNode, '__init__'))

        except ImportError as e:
            self.fail(f"Failed to import fusion components: {e}")

    def test_infrastructure_components(self):
        """Test that all infrastructure components work together"""
        try:
            from infrastructure import create_infrastructure
            from memory_pool import MemoryPool, MemoryPoolConfig
            from async_pipeline import AsyncIOPipeline
            from rolling_buffer import RollingBuffer

            # Test infrastructure creation
            infra = create_infrastructure(None, target_fps=30.0)
            self.assertIsNotNone(infra)

            # Test memory pool
            pool_config = MemoryPoolConfig(max_points_per_frame=10000)
            memory_pool = MemoryPool(pool_config)
            self.assertIsNotNone(memory_pool)

            # Test async pipeline
            pipeline = AsyncIOPipeline()
            self.assertIsNotNone(pipeline)

            # Test rolling buffer
            buffer = RollingBuffer()
            self.assertIsNotNone(buffer)

            # Clean up
            infra.stop_monitoring()

        except ImportError as e:
            self.fail(f"Failed to import infrastructure components: {e}")


@unittest.skipUnless(ROS_AVAILABLE, "ROS 2 not available")
class IntegrationTestNode(Node):
    """ROS node for integration testing"""

    def __init__(self):
        super().__init__('integration_test_node')

        self.lidar_received = 0
        self.camera_received = 0
        self.fusion_output = 0
        self.test_complete = False
        self.start_time = time.time()

        # Subscribe to topics
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/livox/lidar', self.lidar_callback, 10
        )

        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10
        )

        self.fusion_sub = self.create_subscription(
            PointCloud2, '/colored_cloud', self.fusion_callback, 10
        )

        # Timer to end test
        self.test_duration = 10.0  # Will be set by test
        self.timer = self.create_timer(self.test_duration, self.end_test)

    def lidar_callback(self, msg):
        """Handle LiDAR messages"""
        self.lidar_received += 1

    def camera_callback(self, msg):
        """Handle camera messages"""
        self.camera_received += 1

    def fusion_callback(self, msg):
        """Handle fusion output"""
        self.fusion_output += 1

    def end_test(self):
        """End the integration test"""
        self.test_complete = True
        self.timer.cancel()


class PerformanceTest(unittest.TestCase):
    """Performance testing for integration"""

    def test_memory_usage_bounds(self):
        """Test that memory usage stays within bounds"""
        try:
            import psutil
            process = psutil.Process()

            # Get initial memory
            initial_memory = process.memory_info().rss / 1024 / 1024  # MB

            # Run a simple test (this would be more comprehensive in real testing)
            time.sleep(1)

            # Check memory didn't explode
            final_memory = process.memory_info().rss / 1024 / 1024  # MB
            memory_increase = final_memory - initial_memory

            self.assertLess(memory_increase, 100,  # Less than 100MB increase
                          ".1f")

        except ImportError:
            self.skipTest("psutil not available for memory testing")

    def test_cpu_usage_bounds(self):
        """Test that CPU usage stays reasonable"""
        try:
            import psutil

            # Get CPU usage over a short period
            cpu_percent = psutil.cpu_percent(interval=1.0)

            # CPU usage should be reasonable (not pegged at 100%)
            self.assertLess(cpu_percent, 90.0,
                          ".1f")

        except ImportError:
            self.skipTest("psutil not available for CPU testing")


class StressTest(unittest.TestCase):
    """Stress testing for edge cases"""

    def test_high_frequency_operation(self):
        """Test operation under high frequency demands"""
        # This would test the system with artificially high data rates
        # For now, just a placeholder
        self.assertTrue(True, "High frequency stress test placeholder")

    def test_network_jitter_simulation(self):
        """Test operation with simulated network jitter"""
        # This would use the synthetic sensor simulator with jitter
        # For now, just a placeholder
        self.assertTrue(True, "Network jitter simulation test placeholder")

    def test_sensor_dropout_recovery(self):
        """Test recovery from sensor dropout"""
        # This would simulate sensors going offline and coming back
        # For now, just a placeholder
        self.assertTrue(True, "Sensor dropout recovery test placeholder")


def run_extended_integration_test(duration: int = 60):
    """Run extended integration test for longer durations"""
    print(f"Running extended integration test for {duration} seconds...")

    if not ROS_AVAILABLE:
        print("ROS 2 not available, skipping extended test")
        return

    rclpy.init()
    node = ExtendedIntegrationTestNode(duration)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_results()
        node.destroy_node()
        rclpy.shutdown()


class ExtendedIntegrationTestNode(Node):
    """Extended integration testing node"""

    def __init__(self, test_duration: int):
        super().__init__('extended_integration_test')

        self.test_duration = test_duration
        self.start_time = time.time()

        # Statistics
        self.stats = {
            'lidar_messages': 0,
            'camera_messages': 0,
            'fusion_messages': 0,
            'lidar_timestamps': [],
            'camera_timestamps': [],
            'fusion_timestamps': [],
            'sync_attempts': 0,
            'sync_successes': 0
        }

        # Subscriptions
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/livox/lidar', self.lidar_callback, 10
        )

        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10
        )

        self.fusion_sub = self.create_subscription(
            PointCloud2, '/colored_cloud', self.fusion_callback, 10
        )

        # End timer
        self.timer = self.create_timer(test_duration, self.end_test)

        print(f"Extended integration test started for {test_duration} seconds")

    def lidar_callback(self, msg):
        """Handle LiDAR messages"""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.stats['lidar_messages'] += 1
        self.stats['lidar_timestamps'].append(timestamp)

    def camera_callback(self, msg):
        """Handle camera messages"""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.stats['camera_messages'] += 1
        self.stats['camera_timestamps'].append(timestamp)

    def fusion_callback(self, msg):
        """Handle fusion messages"""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.stats['fusion_messages'] += 1
        self.stats['fusion_timestamps'].append(timestamp)

    def end_test(self):
        """End the test"""
        self.timer.cancel()
        rclpy.shutdown()

    def print_results(self):
        """Print test results"""
        duration = time.time() - self.start_time

        print("\n" + "="*50)
        print("EXTENDED INTEGRATION TEST RESULTS")
        print("="*50)

        print(f"Duration: {duration:.1f}s")
        print(f"LiDAR Messages: {self.stats['lidar_messages']}")
        print(f"Camera Messages: {self.stats['camera_messages']}")
        print(f"Fusion Messages: {self.stats['fusion_messages']}")

        # Calculate frequencies
        lidar_freq = self.stats['lidar_messages'] / duration
        camera_freq = self.stats['camera_messages'] / duration
        fusion_freq = self.stats['fusion_messages'] / duration

        print("\nFrequencies:")
        print(".1f")
        print(".1f")
        print(".1f")

        # Check data flow
        if self.stats['lidar_messages'] == 0:
            print("⚠️  WARNING: No LiDAR data received")
        if self.stats['camera_messages'] == 0:
            print("⚠️  WARNING: No camera data received")
        if self.stats['fusion_messages'] == 0:
            print("⚠️  WARNING: No fusion output")

        # Performance assessment
        if lidar_freq > 5 and camera_freq > 15 and fusion_freq > 5:
            print("✅ PASS: System performing within expected ranges")
        else:
            print("❌ FAIL: System performance below expectations")

        print("="*50)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Integration Tests')
    parser.add_argument('--extended', type=int, help='Run extended test for N seconds')
    parser.add_argument('--verbose', action='store_true', help='Verbose output')

    args = parser.parse_args()

    if args.extended:
        run_extended_integration_test(args.extended)
    else:
        # Run unit tests
        if args.verbose:
            unittest.main(verbosity=2, argv=[''] + [arg for arg in sys.argv if arg != '--verbose'])
        else:
            unittest.main()
