# This file has been moved to src/infrastructure/infrastructure.py
# Please update imports to use the new location

import time
import threading
from enum import Enum
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass


@dataclass
class SystemHealth:
    """Simplified system health status"""
    sensors_ok: bool = True
    network_ok: bool = True
    thermal_ok: bool = True
    power_ok: bool = True
    overall_ok: bool = True


class FusionMode(Enum):
    """Available processing modes"""
    FULL = "full"           # All sensors available
    BASIC = "basic"         # Core functionality only
    MINIMAL = "minimal"     # Emergency mode


class RealWorldInfrastructure:
    """
    Simplified, maintainable infrastructure combining all real-world capabilities.
    One cohesive class instead of multiple complex classes.
    """

    def __init__(self, node, target_fps: float = 30.0):
        self.node = node
        self.target_fps = target_fps
        self.current_fps = 0.0

        # Health tracking
        self.health = SystemHealth()
        self.health_history: List[SystemHealth] = []

        # Sensor tracking (simple dict instead of complex classes)
        self.sensor_status = {
            'lidar': {'alive': True, 'last_seen': 0, 'frequency': 0},
            'camera': {'alive': True, 'last_seen': 0, 'frequency': 0},
            'imu': {'alive': False, 'last_seen': 0, 'frequency': 0}  # Optional
        }

        # Network monitoring (simple ping-pong)
        self.network_quality = 1.0  # 0.0 to 1.0
        self.last_ping_time = 0
        self.ping_responses = []

        # Thermal management (basic)
        self.cpu_temp = 50.0
        self.thermal_throttling = False

        # Processing adaptation
        self.fusion_mode = FusionMode.FULL
        self.processing_params = {
            'compression_level': 6,
            'downsample_factor': 1,
            'diagnostic_frequency': 5
        }

        # Monitoring threads (simple, not complex)
        self.monitoring_active = False
        self.monitor_thread = None

        # Callbacks for mode changes
        self.mode_change_callbacks: List[Callable] = []

        self.node.get_logger().info("Real-world infrastructure initialized")

    def start_monitoring(self):
        """Start background monitoring"""
        if self.monitoring_active:
            return

        self.monitoring_active = True
        self.monitor_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self.monitor_thread.start()

    def stop_monitoring(self):
        """Stop background monitoring"""
        self.monitoring_active = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)

    def update_sensor_health(self, sensor_name: str, timestamp: float):
        """Update sensor health (simple)"""
        if sensor_name not in self.sensor_status:
            return

        status = self.sensor_status[sensor_name]
        status['last_seen'] = timestamp

        # Calculate frequency
        if status['last_frequency_update']:
            dt = timestamp - status['last_frequency_update']
            if dt > 0:
                status['frequency'] = 1.0 / dt

        status['last_frequency_update'] = timestamp
        status['alive'] = (time.time() - timestamp) < 2.0  # 2 second timeout

        self._update_overall_health()

    def get_network_quality(self) -> float:
        """Get current network quality (0.0 to 1.0)"""
        return self.network_quality

    def should_compress(self) -> bool:
        """Should use compression based on network conditions"""
        return self.network_quality < 0.7

    def get_processing_params(self) -> Dict[str, Any]:
        """Get adaptive processing parameters"""
        return self.processing_params.copy()

    def update_performance(self, current_fps: float):
        """Update performance feedback"""
        self.current_fps = current_fps

        # Simple adaptation logic
        if self.cpu_temp > 75.0:
            self.processing_params['compression_level'] = 1  # Fast compression
            self.processing_params['downsample_factor'] = 2  # Downsample more
        elif self.current_fps < self.target_fps * 0.8:
            self.processing_params['compression_level'] = 9  # Best compression
            self.processing_params['downsample_factor'] = 1  # Full resolution
        else:
            self.processing_params['compression_level'] = 6  # Balanced
            self.processing_params['downsample_factor'] = 1  # Full resolution

    def get_fusion_mode(self) -> FusionMode:
        """Get current fusion mode"""
        return self.fusion_mode

    def get_system_health(self) -> SystemHealth:
        """Get current system health"""
        return self.health

    def add_mode_change_callback(self, callback: Callable):
        """Add callback for mode changes"""
        self.mode_change_callbacks.append(callback)

    def _monitoring_loop(self):
        """Simple monitoring loop"""
        while self.monitoring_active:
            try:
                self._update_thermal_status()
                self._update_network_status()
                self._update_overall_health()
                self._check_mode_changes()

                time.sleep(1.0)  # Simple 1Hz monitoring

            except Exception as e:
                self.node.get_logger().error(f"Monitoring error: {e}")
                time.sleep(2.0)

    def _update_thermal_status(self):
        """Update thermal status (simple)"""
        try:
            # Simple temperature reading
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp_milli = int(f.read().strip())
                self.cpu_temp = temp_milli / 1000.0
        except:
            self.cpu_temp = 50.0  # Safe default

        # Simple thermal logic
        self.health.thermal_ok = self.cpu_temp < 80.0
        self.thermal_throttling = self.cpu_temp > 75.0

    def _update_network_status(self):
        """Update network status (simple ping simulation)"""
        # Simplified network quality based on system load
        # In real implementation, would do actual ping tests
        self.network_quality = max(0.3, 1.0 - (self.cpu_temp - 50.0) / 50.0)
        self.health.network_ok = self.network_quality > 0.5

    def _update_overall_health(self):
        """Update overall system health"""
        # Simple logic: all components must be OK
        sensors_ok = all(status['alive'] for status in self.sensor_status.values()
                        if status.get('required', True))

        self.health.sensors_ok = sensors_ok
        self.health.power_ok = True  # Simplified
        self.health.overall_ok = all([
            self.health.sensors_ok,
            self.health.network_ok,
            self.health.thermal_ok,
            self.health.power_ok
        ])

        # Keep health history (last 10 states)
        self.health_history.append(self.health)
        if len(self.health_history) > 10:
            self.health_history.pop(0)

    def _check_mode_changes(self):
        """Check if fusion mode should change"""
        old_mode = self.fusion_mode

        if not self.health.overall_ok:
            if self.health.sensors_ok:
                self.fusion_mode = FusionMode.BASIC
            else:
                self.fusion_mode = FusionMode.MINIMAL
        else:
            self.fusion_mode = FusionMode.FULL

        # Notify if mode changed
        if old_mode != self.fusion_mode:
            self.node.get_logger().info(f"Fusion mode: {old_mode.value} -> {self.fusion_mode.value}")
            for callback in self.mode_change_callbacks:
                try:
                    callback(old_mode, self.fusion_mode)
                except Exception as e:
                    self.node.get_logger().error(f"Mode change callback error: {e}")


class CompressedPublisher:
    """Simplified compression publisher"""

    def __init__(self, node, topic_name: str, msg_type, compression_type: str = "lz4"):
        self.node = node
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.compression_type = compression_type

        # Create publisher (would use QoS in real implementation)
        try:
            self.publisher = node.create_publisher(msg_type, topic_name, 10)
        except:
            self.publisher = None  # ROS not available

    def publish(self, msg):
        """Publish with optional compression"""
        if self.publisher:
            self.publisher.publish(msg)


# Utility functions for easy integration
def create_infrastructure(node, target_fps: float = 30.0) -> RealWorldInfrastructure:
    """Create and start infrastructure"""
    infra = RealWorldInfrastructure(node, target_fps)
    infra.start_monitoring()
    return infra


def create_compressed_publisher(node, topic_name: str, msg_type, compression: str = "lz4"):
    """Create compressed publisher"""
    return CompressedPublisher(node, topic_name, msg_type, compression)


# Example usage
"""
Simple integration:

class SimpleFusionNode(Node):
    def __init__(self):
        super().__init__('simple_fusion')

        # One line infrastructure setup
        self.infra = create_infrastructure(self, target_fps=30.0)

        # Compressed publisher
        self.publisher = create_compressed_publisher(self, '/fused_cloud', PointCloud2)

        # Add mode change handler
        self.infra.add_mode_change_callback(self.on_mode_change)

        # Simple subscriptions
        self.lidar_sub = self.create_subscription(PointCloud2, '/lidar', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera', self.camera_callback, 10)

    def lidar_callback(self, msg):
        self.infra.update_sensor_health('lidar', self.get_clock().now().nanoseconds / 1e9)
        # Process...

    def camera_callback(self, msg):
        self.infra.update_sensor_health('camera', self.get_clock().now().nanoseconds / 1e9)
        # Process...

    def on_mode_change(self, old_mode, new_mode):
        self.get_logger().info(f"Adapting to mode: {new_mode.value}")
        # Adjust processing based on mode
"""


def get_memory_usage() -> float:
    """Get current memory usage in MB"""
    try:
        import psutil
        process = psutil.Process()
        return process.memory_info().rss / 1024 / 1024
    except ImportError:
        return 0.0  # Fallback if psutil not available


def get_cpu_usage() -> float:
    """Get current CPU usage percentage"""
    try:
        import psutil
        return psutil.cpu_percent(interval=0.1)
    except ImportError:
        return 0.0  # Fallback if psutil not available
