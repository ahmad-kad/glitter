#!/usr/bin/env python3
"""
Rolling Buffer with Motion Compensation
=======================================

Temporal buffer for motion-aware frame synchronization and interpolation.
Handles network jitter and provides motion compensation between frames.
"""

import time
import threading
from collections import deque
from typing import Optional, Tuple, List, Dict, Any
import numpy as np


class FrameData:
    """Container for frame data with timing information"""

    def __init__(self, timestamp: float, data: Any = None, pose: Optional[np.ndarray] = None):
        self.timestamp = timestamp
        self.data = data
        self.pose = pose or np.eye(4)
        self.processed = False
        self.motion_compensated = False

    def get_age(self) -> float:
        """Get age of frame in seconds"""
        return time.time() - self.timestamp


class MotionEstimator:
    """Estimates motion between consecutive frames"""

    def __init__(self, max_history: int = 10):
        self.pose_history: deque = deque(maxlen=max_history)
        self.velocity_history: deque = deque(maxlen=max_history)
        self.last_pose = np.eye(4)
        self.last_timestamp = 0.0

        # Motion model parameters
        self.process_noise = 0.1  # Process noise for Kalman-like filtering
        self.measurement_noise = 0.05  # Measurement noise

    def add_pose_measurement(self, timestamp: float, pose: np.ndarray):
        """Add pose measurement for motion estimation"""
        if self.last_timestamp > 0:
            dt = timestamp - self.last_timestamp

            if dt > 0:
                # Estimate velocity from pose change
                pose_change = np.linalg.inv(self.last_pose) @ pose

                # Extract twist (velocity) from pose change
                velocity = self._pose_to_twist(pose_change, dt)
                self.velocity_history.append((timestamp, velocity))

        self.pose_history.append((timestamp, pose.copy()))
        self.last_pose = pose.copy()
        self.last_timestamp = timestamp

    def predict_pose(self, target_timestamp: float) -> np.ndarray:
        """Predict pose at target timestamp using motion model"""
        if not self.velocity_history:
            return self.last_pose

        # Use most recent velocity estimate
        _, recent_velocity = self.velocity_history[-1]
        dt = target_timestamp - self.last_timestamp

        # Integrate velocity to predict pose change
        pose_change = self._twist_to_pose(recent_velocity, dt)

        # Apply pose change to last known pose
        predicted_pose = self.last_pose @ pose_change

        return predicted_pose

    def get_smoothed_velocity(self) -> Optional[np.ndarray]:
        """Get smoothed velocity estimate"""
        if not self.velocity_history:
            return None

        # Simple moving average of recent velocities
        recent_velocities = [vel for _, vel in list(self.velocity_history)[-5:]]
        if recent_velocities:
            return np.mean(recent_velocities, axis=0)

        return None

    def _pose_to_twist(self, pose: np.ndarray, dt: float) -> np.ndarray:
        """Convert pose change to twist (velocity)"""
        # Extract rotation and translation
        R = pose[:3, :3]
        t = pose[:3, 3]

        # Angular velocity from rotation
        angle, axis = self._rotation_matrix_to_axis_angle(R)

        if angle > 1e-8:
            w = axis * (angle / dt)  # Angular velocity
        else:
            w = np.zeros(3)

        # Linear velocity
        v = t / dt  # Linear velocity

        return np.concatenate([v, w])

    def _twist_to_pose(self, twist: np.ndarray, dt: float) -> np.ndarray:
        """Convert twist to pose change"""
        v = twist[:3]  # Linear velocity
        w = twist[3:]  # Angular velocity

        # Translation
        translation = v * dt

        # Rotation
        angle = np.linalg.norm(w) * dt
        if angle > 1e-8:
            axis = w / np.linalg.norm(w)
            R = self._axis_angle_to_rotation_matrix(axis, angle)
        else:
            R = np.eye(3)

        # Compose pose change
        pose_change = np.eye(4)
        pose_change[:3, :3] = R
        pose_change[:3, 3] = translation

        return pose_change

    @staticmethod
    def _rotation_matrix_to_axis_angle(R: np.ndarray) -> Tuple[float, np.ndarray]:
        """Convert rotation matrix to axis-angle representation"""
        # Rodrigues' formula
        angle = np.arccos((np.trace(R) - 1) / 2)

        if abs(angle) < 1e-8:
            return 0.0, np.array([1, 0, 0])  # Identity rotation

        axis = np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1]
        ])
        axis = axis / (2 * np.sin(angle))

        return angle, axis

    @staticmethod
    def _axis_angle_to_rotation_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
        """Convert axis-angle to rotation matrix"""
        axis = axis / np.linalg.norm(axis)
        a = np.cos(angle / 2)
        b, c, d = -axis * np.sin(angle / 2)

        return np.array([
            [a*a+b*b-c*c-d*d, 2*(b*c-a*d), 2*(b*d+a*c)],
            [2*(b*c+a*d), a*a+c*c-b*b-d*d, 2*(c*d-a*b)],
            [2*(b*d-a*c), 2*(c*d+a*b), a*a+d*d-b*b-c*c]
        ])


class RollingBuffer:
    """
    Rolling buffer for temporal synchronization with motion compensation.

    Handles network jitter, sensor timing drift, and provides motion-aware
    frame interpolation between sensor measurements.
    """

    def __init__(self, capacity: int = 30, max_age: float = 1.0):
        self.capacity = capacity
        self.max_age = max_age

        # Buffer storage (thread-safe)
        self.lock = threading.RLock()
        self.lidar_buffer: deque = deque(maxlen=capacity)
        self.camera_buffer: deque = deque(maxlen=capacity)

        # Motion estimation
        self.motion_estimator = MotionEstimator()

        # Statistics
        self.stats = {
            'frames_received_lidar': 0,
            'frames_received_camera': 0,
            'sync_successes': 0,
            'sync_failures': 0,
            'motion_compensations': 0,
            'buffer_overruns': 0,
            'avg_sync_latency': 0.0
        }

    def add_lidar_frame(self, timestamp: float, data: Any, pose: Optional[np.ndarray] = None):
        """Add LiDAR frame to buffer"""
        with self.lock:
            frame = FrameData(timestamp, data, pose)
            self.lidar_buffer.append(frame)
            self.stats['frames_received_lidar'] += 1

            # Update motion estimate if pose available
            if pose is not None:
                self.motion_estimator.add_pose_measurement(timestamp, pose)

            self._cleanup_old_frames()

    def add_camera_frame(self, timestamp: float, data: Any, pose: Optional[np.ndarray] = None):
        """Add camera frame to buffer"""
        with self.lock:
            frame = FrameData(timestamp, data, pose)
            self.camera_buffer.append(frame)
            self.stats['frames_received_camera'] += 1

            # Update motion estimate if pose available
            if pose is not None:
                self.motion_estimator.add_pose_measurement(timestamp, pose)

            self._cleanup_old_frames()

    def get_synced_pair(self, tolerance: float = 0.1) -> Optional[Tuple[FrameData, FrameData]]:
        """
        Get synchronized LiDAR-camera pair closest in time.

        Args:
            tolerance: Maximum time difference for sync (seconds)

        Returns:
            Tuple of (lidar_frame, camera_frame) or None if no valid pair
        """
        with self.lock:
            if not self.lidar_buffer or not self.camera_buffer:
                return None

            # Find best matching pair
            best_pair = None
            best_time_diff = float('inf')

            for lidar_frame in reversed(self.lidar_buffer):
                for camera_frame in reversed(self.camera_buffer):
                    time_diff = abs(lidar_frame.timestamp - camera_frame.timestamp)

                    if time_diff <= tolerance and time_diff < best_time_diff:
                        best_time_diff = time_diff
                        best_pair = (lidar_frame, camera_frame)

                        # Early exit if we find perfect sync
                        if time_diff < 0.001:  # 1ms tolerance
                            break

            if best_pair:
                self.stats['sync_successes'] += 1

                # Update sync latency stats
                total_syncs = self.stats['sync_successes'] + self.stats['sync_failures']
                self.stats['avg_sync_latency'] = (
                    (self.stats['avg_sync_latency'] * (total_syncs - 1)) + best_time_diff
                ) / total_syncs

                return best_pair
            else:
                self.stats['sync_failures'] += 1
                return None

    def get_motion_compensated_pair(self, tolerance: float = 0.1) -> Optional[Tuple[FrameData, FrameData]]:
        """
        Get motion-compensated synchronized pair.

        Uses motion estimation to interpolate frames when direct sync fails.
        """
        # Try direct sync first
        direct_pair = self.get_synced_pair(tolerance)
        if direct_pair:
            return direct_pair

        # Fall back to motion compensation
        return self._find_motion_compensated_pair(tolerance)

    def _find_motion_compensated_pair(self, tolerance: float) -> Optional[Tuple[FrameData, FrameData]]:
        """Find pair using motion compensation"""
        with self.lock:
            if not self.lidar_buffer or not self.camera_buffer:
                return None

            # Find frames that could be motion-compensated
            for lidar_frame in reversed(self.lidar_buffer):
                for camera_frame in reversed(self.camera_buffer):
                    time_diff = abs(lidar_frame.timestamp - camera_frame.timestamp)

                    # If time difference is reasonable for motion compensation
                    if time_diff <= tolerance * 2:  # Allow larger tolerance for motion comp
                        # Create motion-compensated versions
                        compensated_lidar = self._compensate_frame(lidar_frame, camera_frame.timestamp)
                        compensated_camera = self._compensate_frame(camera_frame, lidar_frame.timestamp)

                        if compensated_lidar and compensated_camera:
                            self.stats['motion_compensations'] += 1
                            return compensated_lidar, compensated_camera

            return None

    def _compensate_frame(self, source_frame: FrameData, target_timestamp: float) -> Optional[FrameData]:
        """Create motion-compensated version of frame at target timestamp"""
        dt = target_timestamp - source_frame.timestamp

        # Don't extrapolate too far into the future/past
        if abs(dt) > 0.5:  # 500ms limit
            return None

        # Predict pose at target timestamp
        predicted_pose = self.motion_estimator.predict_pose(target_timestamp)

        # Create compensated frame
        compensated_frame = FrameData(
            timestamp=target_timestamp,
            data=source_frame.data,  # Data remains the same
            pose=predicted_pose
        )
        compensated_frame.motion_compensated = True

        return compensated_frame

    def _cleanup_old_frames(self):
        """Remove frames older than max_age"""
        current_time = time.time()

        # Clean lidar buffer
        while self.lidar_buffer and (current_time - self.lidar_buffer[0].timestamp) > self.max_age:
            self.lidar_buffer.popleft()
            self.stats['buffer_overruns'] += 1

        # Clean camera buffer
        while self.camera_buffer and (current_time - self.camera_buffer[0].timestamp) > self.max_age:
            self.camera_buffer.popleft()
            self.stats['buffer_overruns'] += 1

    def get_buffer_stats(self) -> Dict[str, Any]:
        """Get buffer statistics"""
        with self.lock:
            return {
                'lidar_buffer_size': len(self.lidar_buffer),
                'camera_buffer_size': len(self.camera_buffer),
                'lidar_frames_received': self.stats['frames_received_lidar'],
                'camera_frames_received': self.stats['frames_received_camera'],
                'sync_success_rate': (
                    self.stats['sync_successes'] /
                    max(1, self.stats['sync_successes'] + self.stats['sync_failures']) * 100
                ),
                'motion_compensations': self.stats['motion_compensations'],
                'avg_sync_latency_ms': self.stats['avg_sync_latency'] * 1000,
                'buffer_overruns': self.stats['buffer_overruns']
            }

    def reset_stats(self):
        """Reset statistics counters"""
        with self.lock:
            for key in self.stats:
                if key.startswith('avg_'):
                    continue  # Keep averages
                self.stats[key] = 0


class TemporalSyncBenchmark:
    """Benchmark temporal synchronization performance"""

    def __init__(self):
        self.buffer = RollingBuffer(capacity=50, max_age=2.0)

        # Simulated timing parameters
        self.lidar_frequency = 10.0  # 10 Hz
        self.camera_frequency = 30.0  # 30 Hz
        self.network_jitter = 0.05  # 50ms jitter

    def simulate_realistic_timing(self, duration: float = 10.0) -> Dict[str, Any]:
        """Simulate realistic sensor timing with network jitter"""
        print(f"Simulating {duration}s of realistic sensor timing...")

        start_time = time.time()
        simulation_time = 0.0

        # Generate timestamps with realistic timing
        lidar_timestamps = []
        camera_timestamps = []

        while simulation_time < duration:
            # LiDAR at 10Hz with jitter
            if not lidar_timestamps or simulation_time - lidar_timestamps[-1] >= 1.0/self.lidar_frequency:
                jitter = np.random.normal(0, self.network_jitter)
                timestamp = simulation_time + jitter
                lidar_timestamps.append(timestamp)

                # Add to buffer
                self.buffer.add_lidar_frame(timestamp, f"lidar_frame_{len(lidar_timestamps)}")

            # Camera at 30Hz with different jitter
            if not camera_timestamps or simulation_time - camera_timestamps[-1] >= 1.0/self.camera_frequency:
                jitter = np.random.normal(0, self.network_jitter * 0.5)  # Less jitter for camera
                timestamp = simulation_time + jitter
                camera_timestamps.append(timestamp)

                # Add to buffer
                self.buffer.add_camera_frame(timestamp, f"camera_frame_{len(camera_timestamps)}")

            simulation_time += 0.001  # 1ms simulation steps

        # Test synchronization
        sync_attempts = 0
        sync_successes = 0
        latencies = []

        for _ in range(100):  # Test 100 sync attempts
            sync_attempts += 1
            pair = self.buffer.get_synced_pair(tolerance=0.1)
            if pair:
                sync_successes += 1
                latency = abs(pair[0].timestamp - pair[1].timestamp)
                latencies.append(latency)

        elapsed = time.time() - start_time

        results = {
            'simulation_duration': duration,
            'lidar_frames_generated': len(lidar_timestamps),
            'camera_frames_generated': len(camera_timestamps),
            'sync_attempts': sync_attempts,
            'sync_successes': sync_successes,
            'sync_success_rate': sync_successes / sync_attempts * 100,
            'avg_sync_latency_ms': sum(latencies) / len(latencies) * 1000 if latencies else 0,
            'buffer_stats': self.buffer.get_buffer_stats(),
            'computation_time': elapsed
        }

        print(".1f")
        print(".1f")
        print(".1f")

        return results

    def test_motion_compensation(self) -> Dict[str, Any]:
        """Test motion compensation capabilities"""
        print("Testing motion compensation...")

        # Add some frames with known motion
        base_time = time.time()

        # Simulate moving sensor
        for i in range(10):
            timestamp = base_time + i * 0.1  # 10Hz

            # Simulate translation (moving forward)
            pose = np.eye(4)
            pose[2, 3] = i * 0.1  # Move 0.1m per frame in Z

            self.buffer.add_lidar_frame(timestamp, f"motion_frame_{i}", pose)

        # Test motion compensation
        test_timestamp = base_time + 0.55  # Between frames 5 and 6
        compensated_frame = self.buffer._compensate_frame(
            self.buffer.lidar_buffer[5], test_timestamp
        )

        expected_position = 5.5 * 0.1  # Should be at 0.55m
        actual_position = compensated_frame.pose[2, 3] if compensated_frame else 0

        return {
            'test_timestamp': test_timestamp,
            'expected_position': expected_position,
            'actual_position': actual_position,
            'position_error': abs(actual_position - expected_position),
            'compensation_successful': compensated_frame is not None
        }


if __name__ == "__main__":
    # Run benchmark
    benchmark = TemporalSyncBenchmark()

    print("=== Rolling Buffer Benchmark ===")

    # Test realistic timing
    timing_results = benchmark.simulate_realistic_timing(5.0)

    # Test motion compensation
    motion_results = benchmark.test_motion_compensation()

    print("\nMotion Compensation Test:")
    print(".3f")
    print(".3f")
    print(".6f")

    print("\nBenchmark completed!")
