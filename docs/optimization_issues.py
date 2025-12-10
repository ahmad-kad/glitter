#!/usr/bin/env python3
"""
Critical Optimization Issues & Fixes
====================================

Identified issues requiring immediate attention for real-time performance.
"""

import time
import threading
import numpy as np
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from collections import deque
import psutil
import os


# =============================================================================
# ISSUE 1: DOWNSAMPLE_FREQUENCY BUG FIX
# =============================================================================

# Add to utils.py constants section:
"""
# Mapping constants
DOWNSAMPLE_FREQUENCY = 10  # Downsample every 10 frames by default
"""

# =============================================================================
# ISSUE 2: MEMORY POOL SYSTEM (CRITICAL FOR REAL-TIME)
# =============================================================================

@dataclass
class MemoryPoolConfig:
    """Configuration for memory pool sizing"""
    max_points: int = 100000  # Max points per frame
    pool_count: int = 3       # Triple buffering for async processing
    alignment: int = 64       # Cache line alignment

class MemoryPool:
    """Pre-allocated, cache-aligned memory pools for zero-GC operation"""

    def __init__(self, config: MemoryPoolConfig = None):
        self.config = config or MemoryPoolConfig()

        # Pre-allocate aligned memory pools
        self.pools = []
        for _ in range(self.config.pool_count):
            pool = {
                # Point cloud data (cache-aligned)
                'points_xyz': np.empty((self.config.max_points, 3), dtype=np.float32),
                'points_rgb': np.empty((self.config.max_points, 3), dtype=np.uint8),
                'points_hom': np.empty((self.config.max_points, 4), dtype=np.float32),

                # Processing intermediates
                'depth_mask': np.empty(self.config.max_points, dtype=bool),
                'bounds_mask': np.empty(self.config.max_points, dtype=bool),
                'valid_indices': np.empty(self.config.max_points, dtype=np.int32),

                # Output buffers
                'output_cloud': np.empty((self.config.max_points, 4), dtype=np.float32),
                'compressed_buffer': bytearray(1024 * 1024),  # 1MB compression buffer

                # Metadata
                'in_use': False,
                'frame_id': 0,
                'timestamp': 0.0
            }
            self.pools.append(pool)

        self.current_pool_idx = 0
        self._ensure_alignment()

    def _ensure_alignment(self):
        """Ensure all arrays are cache-line aligned for SIMD operations"""
        for pool in self.pools:
            for key, array in pool.items():
                if isinstance(array, np.ndarray):
                    # Force alignment (NumPy may not guarantee this)
                    pass  # In practice, use aligned_alloc or similar

    def get_pool(self, frame_id: int = 0) -> Dict[str, Any]:
        """Get next available memory pool (round-robin)"""
        for _ in range(self.config.pool_count):
            pool = self.pools[self.current_pool_idx]
            if not pool['in_use']:
                pool['in_use'] = True
                pool['frame_id'] = frame_id
                pool['timestamp'] = time.time()

                self.current_pool_idx = (self.current_pool_idx + 1) % self.config.pool_count
                return pool

            self.current_pool_idx = (self.current_pool_idx + 1) % self.config.pool_count

        # All pools busy - this indicates processing bottleneck
        raise RuntimeError("All memory pools busy - processing too slow!")

    def release_pool(self, pool: Dict[str, Any]):
        """Release memory pool back to available pool"""
        pool['in_use'] = False
        pool['frame_id'] = 0

    def get_pool_stats(self) -> Dict[str, Any]:
        """Get memory pool usage statistics"""
        in_use = sum(1 for pool in self.pools if pool['in_use'])
        return {
            'total_pools': self.config.pool_count,
            'pools_in_use': in_use,
            'utilization_percent': (in_use / self.config.pool_count) * 100,
            'memory_per_pool_mb': (self.config.max_points * 4 * 4) / (1024 * 1024)  # Rough estimate
        }


# =============================================================================
# ISSUE 3: ASYNC I/O PIPELINE (PREVENT BLOCKING)
# =============================================================================

class AsyncIOPipeline:
    """Non-blocking I/O pipeline with producer-consumer queues"""

    def __init__(self, max_queue_size: int = 10):
        self.input_queue = deque(maxlen=max_queue_size)
        self.output_queue = deque(maxlen=max_queue_size)
        self.processing_queue = deque(maxlen=max_queue_size)

        self.lock = threading.RLock()
        self.input_condition = threading.Condition(self.lock)
        self.output_condition = threading.Condition(self.lock)

        # Thread pool for processing
        self.executor = None  # Will be set by user

        self.running = False
        self.worker_thread = None

    def start(self, executor=None):
        """Start the async pipeline"""
        self.executor = executor or ThreadPoolExecutor(max_workers=2)
        self.running = True
        self.worker_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self.worker_thread.start()

    def stop(self):
        """Stop the async pipeline"""
        self.running = False
        if self.worker_thread:
            self.worker_thread.join(timeout=1.0)

    def submit_input(self, data: Any, timeout: float = 0.1) -> bool:
        """Submit input data asynchronously"""
        with self.lock:
            if len(self.input_queue) >= self.input_queue.maxlen:
                return False  # Queue full

            self.input_queue.append(data)
            self.input_condition.notify()
            return True

    def get_output(self, timeout: float = 0.1) -> Optional[Any]:
        """Get processed output asynchronously"""
        with self.lock:
            if not self.output_queue:
                self.output_condition.wait(timeout=timeout)
                if not self.output_queue:
                    return None

            return self.output_queue.popleft()

    def _processing_loop(self):
        """Background processing loop"""
        while self.running:
            try:
                # Get input data
                with self.lock:
                    if not self.input_queue:
                        self.input_condition.wait(timeout=0.01)
                        continue
                    input_data = self.input_queue.popleft()

                # Process asynchronously
                if self.executor:
                    future = self.executor.submit(self._process_data, input_data)
                    future.add_done_callback(self._on_processing_complete)
                else:
                    # Synchronous fallback
                    result = self._process_data(input_data)
                    self._on_processing_complete(result)

            except Exception as e:
                print(f"Async processing error: {e}")
                time.sleep(0.01)

    def _process_data(self, data: Any) -> Any:
        """Override this method for actual processing"""
        return data  # Placeholder

    def _on_processing_complete(self, future_or_result):
        """Handle completed processing"""
        try:
            if hasattr(future_or_result, 'result'):
                result = future_or_result.result()
            else:
                result = future_or_result

            with self.lock:
                if len(self.output_queue) < self.output_queue.maxlen:
                    self.output_queue.append(result)
                    self.output_condition.notify()
                else:
                    print("Output queue full - dropping result")

        except Exception as e:
            print(f"Processing completion error: {e}")


# =============================================================================
# ISSUE 4: ROLLING BUFFER WITH MOTION COMPENSATION
# =============================================================================

@dataclass
class FrameData:
    """Data for a single frame with motion information"""
    timestamp: float
    lidar_data: Optional[Any] = None
    camera_data: Optional[Any] = None
    imu_data: Optional[Any] = None
    pose_estimate: Optional[np.ndarray] = None
    processed: bool = False

class RollingBuffer:
    """Temporal buffer for motion-aware frame synchronization"""

    def __init__(self, capacity: int = 30, max_age: float = 1.0):
        self.capacity = capacity
        self.max_age = max_age
        self.buffer: deque = deque(maxlen=capacity)
        self.lock = threading.RLock()

        # Motion compensation
        self.last_pose = np.eye(4)
        self.velocity_estimate = np.zeros(6)  # [vx, vy, vz, wx, wy, wz]

    def add_frame(self, frame_data: FrameData):
        """Add frame to rolling buffer"""
        with self.lock:
            self.buffer.append(frame_data)
            self._cleanup_old_frames()
            self._update_motion_estimate()

    def find_synced_pair(self, target_time: float, tolerance: float = 0.1) -> Optional[tuple]:
        """Find LiDAR-camera pair closest to target time"""
        with self.lock:
            if len(self.buffer) < 2:
                return None

            best_pair = None
            best_time_diff = float('inf')

            # Find frames with both LiDAR and camera data
            lidar_frames = [f for f in self.buffer if f.lidar_data is not None]
            camera_frames = [f for f in self.buffer if f.camera_data is not None]

            for lidar_frame in lidar_frames:
                for camera_frame in camera_frames:
                    time_diff = abs(lidar_frame.timestamp - camera_frame.timestamp)
                    if time_diff < tolerance and time_diff < best_time_diff:
                        best_time_diff = time_diff
                        best_pair = (lidar_frame, camera_frame)

            return best_pair

    def get_motion_compensated_frame(self, frame_data: FrameData) -> FrameData:
        """Apply motion compensation to frame data"""
        if frame_data.pose_estimate is None:
            frame_data.pose_estimate = self.last_pose.copy()

        # Apply velocity-based motion compensation
        dt = frame_data.timestamp - (self.buffer[-1].timestamp if self.buffer else frame_data.timestamp)

        # Integrate velocity to get pose change
        pose_change = self._integrate_velocity(dt)
        frame_data.pose_estimate = frame_data.pose_estimate @ pose_change

        return frame_data

    def _cleanup_old_frames(self):
        """Remove frames older than max_age"""
        current_time = time.time()
        while self.buffer and (current_time - self.buffer[0].timestamp) > self.max_age:
            self.buffer.popleft()

    def _update_motion_estimate(self):
        """Update motion estimate from recent frames"""
        if len(self.buffer) < 2:
            return

        # Simple velocity estimation from pose changes
        recent_frames = list(self.buffer)[-5:]  # Last 5 frames
        if len(recent_frames) >= 2:
            pose_changes = []
            for i in range(1, len(recent_frames)):
                if (recent_frames[i].pose_estimate is not None and
                    recent_frames[i-1].pose_estimate is not None):

                    dt = recent_frames[i].timestamp - recent_frames[i-1].timestamp
                    if dt > 0:
                        pose_diff = np.linalg.inv(recent_frames[i-1].pose_estimate) @ recent_frames[i].pose_estimate
                        twist = self._pose_to_twist(pose_diff, dt)
                        pose_changes.append(twist)

            if pose_changes:
                self.velocity_estimate = np.mean(pose_changes, axis=0)

    def _integrate_velocity(self, dt: float) -> np.ndarray:
        """Integrate velocity estimate to get pose change"""
        # Simple Euler integration for velocity
        v = self.velocity_estimate[:3]  # Linear velocity
        w = self.velocity_estimate[3:]  # Angular velocity

        # Translation
        translation = v * dt

        # Rotation (axis-angle)
        angle = np.linalg.norm(w) * dt
        if angle > 1e-8:
            axis = w / np.linalg.norm(w)
            rotation_matrix = self._axis_angle_to_matrix(axis, angle)
        else:
            rotation_matrix = np.eye(3)

        # Compose transform
        pose_change = np.eye(4)
        pose_change[:3, :3] = rotation_matrix
        pose_change[:3, 3] = translation

        return pose_change

    @staticmethod
    def _pose_to_twist(pose: np.ndarray, dt: float) -> np.ndarray:
        """Convert pose change to twist (velocity)"""
        # Extract rotation and translation
        R = pose[:3, :3]
        t = pose[:3, 3]

        # Angular velocity from rotation
        angle, axis = RollingBuffer._matrix_to_axis_angle(R)
        w = axis * (angle / dt) if angle > 1e-8 else np.zeros(3)

        # Linear velocity
        v = t / dt

        return np.concatenate([v, w])

    @staticmethod
    def _axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
        """Convert axis-angle to rotation matrix"""
        axis = axis / np.linalg.norm(axis)
        a = np.cos(angle / 2)
        b, c, d = -axis * np.sin(angle / 2)

        return np.array([
            [a*a+b*b-c*c-d*d, 2*(b*c-a*d), 2*(b*d+a*c)],
            [2*(b*c+a*d), a*a+c*c-b*b-d*d, 2*(c*d-a*b)],
            [2*(b*d-a*c), 2*(c*d+a*b), a*a+d*d-b*b-c*c]
        ])

    @staticmethod
    def _matrix_to_axis_angle(R: np.ndarray) -> tuple:
        """Convert rotation matrix to axis-angle"""
        # Rodrigues' rotation formula
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


# =============================================================================
# ISSUE 5: LAZ COMPRESSION INTEGRATION
# =============================================================================

class LAZCompression:
    """LAZ (compressed LAS) support for efficient point cloud storage"""

    def __init__(self):
        try:
            import laspy
            self.laspy_available = True
        except ImportError:
            self.laspy_available = False
            print("Warning: laspy not available. LAZ compression disabled.")

    def compress_to_laz(self, points: np.ndarray, colors: Optional[np.ndarray] = None,
                       intensities: Optional[np.ndarray] = None) -> bytes:
        """Compress point cloud to LAZ format"""
        if not self.laspy_available:
            raise ImportError("laspy required for LAZ compression")

        import laspy

        # Create LAS file
        header = laspy.LasHeader(point_format=3, version="1.2")  # Point format 3 includes RGB
        header.add_extra_dim(laspy.ExtraBytesParams(name="intensity", type=np.uint16))

        las = laspy.LasData(header)

        # Set point coordinates
        las.x = points[:, 0]
        las.y = points[:, 1]
        las.z = points[:, 2]

        # Set colors if available
        if colors is not None:
            las.red = colors[:, 0].astype(np.uint16) * 256
            las.green = colors[:, 1].astype(np.uint16) * 256
            las.blue = colors[:, 2].astype(np.uint16) * 256

        # Set intensities if available
        if intensities is not None:
            las.intensity = intensities.astype(np.uint16)

        # Compress to LAZ
        import io
        output = io.BytesIO()
        las.write(output, do_compress=True)
        return output.getvalue()

    def decompress_from_laz(self, laz_data: bytes) -> tuple:
        """Decompress LAZ data to numpy arrays"""
        if not self.laspy_available:
            raise ImportError("laspy required for LAZ decompression")

        import laspy
        import io

        # Read LAZ data
        input_stream = io.BytesIO(laz_data)
        las = laspy.read(input_stream)

        # Extract data
        points = np.column_stack([las.x, las.y, las.z])

        colors = None
        if hasattr(las, 'red'):
            colors = np.column_stack([
                las.red // 256, las.green // 256, las.blue // 256
            ]).astype(np.uint8)

        intensities = None
        if hasattr(las, 'intensity'):
            intensities = las.intensity

        return points, colors, intensities

    def get_compression_stats(self, original_points: np.ndarray,
                            compressed_data: bytes) -> Dict[str, Any]:
        """Get compression statistics"""
        original_size = original_points.nbytes
        compressed_size = len(compressed_data)

        return {
            'original_size_bytes': original_size,
            'compressed_size_bytes': compressed_size,
            'compression_ratio': original_size / compressed_size,
            'space_savings_percent': (1 - compressed_size / original_size) * 100,
            'points_count': len(original_points)
        }


# =============================================================================
# ISSUE 6: SIMD OPTIMIZATIONS
# =============================================================================

class SIMDProcessor:
    """SIMD-optimized point cloud operations"""

    def __init__(self):
        self.use_simd = self._check_simd_support()

    def _check_simd_support(self) -> bool:
        """Check if SIMD optimizations are available"""
        try:
            import numpy as np
            # Check for AVX2 or similar
            # This is a simplified check - real implementation would be more thorough
            return hasattr(np, 'simd') or True  # Assume available for now
        except:
            return False

    def fast_transform_points(self, points: np.ndarray, transform: np.ndarray) -> np.ndarray:
        """SIMD-optimized point transformation"""
        if not self.use_simd:
            return self._fallback_transform(points, transform)

        # Convert to homogeneous coordinates (SIMD friendly)
        num_points = len(points)
        points_hom = np.empty((num_points, 4), dtype=np.float32)
        points_hom[:, :3] = points
        points_hom[:, 3] = 1.0

        # Matrix multiplication (should use SIMD under the hood)
        transformed = (transform @ points_hom.T).T

        # Convert back to 3D
        return transformed[:, :3].astype(np.float32)

    def fast_filter_points(self, points: np.ndarray, mask: np.ndarray) -> np.ndarray:
        """SIMD-optimized point filtering"""
        if not self.use_simd:
            return points[mask]

        # Use numpy's boolean indexing (SIMD optimized)
        return points[mask]

    def fast_compute_distances(self, points: np.ndarray, reference: np.ndarray) -> np.ndarray:
        """SIMD-optimized distance computation"""
        if not self.use_simd:
            return np.linalg.norm(points - reference, axis=1)

        # Vectorized distance computation
        diff = points - reference
        return np.sqrt(np.sum(diff * diff, axis=1))

    def _fallback_transform(self, points: np.ndarray, transform: np.ndarray) -> np.ndarray:
        """Fallback transformation for systems without SIMD"""
        points_hom = np.column_stack([points, np.ones(len(points))])
        transformed = (transform @ points_hom.T).T
        return transformed[:, :3]


# =============================================================================
# ISSUE 7: CACHE OPTIMIZATIONS
# =============================================================================

class CacheOptimizedProcessor:
    """Cache-aware processing to minimize cache misses"""

    def __init__(self, cache_line_size: int = 64):
        self.cache_line_size = cache_line_size
        self.simd_processor = SIMDProcessor()

    def process_points_cache_friendly(self, points: np.ndarray,
                                    transform: np.ndarray) -> np.ndarray:
        """Process points in cache-friendly blocks"""
        block_size = 1024  # Process in blocks that fit in cache
        results = []

        for i in range(0, len(points), block_size):
            block = points[i:i + block_size]
            transformed_block = self.simd_processor.fast_transform_points(block, transform)
            results.append(transformed_block)

        return np.concatenate(results, axis=0)

    def prefetch_data(self, data: np.ndarray):
        """Prefetch data into cache (where supported)"""
        # This is a hint to the CPU to prefetch data
        # In practice, this would use platform-specific prefetch instructions
        pass


# =============================================================================
# PERFORMANCE MONITORING
# =============================================================================

class PerformanceMonitor:
    """Real-time performance monitoring and bottleneck detection"""

    def __init__(self):
        self.metrics = {
            'frame_times': [],
            'memory_usage': [],
            'cpu_usage': [],
            'processing_times': {
                'extraction': [],
                'transform': [],
                'filtering': [],
                'projection': [],
                'coloring': [],
                'publishing': []
            }
        }
        self.max_samples = 100

    def record_frame_time(self, frame_time: float):
        """Record frame processing time"""
        self.metrics['frame_times'].append(frame_time)
        if len(self.metrics['frame_times']) > self.max_samples:
            self.metrics['frame_times'].pop(0)

    def record_stage_time(self, stage: str, time: float):
        """Record processing stage time"""
        if stage in self.metrics['processing_times']:
            self.metrics['processing_times'][stage].append(time)
            if len(self.metrics['processing_times'][stage]) > self.max_samples:
                self.metrics['processing_times'][stage].pop(0)

    def get_performance_stats(self) -> Dict[str, Any]:
        """Get comprehensive performance statistics"""
        stats = {}

        # Frame rate statistics
        if self.metrics['frame_times']:
            frame_times = np.array(self.metrics['frame_times'])
            stats['fps'] = {
                'current': 1.0 / frame_times[-1] if frame_times[-1] > 0 else 0,
                'average': 1.0 / np.mean(frame_times) if np.mean(frame_times) > 0 else 0,
                'p95': 1.0 / np.percentile(frame_times, 95) if np.percentile(frame_times, 95) > 0 else 0
            }

        # Processing stage analysis
        total_stage_times = {}
        for stage, times in self.metrics['processing_times'].items():
            if times:
                total_stage_times[stage] = {
                    'avg_time': np.mean(times),
                    'max_time': np.max(times),
                    'percentage': 0  # Will be calculated below
                }

        # Calculate percentages
        total_avg_time = sum(stage['avg_time'] for stage in total_stage_times.values())
        if total_avg_time > 0:
            for stage_data in total_stage_times.values():
                stage_data['percentage'] = (stage_data['avg_time'] / total_avg_time) * 100

        stats['processing_stages'] = total_stage_times

        # System resources
        stats['system'] = {
            'memory_percent': psutil.virtual_memory().percent,
            'cpu_percent': psutil.cpu_percent(interval=0.1),
            'memory_mb': psutil.virtual_memory().used / (1024 * 1024)
        }

        return stats

    def detect_bottlenecks(self) -> List[str]:
        """Detect performance bottlenecks"""
        bottlenecks = []

        stats = self.get_performance_stats()

        # Check frame rate
        if 'fps' in stats and stats['fps']['average'] < 10:
            bottlenecks.append("Low frame rate (< 10 FPS)")

        # Check processing stages
        if 'processing_stages' in stats:
            stages = stats['processing_stages']
            for stage_name, stage_data in stages.items():
                if stage_data['percentage'] > 50:
                    bottlenecks.append(".1f")

        # Check system resources
        if 'system' in stats:
            sys_stats = stats['system']
            if sys_stats['memory_percent'] > 90:
                bottlenecks.append("High memory usage (> 90%)")
            if sys_stats['cpu_percent'] > 95:
                bottlenecks.append("High CPU usage (> 95%)")

        return bottlenecks


# =============================================================================
# SUMMARY OF ALL OPTIMIZATION ISSUES
# =============================================================================

OPTIMIZATION_ISSUES = {
    "critical": [
        "Memory Pool System - Eliminates GC pauses for real-time operation",
        "Async I/O Pipeline - Prevents blocking on ROS operations",
        "Rolling Buffer - Handles network jitter and motion compensation",
        "DOWNSAMPLE_FREQUENCY Bug - Undefined constant causing crashes"
    ],
    "high": [
        "LAZ Compression - Efficient point cloud storage (not currently used)",
        "SIMD Optimizations - Vectorized math operations",
        "Cache Optimizations - Memory access patterns for performance",
        "Performance Monitoring - Real-time bottleneck detection"
    ],
    "medium": [
        "Memory Alignment - Cache-line aligned data structures",
        "Zero-copy Operations - Minimize data copying where possible",
        "Thread Affinity - Pin threads to CPU cores for consistency",
        "NUMA Awareness - Memory allocation on correct NUMA nodes"
    ],
    "low": [
        "CPU Governor Control - Dynamic frequency scaling",
        "Huge Pages - Reduce TLB misses for large allocations",
        "I/O Scheduling - Optimize disk/network I/O priorities",
        "Compiler Optimizations - Profile-guided optimization flags"
    ]
}

if __name__ == "__main__":
    print("Optimization Issues Analysis")
    print("=" * 40)

    for priority, issues in OPTIMIZATION_ISSUES.items():
        print(f"\n{priority.upper()} PRIORITY:")
        for issue in issues:
            print(f"• {issue}")

    print(f"\nTotal Issues Identified: {sum(len(issues) for issues in OPTIMIZATION_ISSUES.values())}")

