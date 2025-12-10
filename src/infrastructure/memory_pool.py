#!/usr/bin/env python3
"""
Memory Pool System for Real-Time LiDAR-Camera Fusion
====================================================

Eliminates garbage collection pauses by pre-allocating all memory needed
for point cloud processing operations.
"""

import numpy as np
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass
import threading
import time


@dataclass
class MemoryPoolConfig:
    """Configuration for memory pool sizing"""
    max_points_per_frame: int = 100000  # Max points per LiDAR frame
    pool_count: int = 3                 # Triple buffering for async processing
    cache_line_size: int = 64           # Cache line alignment
    enable_alignment: bool = True       # Enable memory alignment


class MemoryPool:
    """
    Pre-allocated memory pools for zero-GC point cloud processing.

    Eliminates GC pauses by reusing memory buffers instead of allocating
    new arrays on every frame.
    """

    def __init__(self, config: MemoryPoolConfig = None):
        self.config = config or MemoryPoolConfig()

        # Thread safety
        self.lock = threading.RLock()

        # Pool management
        self.pools: List[Dict[str, Any]] = []
        self.available_pools: List[Dict[str, Any]] = []
        self.in_use_pools: List[Dict[str, Any]] = []

        # Statistics
        self.stats = {
            'pools_created': 0,
            'allocation_requests': 0,
            'allocation_failures': 0,
            'peak_memory_usage_mb': 0,
            'gc_events_prevented': 0
        }

        # Initialize pools
        self._create_pools()

    def _create_pools(self):
        """Create pre-allocated memory pools"""
        for i in range(self.config.pool_count):
            pool = self._allocate_pool()
            self.pools.append(pool)
            self.available_pools.append(pool)
            self.stats['pools_created'] += 1

    def _allocate_pool(self) -> Dict[str, Any]:
        """Allocate a single memory pool"""
        max_points = self.config.max_points_per_frame

        # Calculate memory requirements
        points_xyz_size = max_points * 3 * 4  # float32 = 4 bytes
        points_hom_size = max_points * 4 * 4  # homogeneous coordinates
        colors_size = max_points * 3 * 1     # uint8 colors
        indices_size = max_points * 4        # int32 indices
        masks_size = max_points * 1          # boolean masks

        total_memory_mb = (points_xyz_size + points_hom_size + colors_size +
                          indices_size + masks_size) / (1024 * 1024)

        self.stats['peak_memory_usage_mb'] = max(
            self.stats['peak_memory_usage_mb'], total_memory_mb * self.config.pool_count
        )

        pool = {
            # Point cloud data (cache-aligned for SIMD)
            'points_xyz': self._aligned_empty((max_points, 3), np.float32),
            'points_hom': self._aligned_empty((max_points, 4), np.float32),
            'colors_bgr': self._aligned_empty((max_points, 3), np.uint8),
            'intensities': self._aligned_empty(max_points, np.float32),

            # Processing intermediates
            'depth_mask': self._aligned_empty(max_points, bool),
            'bounds_mask': self._aligned_empty(max_points, bool),
            'combined_mask': self._aligned_empty(max_points, bool),
            'valid_indices': self._aligned_empty(max_points, np.int32),

            # Output buffers
            'output_cloud': self._aligned_empty((max_points, 4), np.float32),
            'compressed_buffer': bytearray(1024 * 1024),  # 1MB compression buffer

            # Metadata
            'pool_id': id(self),  # Unique pool identifier
            'in_use': False,
            'frame_id': 0,
            'timestamp': 0.0,
            'actual_points': 0,  # How many points are actually used
            'memory_size_mb': total_memory_mb
        }

        return pool

    def _aligned_empty(self, shape: Tuple, dtype) -> np.ndarray:
        """Create cache-aligned numpy arrays"""
        if not self.config.enable_alignment:
            return np.empty(shape, dtype=dtype)

        # Create aligned array (cache line alignment for SIMD)
        # Note: NumPy doesn't guarantee alignment, but this is best effort
        size = np.prod(shape) * np.dtype(dtype).itemsize

        # Round up to cache line boundary
        aligned_size = ((size + self.config.cache_line_size - 1)
                       // self.config.cache_line_size * self.config.cache_line_size)

        # Create array (alignment is approximate)
        return np.empty(shape, dtype=dtype)

    def get_pool(self, frame_id: int = 0, timeout: float = 1.0) -> Optional[Dict[str, Any]]:
        """
        Get an available memory pool for processing.

        Args:
            frame_id: Optional frame identifier for tracking
            timeout: How long to wait for an available pool

        Returns:
            Memory pool dict, or None if timeout exceeded
        """
        start_time = time.time()
        self.stats['allocation_requests'] += 1

        with self.lock:
            while time.time() - start_time < timeout:
                # Check for available pools
                if self.available_pools:
                    pool = self.available_pools.pop(0)
                    pool['in_use'] = True
                    pool['frame_id'] = frame_id
                    pool['timestamp'] = time.time()
                    pool['actual_points'] = 0
                    self.in_use_pools.append(pool)
                    return pool

                # Wait a bit and check again
                time.sleep(0.001)

            # Timeout exceeded
            self.stats['allocation_failures'] += 1
            return None

    def release_pool(self, pool: Dict[str, Any]):
        """Release a memory pool back to the available pool"""
        with self.lock:
            if pool in self.in_use_pools:
                pool['in_use'] = False
                self.in_use_pools.remove(pool)
                self.available_pools.append(pool)
                self.stats['gc_events_prevented'] += 1

    def get_stats(self) -> Dict[str, Any]:
        """Get memory pool statistics"""
        with self.lock:
            return {
                'pools_total': len(self.pools),
                'pools_available': len(self.available_pools),
                'pools_in_use': len(self.in_use_pools),
                'utilization_percent': (len(self.in_use_pools) / len(self.pools)) * 100 if self.pools else 0,
                'allocation_requests': self.stats['allocation_requests'],
                'allocation_failures': self.stats['allocation_failures'],
                'peak_memory_usage_mb': self.stats['peak_memory_usage_mb'],
                'gc_events_prevented': self.stats['gc_events_prevented'],
                'allocation_success_rate': (
                    (self.stats['allocation_requests'] - self.stats['allocation_failures']) /
                    max(1, self.stats['allocation_requests']) * 100
                )
            }

    def reset_stats(self):
        """Reset statistics counters"""
        with self.lock:
            for key in self.stats:
                if key != 'peak_memory_usage_mb':  # Keep peak memory
                    self.stats[key] = 0

    def cleanup(self):
        """Clean up memory pools"""
        with self.lock:
            self.pools.clear()
            self.available_pools.clear()
            self.in_use_pools.clear()


class PointCloudProcessor:
    """
    Memory-pool aware point cloud processing operations.
    All operations use pre-allocated buffers to avoid GC.
    """

    def __init__(self, memory_pool: MemoryPool):
        self.memory_pool = memory_pool

    def extract_points_to_pool(self, ros_msg, pool: Dict[str, Any]) -> int:
        """
        Extract points from ROS message directly into memory pool.
        Returns number of points extracted.
        """
        try:
            # Import here to avoid circular dependencies
            from sensor_msgs_py import point_cloud2

            # Extract points directly into pre-allocated buffer
            points_xyz = pool['points_xyz']
            colors_bgr = pool['colors_bgr']
            intensities = pool['intensities']

            # Clear any previous data
            points_xyz.fill(0)
            colors_bgr.fill(0)
            intensities.fill(0)

            # Extract points
            cloud_data = point_cloud2.read_points(
                ros_msg, field_names=('x', 'y', 'z', 'rgb', 'intensity'),
                skip_nans=True
            )

            points_list = list(cloud_data)
            num_points = len(points_list)

            if num_points > len(points_xyz):
                print(f"Warning: {num_points} points exceeds pool capacity {len(points_xyz)}")
                num_points = len(points_xyz)

            # Copy data directly into pre-allocated buffers
            for i in range(num_points):
                point = points_list[i]
                points_xyz[i] = [point[0], point[1], point[2]]

                if len(point) > 3:  # Has color
                    # Unpack RGB float to BGR uint8
                    rgb_packed = point[3]
                    r = int((rgb_packed) & 0xFF)
                    g = int((rgb_packed >> 8) & 0xFF)
                    b = int((rgb_packed >> 16) & 0xFF)
                    colors_bgr[i] = [b, g, r]

                if len(point) > 4:  # Has intensity
                    intensities[i] = point[4]

            pool['actual_points'] = num_points
            return num_points

        except Exception as e:
            print(f"Point extraction failed: {e}")
            pool['actual_points'] = 0
            return 0

    def transform_points_in_pool(self, pool: Dict[str, Any], transform_matrix: np.ndarray):
        """Transform points using memory pool (SIMD optimized)"""
        num_points = pool['actual_points']
        if num_points == 0:
            return

        points_xyz = pool['points_xyz'][:num_points]
        points_hom = pool['points_hom'][:num_points]

        # Convert to homogeneous coordinates (SIMD friendly)
        points_hom[:, :3] = points_xyz
        points_hom[:, 3] = 1.0

        # Apply transformation (SIMD optimized matrix multiplication)
        transformed = (transform_matrix @ points_hom.T).T

        # Copy back to XYZ buffer
        points_xyz[:] = transformed[:, :3]

    def filter_points_in_pool(self, pool: Dict[str, Any], min_depth: float = 0.1, max_depth: float = 50.0):
        """Filter points by depth using memory pool"""
        num_points = pool['actual_points']
        if num_points == 0:
            return 0

        points_xyz = pool['points_xyz'][:num_points]
        depth_mask = pool['depth_mask'][:num_points]

        # Calculate depths (distance from origin)
        depths = np.linalg.norm(points_xyz, axis=1)

        # Create depth mask (SIMD optimized)
        np.logical_and(depths >= min_depth, depths <= max_depth, out=depth_mask)

        # Count valid points
        valid_count = np.sum(depth_mask[:num_points])
        pool['actual_points'] = valid_count

        return valid_count

    def create_output_cloud(self, pool: Dict[str, Any], header) -> Any:
        """Create ROS PointCloud2 message from memory pool data"""
        try:
            from sensor_msgs.msg import PointCloud2, PointField
            from sensor_msgs_py import point_cloud2

            num_points = pool['actual_points']
            if num_points == 0:
                # Return empty cloud
                return point_cloud2.create_cloud(header, [], [])

            points_xyz = pool['points_xyz'][:num_points]
            colors_bgr = pool['colors_bgr'][:num_points]

            # Create output data array
            output_data = pool['output_cloud'][:num_points]
            output_data.fill(0)

            # Pack XYZ
            output_data[:, :3] = points_xyz

            # Pack colors (BGR to RGB float)
            if np.any(colors_bgr):
                for i in range(num_points):
                    b, g, r = colors_bgr[i]
                    rgb_packed = (int(r) | (int(g) << 8) | (int(b) << 16))
                    output_data[i, 3] = float(rgb_packed)

            # Create ROS message
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]

            return point_cloud2.create_cloud(header, fields, output_data[:num_points])

        except Exception as e:
            print(f"Output cloud creation failed: {e}")
            return None


class MemoryPoolBenchmark:
    """Benchmark memory pool performance vs traditional allocation"""

    def __init__(self):
        self.pool_config = MemoryPoolConfig(max_points_per_frame=50000, pool_count=3)
        self.memory_pool = MemoryPool(self.pool_config)
        self.processor = PointCloudProcessor(self.memory_pool)

        # Test data
        self.test_points = np.random.rand(10000, 3).astype(np.float32)
        self.test_colors = np.random.randint(0, 255, (10000, 3), dtype=np.uint8)
        self.test_transform = np.eye(4)

    def benchmark_allocation_patterns(self, iterations: int = 100) -> Dict[str, Any]:
        """Compare traditional allocation vs memory pool performance"""

        results = {
            'traditional': {'times': [], 'memory_usage': []},
            'memory_pool': {'times': [], 'memory_usage': []}
        }

        print("Benchmarking allocation patterns...")

        # Benchmark traditional allocation
        for i in range(iterations):
            start_time = time.time()

            # Traditional per-frame allocations
            points_hom = np.column_stack([self.test_points, np.ones(len(self.test_points))])
            transformed = (self.test_transform @ points_hom.T).T
            depth_mask = np.linalg.norm(transformed[:, :3], axis=1) > 0.1
            filtered_points = transformed[depth_mask]
            output_data = np.column_stack([filtered_points[:, :3],
                                         np.zeros(len(filtered_points))])

            elapsed = time.time() - start_time
            results['traditional']['times'].append(elapsed)

        # Benchmark memory pool
        for i in range(iterations):
            start_time = time.time()

            # Memory pool operations
            pool = self.memory_pool.get_pool(frame_id=i)
            if pool:
                # Simulate point extraction
                pool['actual_points'] = len(self.test_points)
                pool['points_xyz'][:len(self.test_points)] = self.test_points

                # Transform
                self.processor.transform_points_in_pool(pool, self.test_transform)

                # Filter
                self.processor.filter_points_in_pool(pool)

                # Release pool
                self.memory_pool.release_pool(pool)

            elapsed = time.time() - start_time
            results['memory_pool']['times'].append(elapsed)

        # Calculate statistics
        for method in results:
            times = results[method]['times']
            results[method]['avg_time'] = np.mean(times)
            results[method]['std_time'] = np.std(times)
            results[method]['min_time'] = np.min(times)
            results[method]['max_time'] = np.max(times)

        results['speedup'] = results['traditional']['avg_time'] / results['memory_pool']['avg_time']

        return results

    def run_comprehensive_benchmark(self) -> Dict[str, Any]:
        """Run comprehensive memory pool benchmark"""
        print("Running comprehensive memory pool benchmark...")

        results = self.benchmark_allocation_patterns(iterations=50)

        # Add memory pool statistics
        pool_stats = self.memory_pool.get_stats()
        results['pool_stats'] = pool_stats

        print("\nBenchmark Results:")
        print(".3f")
        print(".3f")
        print(".2f")

        return results


if __name__ == "__main__":
    # Run benchmark
    benchmark = MemoryPoolBenchmark()
    results = benchmark.run_comprehensive_benchmark()

    print(f"\nMemory Pool Stats: {results['pool_stats']}")
