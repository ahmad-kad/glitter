#!/usr/bin/env python3
"""LiDAR-camera fusion utilities."""

import numpy as np
import logging
import time
from typing import Optional, Dict, Any, List, Tuple, Union
from pathlib import Path
import numbers

# Constants
LIDAR_MIN_DEPTH = 0.1
LIDAR_MAX_DEPTH = 50.0
DOWNSAMPLE_STEP = 5
IMAGE_BOUNDS_EPSILON = 1e-8
CALIBRATION_TRACKBAR_RANGE = 100
CALIBRATION_TRACKBAR_OFFSET = 50
CALIBRATION_ANGLE_OFFSET = 180
CALIBRATION_DEPTH_THRESHOLD = 2.0

# L2 LiDAR specific constants
L2_POINT_CLOUD_FIELDS = ('x', 'y', 'z', 'intensity', 'tag', 'line')
L2_DEFAULT_TOPIC = '/livox/lidar'
L2_DEFAULT_FRAME = 'livox_frame'

# Pi Camera v3 intrinsics (1280x720)
PI_CAMERA_V3_WIDTH = 1280
PI_CAMERA_V3_HEIGHT = 720
PI_CAMERA_V3_FX_FACTOR = 0.85
PI_CAMERA_V3_FY_FACTOR = 0.85

try:
    import ros_numpy
    HAS_ROS_NUMPY = True
except ImportError:
    HAS_ROS_NUMPY = False

try:
    from tqdm import tqdm
    HAS_TQDM = True
except ImportError:
    HAS_TQDM = False


class LoggingUtils:
    """Utilities for logging and performance tracking."""

    @staticmethod
    def setup_logging(log_file: Union[str, Path] = "glitter.log",
                     level: int = logging.INFO) -> logging.Logger:
        """Setup structured logging with file and console output.

        Args:
            log_file: Path to log file
            level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)

        Returns:
            Configured logger instance

        Raises:
            ValueError: If log_file path is invalid
            OSError: If log file cannot be created
        """
        if not isinstance(log_file, (str, Path)):
            raise TypeError("log_file must be a string or Path object")

        if not isinstance(level, int) or level not in [logging.DEBUG, logging.INFO,
                                                      logging.WARNING, logging.ERROR, logging.CRITICAL]:
            raise ValueError("Invalid logging level")

        logger = logging.getLogger("glitter")
        logger.setLevel(level)

        # Remove existing handlers to avoid duplicates
        for handler in logger.handlers[:]:
            logger.removeHandler(handler)

        # Create formatters
        file_formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s'
        )
        console_formatter = logging.Formatter(
            '%(asctime)s - %(levelname)s - %(message)s'
        )

        try:
            # File handler
            file_handler = logging.FileHandler(log_file)
            file_handler.setLevel(level)
            file_handler.setFormatter(file_formatter)
            logger.addHandler(file_handler)
        except (OSError, IOError) as e:
            raise OSError(f"Cannot create log file '{log_file}': {e}")

        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.WARNING)  # Only show warnings/errors on console
        console_handler.setFormatter(console_formatter)
        logger.addHandler(console_handler)

        return logger

    @staticmethod
    def time_function(func_name: str, logger: Optional[logging.Logger] = None):
        """Decorator to time function execution."""
        def decorator(func):
            def wrapper(*args, **kwargs):
                start_time = time.time()
                try:
                    result = func(*args, **kwargs)
                    elapsed = time.time() - start_time
                    if logger:
                        logger.debug(f"{func_name} completed in {elapsed:.3f}s")
                    return result
                except Exception as e:
                    elapsed = time.time() - start_time
                    if logger:
                        logger.error(f"{func_name} failed after {elapsed:.3f}s: {e}")
                    raise
            return wrapper
        return decorator

    @staticmethod
    def log_performance_stats(stats: Dict[str, Any], logger: logging.Logger):
        """Log performance statistics."""
        logger.info("Performance Stats:")
        for key, value in stats.items():
            if isinstance(value, float):
                logger.info(f"  {key}: {value:.3f}")
            else:
                logger.info(f"  {key}: {value}")

    @staticmethod
    def track_system_resources(logger: logging.Logger):
        """Track and log system resource usage."""
        try:
            import psutil
            memory = psutil.virtual_memory()
            cpu = psutil.cpu_percent(interval=0.1)

            logger.info("System Resources:")
            logger.info(f"  Memory: {memory.percent:.1f}% used ({memory.used/1024/1024:.0f}MB/{memory.total/1024/1024:.0f}MB)")
            logger.info(f"  CPU: {cpu:.1f}%")
            logger.info(f"  Available Memory: {memory.available/1024/1024:.0f}MB")
        except ImportError:
            logger.debug("psutil not available for resource tracking")

    @staticmethod
    def log_frame_stats(logger: logging.Logger, frame_count: int, stats: Dict[str, any], interval: int = 30):
        """Log frame statistics at specified intervals."""
        if frame_count % interval == 0:
            stats_str = ", ".join(f"{k}: {v}" for k, v in stats.items())
            logger.info(f"Frame {frame_count}: {stats_str}")

    @staticmethod
    def create_progress_bar(iterable, desc="Processing", unit="items", disable=not HAS_TQDM):
        """Create a progress bar if tqdm is available."""
        if HAS_TQDM:
            return tqdm(iterable, desc=desc, unit=unit, ncols=80)
        else:
            # Return the iterable as-is if tqdm is not available
            return iterable

    @staticmethod
    def detect_lidar_type():
        """Detect LiDAR type based on available ROS topics."""
        try:
            # This would be called from ROS node context
            # For now, return default L2 topic
            return L2_DEFAULT_TOPIC
        except Exception:
            # Fallback to L1 topic
            return '/unilidar/cloud'


class TransformUtils:
    """Utilities for 3D transformations and projections."""

    @staticmethod
    def euler_to_rotation_matrix(roll: Union[float, np.floating],
                                pitch: Union[float, np.floating],
                                yaw: Union[float, np.floating]) -> np.ndarray:
        """Convert Euler angles (roll, pitch, yaw) to rotation matrix.

        Args:
            roll: Rotation around X-axis (radians)
            pitch: Rotation around Y-axis (radians)
            yaw: Rotation around Z-axis (radians)

        Returns:
            3x3 rotation matrix as numpy array

        Raises:
            TypeError: If inputs are not numeric
        """
        # Input validation
        for name, value in [("roll", roll), ("pitch", pitch), ("yaw", yaw)]:
            if not isinstance(value, (numbers.Real, np.floating)):
                raise TypeError(f"{name} must be a real number, got {type(value)}")

        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        matrix = np.array([
            [cp * cy, -cp * sy, sp],
            [sr * sp * cy + cr * sy, -sr * sp * sy + cr * cy, -sr * cp],
            [-cr * sp * cy + sr * sy, cr * sp * sy + sr * cy, cr * cp]
        ], dtype=np.float32)

        return matrix

    @staticmethod
    def build_extrinsic_matrix(translation: Union[List[float], np.ndarray],
                              rotation: Union[List[float], np.ndarray]) -> np.ndarray:
        """Build 4x4 extrinsic transformation matrix.

        Args:
            translation: [x, y, z] translation vector (meters)
            rotation: [roll, pitch, yaw] rotation angles (radians)

        Returns:
            4x4 transformation matrix as numpy array

        Raises:
            ValueError: If translation/rotation don't have correct dimensions
            TypeError: If inputs are not numeric sequences
        """
        # Input validation
        if not isinstance(translation, (list, np.ndarray)) or len(translation) != 3:
            raise ValueError("translation must be a list/array of 3 numeric values")

        if not isinstance(rotation, (list, np.ndarray)) or len(rotation) != 3:
            raise ValueError("rotation must be a list/array of 3 numeric values")

        # Convert to numpy arrays and validate numeric types
        trans_array = np.asarray(translation, dtype=np.float64)
        rot_array = np.asarray(rotation, dtype=np.float64)

        if not np.all(np.isfinite(trans_array)):
            raise ValueError("translation contains non-finite values")

        if not np.all(np.isfinite(rot_array)):
            raise ValueError("rotation contains non-finite values")

        R = TransformUtils.euler_to_rotation_matrix(*rot_array)
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = R.astype(np.float32)
        T[:3, 3] = trans_array.astype(np.float32)

        return T

    @staticmethod
    def project_3d_to_2d(points_3d: np.ndarray, K: np.ndarray,
                        extrinsic: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Project 3D points to 2D image coordinates.

        Args:
            points_3d: Nx3 array of 3D points in world/camera coordinates
            K: 3x3 camera intrinsic matrix
            extrinsic: 4x4 extrinsic transformation matrix (world to camera)

        Returns:
            Tuple of (u, v) pixel coordinates as integer arrays

        Raises:
            ValueError: If input arrays have incorrect shapes or contain invalid data
            TypeError: If inputs are not numpy arrays
        """
        # Input validation
        if not isinstance(points_3d, np.ndarray) or points_3d.ndim != 2 or points_3d.shape[1] != 3:
            raise ValueError("points_3d must be an Nx3 numpy array")

        if not isinstance(K, np.ndarray) or K.shape != (3, 3):
            raise ValueError("K must be a 3x3 numpy array")

        if not isinstance(extrinsic, np.ndarray) or extrinsic.shape != (4, 4):
            raise ValueError("extrinsic must be a 4x4 numpy array")

        if not np.all(np.isfinite(points_3d)):
            raise ValueError("points_3d contains non-finite values")

        if not np.all(np.isfinite(K)):
            raise ValueError("K matrix contains non-finite values")

        if not np.all(np.isfinite(extrinsic)):
            raise ValueError("extrinsic matrix contains non-finite values")

        # Optimized homogeneous coordinate transformation
        points_hom = PointCloudUtils.to_homogeneous(points_3d)
        points_cam = points_hom @ extrinsic.T

        # Vectorized projection to image plane
        uv_hom = points_cam[:, :3] @ K.T

        # Convert to pixel coordinates with proper division (avoid division by zero)
        z_coords = uv_hom[:, 2]
        valid_z = np.abs(z_coords) > 1e-8  # Slightly more tolerant threshold
        u = np.zeros(len(z_coords), dtype=np.int32)
        v = np.zeros(len(z_coords), dtype=np.int32)
        u[valid_z] = (uv_hom[valid_z, 0] / z_coords[valid_z]).astype(np.int32)
        v[valid_z] = (uv_hom[valid_z, 1] / z_coords[valid_z]).astype(np.int32)

        return u, v


class ColorUtils:
    """Utilities for color packing and unpacking in point clouds."""

    @staticmethod
    def pack_rgb(colors_bgr: np.ndarray) -> np.ndarray:
        """Pack BGR colors into float32 for PointCloud2 RGB field.

        Args:
            colors_bgr: Nx3 array of BGR colors (uint8 values 0-255)

        Returns:
            Array of float32 values with packed RGB

        Raises:
            ValueError: If input array has incorrect shape or values out of range
            TypeError: If input is not a numpy array
        """
        if not isinstance(colors_bgr, np.ndarray):
            raise TypeError("colors_bgr must be a numpy array")

        if colors_bgr.ndim != 2 or colors_bgr.shape[1] != 3:
            raise ValueError("colors_bgr must be an Nx3 array")

        if colors_bgr.dtype != np.uint8:
            raise TypeError("colors_bgr must be uint8 type")

        if np.any(colors_bgr < 0) or np.any(colors_bgr > 255):
            raise ValueError("colors_bgr values must be in range 0-255")

        # Vectorized RGB packing: B | (G << 8) | (R << 16)
        rgb_int = (colors_bgr[:, 0].astype(np.uint32) |
                  (colors_bgr[:, 1].astype(np.uint32) << 8) |
                  (colors_bgr[:, 2].astype(np.uint32) << 16))
        return rgb_int.view(np.float32)

    @staticmethod
    def unpack_rgb(rgb_packed: np.ndarray) -> np.ndarray:
        """Unpack float32 RGB field to separate R, G, B channels.

        Args:
            rgb_packed: Array of packed RGB float32 values

        Returns:
            Nx3 array of RGB colors (uint8 values 0-255)

        Raises:
            TypeError: If input is not a numpy array
            ValueError: If input array is empty
        """
        if not isinstance(rgb_packed, np.ndarray):
            raise TypeError("rgb_packed must be a numpy array")

        if rgb_packed.size == 0:
            raise ValueError("rgb_packed array cannot be empty")

        rgb_int = rgb_packed.view(np.uint32)
        # Vectorized unpacking: Extract B, G, R in packed order and return as BGR
        rgb_array = np.column_stack([
            rgb_int & 0xFF,           # B (least significant byte)
            (rgb_int >> 8) & 0xFF,    # G
            (rgb_int >> 16) & 0xFF    # R (most significant byte)
        ]).astype(np.uint8)

        return rgb_array


class PointCloudUtils:
    """Utilities for point cloud processing and filtering."""

    @staticmethod
    def extract_xyz_rgb(msg, field_names: Tuple[str, ...] = L2_POINT_CLOUD_FIELDS):
        """Extract XYZ and RGB data from PointCloud2 message.

        Args:
            msg: PointCloud2 ROS message
            field_names: Tuple of field names to extract

        Returns:
            Tuple of (xyz_array, rgb_array) as numpy arrays

        Raises:
            ValueError: If required fields are missing or message is malformed
            ImportError: If neither ros_numpy nor sensor_msgs_py is available
        """
        if not field_names or not all(isinstance(f, str) for f in field_names):
            raise ValueError("field_names must be a non-empty tuple of strings")

        if 'x' not in field_names or 'y' not in field_names or 'z' not in field_names:
            raise ValueError("field_names must include 'x', 'y', and 'z'")

        if HAS_ROS_NUMPY:
            try:
                pc_array = ros_numpy.numpify(msg)
                xyz = np.column_stack((pc_array['x'], pc_array['y'], pc_array['z']))

                # Check if RGB data is requested
                rgb_requested = 'rgb' in field_names or any(f in ['intensity', 'tag', 'line'] for f in field_names)

                if rgb_requested:
                    # Handle both L1 (rgb) and L2 (intensity) fields
                    if 'rgb' in pc_array.dtype.names:
                        rgb_packed = pc_array['rgb']
                    elif 'intensity' in pc_array.dtype.names:
                        # Convert intensity to RGB for visualization
                        intensity = pc_array['intensity'].astype(np.uint8)
                        rgb_packed = (intensity.astype(np.uint32) |
                                    (intensity.astype(np.uint32) << 8) |
                                    (intensity.astype(np.uint32) << 16)).astype(np.float32)
                    else:
                        rgb_packed = np.zeros(len(xyz), dtype=np.float32)
                else:
                    rgb_packed = np.zeros(len(xyz), dtype=np.float32)

                return xyz.astype(np.float32), rgb_packed
            except Exception as e:
                raise ValueError(f"Failed to extract data with ros_numpy: {e}")
        else:
            try:
                from sensor_msgs_py import point_cloud2
                points_gen = point_cloud2.read_points(msg, field_names=field_names, skip_nans=True)
                points_np = np.array(list(points_gen))
                if len(points_np) == 0:
                    return np.empty((0, 3), dtype=np.float32), np.empty(0, dtype=np.float32)
                xyz = points_np[:, :3].astype(np.float32)
                # Handle RGB or intensity field
                rgb_idx = None
                for i, field in enumerate(field_names[:points_np.shape[1]]):
                    if field == 'rgb':
                        rgb_idx = i
                        break
                    elif field == 'intensity' and rgb_idx is None:
                        rgb_idx = i
                if rgb_idx is not None and rgb_idx < points_np.shape[1]:
                    rgb_data = points_np[:, rgb_idx].astype(np.float32)
                    if field_names[rgb_idx] == 'intensity':
                        # Convert intensity to RGB
                        intensity = rgb_data.astype(np.uint8)
                        rgb_data = (intensity.astype(np.uint32) |
                                  (intensity.astype(np.uint32) << 8) |
                                  (intensity.astype(np.uint32) << 16)).astype(np.float32)
                    rgb_packed = rgb_data
                else:
                    rgb_packed = np.zeros(len(points_np), dtype=np.float32)
                return xyz, rgb_packed
            except ImportError:
                raise ImportError("Neither ros_numpy nor sensor_msgs_py is available")
            except Exception as e:
                raise ValueError(f"Failed to extract data with sensor_msgs_py: {e}")

    @staticmethod
    def filter_points_in_front(points: np.ndarray, min_depth: float = 0.1) -> np.ndarray:
        """Filter points that are in front of the camera.

        Args:
            points: Nx3 array of 3D points
            min_depth: Minimum depth threshold (meters)

        Returns:
            Filtered points array

        Raises:
            ValueError: If input array has incorrect shape
            TypeError: If inputs are not valid types
        """
        if not isinstance(points, np.ndarray) or points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points must be an Nx3 numpy array")

        if not isinstance(min_depth, (int, float)) or min_depth < 0:
            raise ValueError("min_depth must be a non-negative number")

        return points[points[:, 2] > min_depth]

    @staticmethod
    def filter_points_in_image_bounds(u: np.ndarray, v: np.ndarray,
                                    width: int, height: int) -> Tuple[np.ndarray, np.ndarray]:
        """Filter points that fall within image boundaries.

        Args:
            u: U coordinates array
            v: V coordinates array
            width: Image width in pixels
            height: Image height in pixels

        Returns:
            Tuple of filtered (u, v) coordinate arrays

        Raises:
            ValueError: If coordinate arrays have different lengths or invalid dimensions
            TypeError: If inputs have incorrect types
        """
        if not isinstance(u, np.ndarray) or not isinstance(v, np.ndarray):
            raise TypeError("u and v must be numpy arrays")

        if len(u) != len(v):
            raise ValueError("u and v arrays must have the same length")

        if not isinstance(width, int) or not isinstance(height, int) or width <= 0 or height <= 0:
            raise ValueError("width and height must be positive integers")

        # Vectorized bounds checking
        valid = ((u >= 0) & (u < width) & (v >= 0) & (v < height))
        return u[valid], v[valid]

    @staticmethod
    def downsample_points(points: np.ndarray, step: int = 5) -> np.ndarray:
        """Downsample points by taking every Nth point.

        Args:
            points: Input point cloud array
            step: Downsampling factor (must be >= 1)

        Returns:
            Downsampled point cloud array

        Raises:
            ValueError: If step is invalid or points array is malformed
            TypeError: If inputs have incorrect types
        """
        if not isinstance(points, np.ndarray):
            raise TypeError("points must be a numpy array")

        if not isinstance(step, int) or step < 1:
            raise ValueError("step must be a positive integer")

        if points.size == 0:
            return points.copy()

        return points[::step]

    @staticmethod
    def to_homogeneous(points: np.ndarray) -> np.ndarray:
        """Convert points to homogeneous coordinates.

        Args:
            points: Nx3 array of 3D points

        Returns:
            Nx4 array of homogeneous coordinates

        Raises:
            ValueError: If input array has incorrect shape
            TypeError: If input is not a numpy array
        """
        if not isinstance(points, np.ndarray):
            raise TypeError("points must be a numpy array")

        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points must be an Nx3 array")

        if points.shape[0] == 0:
            return np.empty((0, 4), dtype=points.dtype)

        return np.hstack((points, np.ones((points.shape[0], 1), dtype=points.dtype)))


class CameraUtils:
    """Utilities for camera intrinsic parameters."""

    @staticmethod
    def create_intrinsic_matrix(fx: Union[float, int], fy: Union[float, int],
                               cx: Union[float, int], cy: Union[float, int]) -> np.ndarray:
        """Create camera intrinsic matrix.

        Args:
            fx: Focal length in x direction (pixels)
            fy: Focal length in y direction (pixels)
            cx: Principal point x coordinate (pixels)
            cy: Principal point y coordinate (pixels)

        Returns:
            3x3 camera intrinsic matrix

        Raises:
            ValueError: If any parameter is non-positive or non-finite
            TypeError: If parameters are not numeric
        """
        params = [fx, fy, cx, cy]
        param_names = ['fx', 'fy', 'cx', 'cy']

        for name, value in zip(param_names, params):
            if not isinstance(value, (int, float)):
                raise TypeError(f"{name} must be a number")
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be a positive finite number")

        return np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float32)

    @staticmethod
    def default_pi_camera_matrix(width: int = PI_CAMERA_V3_WIDTH, height: int = PI_CAMERA_V3_HEIGHT) -> np.ndarray:
        """Create default intrinsic matrix for Pi Camera v3.

        Args:
            width: Image width in pixels
            height: Image height in pixels

        Returns:
            3x3 camera intrinsic matrix with typical Pi Camera v3 parameters

        Raises:
            ValueError: If width or height are invalid
        """
        if not isinstance(width, int) or width <= 0:
            raise ValueError("width must be a positive integer")

        if not isinstance(height, int) or height <= 0:
            raise ValueError("height must be a positive integer")

        # Pi Camera v3 focal length factors
        fx = width * PI_CAMERA_V3_FX_FACTOR
        fy = height * PI_CAMERA_V3_FY_FACTOR
        cx = width / 2.0
        cy = height / 2.0

        return CameraUtils.create_intrinsic_matrix(fx, fy, cx, cy)
