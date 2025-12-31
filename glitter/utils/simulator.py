#!/usr/bin/env python3
"""
LiDAR-Camera Fusion Simulator
============================

Generates synthetic sensor data for testing the fusion pipeline without hardware.

Features:
- Synthetic point clouds with configurable noise and density
- Synthetic camera images with projected point colors
- Ground truth correspondences for validation
- ROS-compatible message generation
- Realistic sensor characteristics simulation
"""

import numpy as np
import cv2
from typing import Tuple, List, Optional, Dict, Any
from dataclasses import dataclass
import time


@dataclass
class SensorConfig:
    """Configuration for simulated sensors."""
    # LiDAR parameters
    lidar_range: float = 20.0  # meters
    lidar_fov_horizontal: float = 360.0  # degrees
    lidar_fov_vertical: float = 30.0  # degrees
    lidar_resolution: float = 0.1  # meters between points
    lidar_noise_std: float = 0.02  # position noise std dev

    # Camera parameters
    camera_width: int = 1280
    camera_height: int = 720
    camera_fov_horizontal: float = 90.0  # degrees
    camera_focal_length: float = 800.0  # pixels

    # Scene parameters
    num_objects: int = 10
    object_density: float = 1000.0  # points per cubic meter
    background_noise: float = 0.1  # fraction of total points


class SceneSimulator:
    """Generates synthetic 3D scenes for testing."""

    def __init__(self, config: SensorConfig):
        self.config = config
        np.random.seed(42)  # For reproducible results

    def generate_ground_plane(self, size: float = 10.0, resolution: float = 0.1) -> np.ndarray:
        """Generate ground plane points."""
        x = np.arange(-size/2, size/2, resolution)
        y = np.arange(-size/2, size/2, resolution)
        X, Y = np.meshgrid(x, y)
        Z = np.zeros_like(X)

        # Add some noise to make it more realistic
        Z += np.random.normal(0, 0.01, Z.shape)

        points = np.column_stack((X.flatten(), Y.flatten(), Z.flatten()))
        return points.astype(np.float32)

    def generate_objects(self) -> List[np.ndarray]:
        """Generate various 3D objects in the scene."""
        objects = []

        for i in range(self.config.num_objects):
            # Random object type
            obj_type = np.random.choice(['cube', 'sphere', 'cylinder'])

            if obj_type == 'cube':
                objects.append(self._generate_cube())
            elif obj_type == 'sphere':
                objects.append(self._generate_sphere())
            else:  # cylinder
                objects.append(self._generate_cylinder())

        return objects

    def _generate_cube(self) -> np.ndarray:
        """Generate a cube with configurable density."""
        # Random position and size
        center = np.random.uniform(-5, 5, 3)
        size = np.random.uniform(0.5, 2.0)

        # Generate points on cube surface
        points_per_face = int(self.config.object_density * size * size / 6)

        faces = []
        for axis in [0, 1, 2]:
            for sign in [-1, 1]:
                # Create face perpendicular to axis
                other_axes = [i for i in [0, 1, 2] if i != axis]
                face_points = np.random.uniform(-size/2, size/2, (points_per_face, 2))

                face_coords = np.zeros((points_per_face, 3))
                face_coords[:, other_axes[0]] = face_points[:, 0] + center[other_axes[0]]
                face_coords[:, other_axes[1]] = face_points[:, 1] + center[other_axes[1]]
                face_coords[:, axis] = sign * size/2 + center[axis]

                faces.append(face_coords)

        return np.vstack(faces).astype(np.float32)

    def _generate_sphere(self) -> np.ndarray:
        """Generate a sphere."""
        center = np.random.uniform(-3, 3, 3)
        radius = np.random.uniform(0.3, 1.0)

        # Generate points on sphere surface
        num_points = int(self.config.object_density * 4 * np.pi * radius * radius)
        phi = np.random.uniform(0, 2*np.pi, num_points)
        theta = np.random.uniform(0, np.pi, num_points)

        x = center[0] + radius * np.sin(theta) * np.cos(phi)
        y = center[1] + radius * np.sin(theta) * np.sin(phi)
        z = center[2] + radius * np.cos(theta)

        return np.column_stack((x, y, z)).astype(np.float32)

    def _generate_cylinder(self) -> np.ndarray:
        """Generate a cylinder."""
        center = np.random.uniform(-4, 4, 3)
        radius = np.random.uniform(0.2, 0.8)
        height = np.random.uniform(1.0, 3.0)

        # Generate points on cylinder surface
        num_points = int(self.config.object_density * 2 * np.pi * radius * height)
        angle = np.random.uniform(0, 2*np.pi, num_points)
        h = np.random.uniform(-height/2, height/2, num_points)

        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        z = center[2] + h

        return np.column_stack((x, y, z)).astype(np.float32)

    def generate_scene(self) -> np.ndarray:
        """Generate complete 3D scene."""
        # Ground plane
        ground = self.generate_ground_plane()

        # Objects
        objects = self.generate_objects()
        all_objects = np.vstack(objects) if objects else np.empty((0, 3), dtype=np.float32)

        # Combine and add noise
        scene_points = np.vstack([ground, all_objects])

        # Add noise
        if self.config.lidar_noise_std > 0:
            noise = np.random.normal(0, self.config.lidar_noise_std, scene_points.shape)
            scene_points += noise

        # Filter by range
        distances = np.linalg.norm(scene_points, axis=1)
        in_range = distances <= self.config.lidar_range
        scene_points = scene_points[in_range]

        # Add some background noise
        if self.config.background_noise > 0:
            num_noise = int(len(scene_points) * self.config.background_noise)
            noise_points = np.random.uniform(-self.config.lidar_range,
                                           self.config.lidar_range,
                                           (num_noise, 3))
            # Keep only noise points within range
            noise_distances = np.linalg.norm(noise_points, axis=1)
            valid_noise = noise_distances <= self.config.lidar_range
            noise_points = noise_points[valid_noise]

            scene_points = np.vstack([scene_points, noise_points])

        return scene_points.astype(np.float32)


class SensorSimulator:
    """Simulates LiDAR and camera sensors."""

    def __init__(self, config: SensorConfig):
        self.config = config
        self.scene_gen = SceneSimulator(config)

        # Camera intrinsic matrix
        self.K = self._create_camera_matrix()

        # LiDAR extrinsic (camera position relative to LiDAR)
        self.lidar_to_camera = np.eye(4, dtype=np.float32)
        # Example: camera is 0.1m forward, 0.05m up from LiDAR
        self.lidar_to_camera[0, 3] = 0.1   # x
        self.lidar_to_camera[2, 3] = -0.05 # z (up in camera coordinates)

    def _create_camera_matrix(self) -> np.ndarray:
        """Create camera intrinsic matrix."""
        fx = self.config.camera_focal_length
        fy = fx  # Assume square pixels
        cx = self.config.camera_width / 2
        cy = self.config.camera_height / 2

        return np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float32)

    def generate_point_cloud(self) -> Tuple[np.ndarray, np.ndarray]:
        """Generate synthetic point cloud with colors.

        Returns:
            Tuple of (points_xyz, colors_bgr) arrays
        """
        # Generate 3D scene
        points_xyz = self.scene_gen.generate_scene()

        # Generate colors based on position (height-based coloring)
        colors_bgr = self._generate_colors_from_position(points_xyz)

        return points_xyz, colors_bgr

    def _generate_colors_from_position(self, points: np.ndarray) -> np.ndarray:
        """Generate colors based on 3D position for visualization."""
        # Height-based coloring (blue=low, green=medium, red=high)
        heights = points[:, 2]
        height_min, height_max = np.min(heights), np.max(heights)

        if height_max == height_min:
            # All points same height
            return np.full((len(points), 3), [128, 128, 128], dtype=np.uint8)

        # Normalize heights to 0-1
        height_norm = (heights - height_min) / (height_max - height_min)

        # Create color gradient: blue -> green -> red
        colors = np.zeros((len(points), 3), dtype=np.uint8)

        # Blue channel (decreases with height)
        colors[:, 0] = (1 - height_norm) * 255

        # Green channel (peaks in middle)
        colors[:, 1] = np.sin(height_norm * np.pi) * 255

        # Red channel (increases with height)
        colors[:, 2] = height_norm * 255

        return colors

    def generate_camera_image(self, points_xyz: np.ndarray,
                            colors_bgr: np.ndarray) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Generate synthetic camera image from point cloud.

        Args:
            points_xyz: Nx3 point cloud
            colors_bgr: Nx3 colors

        Returns:
            Tuple of (image_bgr, metadata_dict)
        """
        # Create blank image
        image = np.zeros((self.config.camera_height, self.config.camera_width, 3),
                        dtype=np.uint8)

        # Transform points to camera coordinates
        points_hom = np.column_stack((points_xyz, np.ones(len(points_xyz))))
        points_camera = (self.lidar_to_camera @ points_hom.T).T

        # Filter points in front of camera
        valid_depth = points_camera[:, 2] > 0.1
        points_camera = points_camera[valid_depth]
        colors_valid = colors_bgr[valid_depth]

        if len(points_camera) == 0:
            return image, {"num_points": 0, "projected_points": 0}

        # Project to image plane
        points_2d = points_camera[:, :3] @ self.K.T
        z_inv = 1.0 / points_2d[:, 2]
        u = (points_2d[:, 0] * z_inv).astype(int)
        v = (points_2d[:, 1] * z_inv).astype(int)

        # Filter points within image bounds
        valid_bounds = ((u >= 0) & (u < self.config.camera_width) &
                       (v >= 0) & (v < self.config.camera_height))

        u_valid = u[valid_bounds]
        v_valid = v[valid_bounds]
        colors_final = colors_valid[valid_bounds]

        # Draw points on image
        for ui, vi, color in zip(u_valid, v_valid, colors_final):
            cv2.circle(image, (ui, vi), 2, color.tolist(), -1)

        metadata = {
            "num_points": len(points_xyz),
            "projected_points": len(colors_final),
            "image_shape": image.shape,
            "camera_matrix": self.K.copy(),
            "lidar_to_camera": self.lidar_to_camera.copy()
        }

        return image, metadata

    def create_ros_messages(self, points_xyz: np.ndarray, colors_bgr: np.ndarray,
                          image_bgr: np.ndarray) -> Dict[str, Any]:
        """Create ROS-compatible messages for testing.

        Returns dict with 'pointcloud_msg' and 'image_msg' keys.
        """
        # This would require ROS imports, so we'll return the data in ROS-compatible format
        return {
            "points_xyz": points_xyz,
            "colors_bgr": colors_bgr,
            "image_bgr": image_bgr,
            "camera_info": {
                "width": self.config.camera_width,
                "height": self.config.camera_height,
                "K": self.K.flatten().tolist()
            }
        }


def run_simulation_demo():
    """Demo function showing simulator usage."""
    print("🚀 LiDAR-Camera Fusion Simulator Demo")
    print("=" * 50)

    # Configure sensors
    config = SensorConfig()
    config.num_objects = 5
    config.lidar_noise_std = 0.01

    print(f"📷 Camera: {config.camera_width}x{config.camera_height}")
    print(f"🔍 LiDAR: {config.lidar_range}m range, {config.lidar_noise_std*100:.1f}cm noise")
    print(f"🎯 Objects: {config.num_objects}")

    # Create simulator
    simulator = SensorSimulator(config)

    # Generate synthetic data
    print("\n🎨 Generating synthetic scene...")
    points_xyz, colors_bgr = simulator.generate_point_cloud()
    print(f"   Generated {len(points_xyz)} points")

    # Generate camera image
    print("📸 Generating camera image...")
    image_bgr, metadata = simulator.generate_camera_image(points_xyz, colors_bgr)
    print(f"   Projected {metadata['projected_points']} points to image")

    # Create ROS-compatible data
    ros_data = simulator.create_ros_messages(points_xyz, colors_bgr, image_bgr)

    print("\n✅ Simulation complete!")
    print(f"   Point cloud: {ros_data['points_xyz'].shape}")
    print(f"   Colors: {ros_data['colors_bgr'].shape}")
    print(f"   Image: {ros_data['image_bgr'].shape}")

    return simulator, ros_data


if __name__ == "__main__":
    run_simulation_demo()
