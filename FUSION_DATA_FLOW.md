# Fusion Data Flow Documentation

## Overview

The fusion node colors LiDAR point clouds using camera images. Here's how the data flows:

## Data Flow Steps

### 1. **Get Point Cloud from LiDAR**
- **Topic**: `/unilidar/cloud` (PointCloud2 message)
- **Frequency**: ~12 Hz
- **Data**: XYZ coordinates of 3D points
- **Extraction**: Uses `sensor_msgs_py.point_cloud2.read_points()` to extract (x, y, z) tuples

### 2. **Get Image Frame from Camera**
- **Topic**: `/camera/camera/color/image_raw` (Image message)
- **Frequency**: ~30 Hz
- **Data**: BGR color image
- **Conversion**: Uses `cv_bridge` to convert ROS Image to OpenCV format

### 3. **Calculate Camera Parameters (Bootstrap)**
- **Intrinsic Matrix (K)**: 3x3 camera calibration matrix
  - Default: Pi Camera v3 intrinsics (from `CameraUtils.default_pi_camera_matrix()`)
  - Can be set via ROS parameter `camera_matrix`
- **Extrinsic Matrix (RT)**: 4x4 transformation from LiDAR to camera coordinates
  - Translation: `extrinsic_trans` parameter [x, y, z] (default: [0, 0, 0])
  - Rotation: `extrinsic_rot` parameter [roll, pitch, yaw] (default: [0, 0, 0])
  - Built using `TransformUtils.build_extrinsic_matrix(t, r)`

### 4. **Transform Points to Camera Coordinates**
```python
points_hom = PointCloudUtils.to_homogeneous(points_xyz)  # Add w=1
points_cam = (RT @ points_hom.T).T  # Transform to camera frame
```

### 5. **Filter Points in Front of Camera**
```python
depth_mask = points_cam[:, 2] > 0.1  # z > 0.1m (in front)
points_cam_valid = points_cam[depth_mask]
```

### 6. **Project 3D Points to 2D Image Coordinates**
```python
u, v = TransformUtils.project_3d_to_2d(points_cam_valid[:, :3], K, np.eye(4))
```
- Projects 3D camera coordinates to 2D pixel coordinates (u, v)
- Uses camera intrinsic matrix K

### 7. **Filter Points Within Image Bounds**
```python
bounds_mask = (u >= 0) & (u < width) & (v >= 0) & (v < height)
u_valid = u[bounds_mask]
v_valid = v[bounds_mask]
```

### 8. **Sample Colors from Image**
```python
colors_bgr = cv_image[v_valid, u_valid]  # OpenCV uses (row, col) = (v, u)
rgb_packed = ColorUtils.pack_rgb(colors_bgr)  # Pack BGR to uint32 RGB
```

### 9. **Create Colored Point Cloud**
```python
output_data = np.column_stack((points_final, rgb_packed))  # [x, y, z, rgb]
colored_msg = point_cloud2.create_cloud(header, FIELDS, output_data)
```

### 10. **Publish Colored Point Cloud**
- **Topic**: `/unilidar/colored_cloud` (PointCloud2 message)
- Contains: XYZ coordinates + RGB colors from camera

## Time Synchronization

The fusion uses `message_filters.ApproximateTimeSynchronizer` to ensure:
- LiDAR and camera messages are synchronized (within 1.0s tolerance)
- Both messages are processed together
- **All topics must be in the same ROS domain** (ROS_DOMAIN_ID)

## ROS Domain Configuration

To ensure all topics are in the same "island":
```bash
export ROS_DOMAIN_ID=0  # Set to same value for all nodes
```

## Current Issues

1. **Point Cloud Extraction**: Needs to handle structured arrays from `sensor_msgs_py`
2. **Time Synchronization**: Messages may have timestamps too far apart
3. **Projection Function**: Was incomplete (now fixed)

## Bootstrap Values

Default camera parameters (if not calibrated):
- **Intrinsic K**: Pi Camera v3 default matrix
- **Extrinsic RT**: Identity matrix (camera at same position as LiDAR)
- These can be calibrated using `src/core/calibration.py`

