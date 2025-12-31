# RealSense 435 + Unitree L2 ROS2 Setup Guide

This guide provides complete setup instructions for running RealSense 435 camera and Unitree L2 LiDAR together in ROS2 Jazzy, with visualization in RViz.

## Overview

- **RealSense 435**: RGB-D camera providing color images, depth maps, and point clouds
- **Unitree L2**: 4D LiDAR providing high-resolution 3D point clouds
- **Integration**: Both sensors publish to ROS2 topics with proper TF alignment for RViz visualization

## Prerequisites

- Ubuntu 24.04 LTS
- ROS2 Jazzy Jalisco
- Unitree L2 LiDAR connected via Ethernet
- RealSense 435 connected via USB
- Python 3.10+

## Quick Start

### 1. Install RealSense SDK and ROS2 Package

```bash
# Install RealSense SDK (requires sudo)
sudo ./scripts/install_realsense.sh

# Configure RealSense camera
./scripts/configure_realsense.sh
```

### 2. Setup TF Coordinate Frames

```bash
# Create TF transform configurations
./scripts/setup_tf_frames.sh
```

### 3. Test the Setup

```bash
# Run integration tests
./scripts/test_realsense_l2_integration.sh
```

### 4. Launch Combined System

**Option A: Manual Step-by-Step (Recommended for first time)**
```bash
./scripts/launch_realsense_l2_manual.sh
```

**Option B: Automated Launch**
```bash
./scripts/launch_realsense_l2.sh
```

## Expected Output

When running correctly, you should see:

### ROS2 Topics
```
✓ /camera/color/image_raw          # RealSense RGB image
✓ /camera/depth/image_rect_raw     # RealSense depth image
✓ /camera/depth/color/points       # RealSense aligned point cloud
✓ /unilidar/cloud                  # L2 LiDAR point cloud
✓ /tf                              # Coordinate frame transforms
```

### RViz Visualization
- **L2_LiDAR_PointCloud**: White points from LiDAR (intensity colored)
- **RealSense_PointCloud**: RGB colored points from camera
- **TF_Frames**: Coordinate frame axes showing sensor alignment
- **Grid**: Reference ground plane

## Configuration Details

### RealSense 435 Settings
- **Resolution**: 640x480 @ 30fps (color and depth)
- **Point Cloud**: Aligned to color frame with RGB texture
- **Frame ID**: `camera_link`

### Unitree L2 Settings
- **Point Cloud**: `/unilidar/cloud` topic
- **Frame Rate**: ~10-20 Hz depending on configuration
- **Frame ID**: `unilidar_lidar`

### TF Transform (Default)
```
unilidar_lidar → camera_link
- X offset: 0.0m (forward/backward)
- Y offset: 0.0m (left/right)
- Z offset: 0.1m (camera 10cm above LiDAR)
- Rotation: 0° (aligned orientations)
```

## Calibration

If the point clouds don't align properly in RViz:

### 1. Measure Physical Setup
Determine the actual distance and angle between the RealSense camera and L2 LiDAR in your physical setup.

### 2. Update Transform Parameters
Edit `config/tf/static_transforms.launch.py` and modify the offset values:

```python
# Example: Camera is 20cm forward, 5cm right, 15cm above LiDAR, rotated 3°
x_offset='0.2',    # 20cm forward
y_offset='0.05',   # 5cm right
z_offset='0.15',   # 15cm above
yaw_offset='3.0'   # 3° rotation
```

### 3. Test Alignment
```bash
# Launch with custom transforms
ros2 launch config/tf/static_transforms.launch.py \
  x_offset:=0.2 \
  y_offset:=0.05 \
  z_offset:=0.15 \
  yaw_offset:=3.0
```

## Troubleshooting

### RealSense Issues

**Camera not detected:**
```bash
# Check USB connection
lsusb | grep "8086:0b07"

# Test with realsense-viewer
realsense-viewer
```

**No ROS2 topics:**
```bash
# Check driver installation
ros2 pkg list | grep realsense

# Manual launch test
ros2 launch realsense2_camera rs_launch.py
```

### L2 LiDAR Issues

**Cannot connect:**
```bash
# Check network
ping 192.168.1.150

# Check Ethernet interface
ip addr show eth0
```

**No point cloud data:**
```bash
# Check existing L2 setup
./scripts/utilities/check_lidar_data.sh

# Verify driver
ros2 launch unitree_lidar_ros2 launch.py
```

### RViz Issues

**Point clouds not visible:**
1. Check **Fixed Frame** is set to `unilidar_lidar`
2. Enable point cloud displays in **Displays** panel
3. Increase **Size (Pixels)** to 5-10 for better visibility
4. Verify topics are publishing: `ros2 topic list`

**Misaligned data:**
1. Check TF transforms are publishing: `ros2 run tf2_tools view_frames.py`
2. Verify transform parameters in `static_transforms.launch.py`
3. Use manual calibration with known reference points

### Performance Issues

**High CPU usage:**
- Reduce resolution: `color_width:=320, color_height:=240`
- Disable unused streams: `enable_infra1:=false, enable_infra2:=false`
- Increase queue sizes and decrease frequencies

**Synchronization issues:**
- Enable sync: `enable_sync:=true`
- Check topic frequencies: `ros2 topic hz /camera/depth/color/points`

## Advanced Configuration

### Custom RealSense Settings

Edit `config/realsense/realsense_launch.py` to modify:

- **Resolution**: Change `color_width`, `color_height`
- **Frame Rate**: Modify `color_fps`, `depth_fps`
- **Filters**: Enable spatial/temporal filters for better depth
- **Alignment**: Set `align_depth:=false` for raw depth

### Multiple Cameras

For multiple RealSense cameras:
```bash
# Launch with specific serial number
ros2 launch realsense2_camera rs_launch.py serial_no:=1234567890
```

### Data Recording

Record sensor data for later analysis:
```bash
# Record all topics
ros2 bag record /camera/* /unilidar/* /tf /tf_static

# Playback
ros2 bag play recording/
```

## File Structure

```
config/
├── realsense/
│   ├── realsense_launch.py     # RealSense launch configuration
│   └── realsense_params.yaml   # RealSense parameters
├── tf/
│   ├── static_transforms.launch.py  # TF transforms
│   ├── calibrate_transforms.py      # Calibration helper
│   └── README.md                    # TF documentation
└── realsense_l2_combined.rviz       # RViz configuration

scripts/
├── install_realsense.sh             # SDK installation
├── configure_realsense.sh           # Camera configuration
├── setup_tf_frames.sh               # TF setup
├── launch_realsense_l2.sh           # Automated launch
├── launch_realsense_l2_manual.sh    # Manual launch
└── test_realsense_l2_integration.sh # Integration tests
```

## Performance Expectations

- **Frame Rate**: 20-30 FPS combined
- **Memory Usage**: 800MB-1.5GB
- **CPU Usage**: 60-80% (depends on resolution)
- **Network**: L2 LiDAR ~50-100Mbps, Camera ~100-200Mbps

## Next Steps

1. ✅ **Basic Setup**: Both sensors publishing data
2. ✅ **TF Alignment**: Coordinate frames properly aligned
3. ✅ **RViz Visualization**: Both point clouds visible together
4. → **Sensor Fusion**: Combine data for better perception
5. → **Calibration**: Fine-tune extrinsic parameters
6. → **Applications**: SLAM, object detection, mapping

## Support

For issues:
1. Run the integration test: `./scripts/test_realsense_l2_integration.sh`
2. Check ROS2 logs: `ros2 doctor`
3. Verify hardware connections
4. Review TF tree: `ros2 run tf2_tools view_frames.py`

The setup is designed to be robust and work out-of-the-box with default parameters, but may require calibration for your specific physical configuration.
