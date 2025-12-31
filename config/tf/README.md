# TF Coordinate Frame Setup

This directory contains the coordinate frame transformations for aligning RealSense camera and Unitree L2 LiDAR data.

## Coordinate Frames

- `map` / `odom`: World reference frame (optional)
- `unilidar_lidar`: Unitree L2 LiDAR base frame
- `camera_link`: RealSense camera base frame
- `camera_color_frame`: RealSense color camera optical frame
- `camera_depth_frame`: RealSense depth camera optical frame

## Transform Parameters

The static transform from L2 LiDAR to RealSense camera is defined by:

- **Translation** (x, y, z): Physical offset between sensor centers
- **Rotation** (roll, pitch, yaw): Angular offset between sensor orientations

### Default Values
- x_offset: 0.0m (forward/backward)
- y_offset: 0.0m (left/right)
- z_offset: 0.1m (camera 10cm above LiDAR)
- roll/pitch/yaw: 0.0° (aligned orientations)

## Calibration Process

1. **Measure Physical Setup**: Determine real-world distance and angle between sensors
2. **Run Sensors**: Start both L2 LiDAR and RealSense camera
3. **Use Calibration Script**: `ros2 run calibrate_transforms.py` to verify data flow
4. **Visual Inspection**: Check alignment in RViz
5. **Fine-tune**: Adjust transform parameters as needed

## Usage

```bash
# Launch static transforms
ros2 launch config/tf/static_transforms.launch.py

# With custom offsets (example)
ros2 launch config/tf/static_transforms.launch.py \
  x_offset:=0.2 \
  y_offset:=0.0 \
  z_offset:=0.15 \
  yaw_offset:=5.0
```

## Troubleshooting

- **No TF data**: Check that static_transform_publisher is running
- **Misaligned data**: Verify physical measurements and transform parameters
- **Frame not found**: Ensure both sensors are publishing TF data
