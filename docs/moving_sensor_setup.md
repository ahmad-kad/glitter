# Moving Sensor Setup - Vehicle-Mounted LiDAR

## Overview

When the LiDAR is mounted on a moving vehicle, you need to:
1. **Track vehicle motion** (using IMU/Odometry)
2. **Update TF transforms** as the vehicle moves
3. **Compensate point clouds** for motion (optional, for better mapping)

## Quick Setup

### Step 1: Launch LiDAR
```bash
cd ~/glitter
./scripts/utilities/launch_lidar_only.sh
```

### Step 2: Launch Moving Sensor Handler
```bash
# In new terminal
cd ~/glitter
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 moving_sensor_handler.py
```

This will:
- Subscribe to `/unilidar/imu` (from LiDAR)
- Subscribe to `/odom` (from vehicle, if available)
- Publish TF: `map → base_link → unilidar_lidar`
- Update transforms as vehicle moves

### Step 3: Launch RViz
```bash
# In another terminal
source /opt/ros/jazzy/setup.bash
rviz2 -d ~/glitter/config/l2_fusion_simple.rviz
```

Set Fixed Frame to `map` to see the vehicle moving.

## How It Works

### TF Chain for Moving Vehicle

```
map (world frame)
  └── base_link (vehicle frame - moves with vehicle)
       └── unilidar_lidar (LiDAR frame - fixed offset from vehicle)
```

### Data Sources

1. **IMU** (`/unilidar/imu`):
   - Provides orientation (roll, pitch, yaw)
   - Provides angular velocity
   - Updates `base_link` orientation

2. **Odometry** (`/odom`) - Optional:
   - Provides position and velocity
   - More accurate than IMU-only
   - If not available, uses IMU for dead reckoning

### Motion Integration

- **With Odometry**: Uses position directly from odometry
- **IMU Only**: Integrates velocity to estimate position (dead reckoning)
- **TF Updates**: Published at 10 Hz, updates as vehicle moves

## Configuration

### LiDAR Mounting Offset

If LiDAR is not at vehicle origin, edit `moving_sensor_handler.py`:

```python
# In publish_transform(), find:
transform_base_lidar.transform.translation.x = 0.0  # Forward (meters)
transform_base_lidar.transform.translation.y = 0.0  # Left (meters)
transform_base_lidar.transform.translation.z = 0.0   # Up (meters)
```

Example: LiDAR 0.5m forward, 0.2m up:
```python
transform_base_lidar.transform.translation.x = 0.5
transform_base_lidar.transform.translation.y = 0.0
transform_base_lidar.transform.translation.z = 0.2
```

### LiDAR Orientation Offset

If LiDAR is rotated relative to vehicle:
```python
# Convert Euler angles to quaternion
from transforms3d.euler import euler2quat
q = euler2quat(roll, pitch, yaw, 'sxyz')
transform_base_lidar.transform.rotation.w = q[0]
transform_base_lidar.transform.rotation.x = q[1]
transform_base_lidar.transform.rotation.y = q[2]
transform_base_lidar.transform.rotation.z = q[3]
```

## Integration with Vehicle Odometry

### If Your Vehicle Publishes Odometry

The handler automatically subscribes to `/odom`. Make sure your vehicle publishes:
- `nav_msgs/msg/Odometry` on `/odom` topic
- Contains: `pose.pose.position`, `pose.pose.orientation`, `twist.twist`

### If No Odometry Available

The handler will use IMU for dead reckoning:
- Uses angular velocity for orientation
- Estimates position from acceleration (less accurate)
- Good for short distances, drifts over time

## Motion Compensation for Mapping

### For Better Room Reconstruction

When the vehicle moves, point clouds from different positions need to be transformed to a common frame (usually `map`).

The accumulator already handles this if TF is correct:
```bash
python3 accumulate_pointcloud.py --output moving_room.pcd
```

Point clouds will be automatically transformed to `map` frame using the TF chain.

## Testing

### Test IMU Data
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Check IMU topic
ros2 topic echo /unilidar/imu --once

# Check IMU frequency
ros2 topic hz /unilidar/imu
```

### Test TF Transforms
```bash
# Check if transforms are updating
ros2 run tf2_ros tf2_echo map base_link

# Move vehicle and watch transform change
# Or rotate LiDAR and watch orientation change
```

### Visualize in RViz

1. **Set Fixed Frame to `map`**:
   - Global Options → Fixed Frame → `map`
   
2. **View Point Cloud**:
   - Displays → Add → PointCloud2
   - Topic: `/unilidar/cloud`
   - Fixed Frame: `map` (will use TF to transform)

3. **Watch Vehicle Move**:
   - As vehicle moves, point cloud moves in `map` frame
   - Accumulated points stay in `map` frame (for mapping)

## Advanced: SLAM Integration

For better accuracy over long distances, integrate with SLAM:

### Option 1: Use SLAM Odometry
- Run SLAM (like `cartographer`, `slam_toolbox`)
- SLAM publishes `/odom` or `/map` -> `/odom` transform
- Moving sensor handler uses SLAM odometry

### Option 2: Use SLAM Map Frame
- SLAM provides `map` -> `odom` transform
- Vehicle odometry provides `odom` -> `base_link`
- Handler provides `base_link` -> `unilidar_lidar`
- Full chain: `map` -> `odom` -> `base_link` -> `unilidar_lidar`

## Troubleshooting

### TF Not Updating?

1. **Check IMU is publishing**:
   ```bash
   ros2 topic hz /unilidar/imu
   ```

2. **Check handler is running**:
   ```bash
   ros2 node list | grep moving_sensor
   ```

3. **Check TF is being published**:
   ```bash
   ros2 topic echo /tf --once
   ```

### Point Cloud Not Moving?

1. **Check Fixed Frame**: Should be `map` (not `unilidar_lidar`)
2. **Check TF chain**: `ros2 run tf2_ros tf2_echo map unilidar_lidar`
3. **Verify transforms updating**: Move vehicle, check TF changes

### Drift Over Time?

- **IMU-only**: Will drift (expected)
- **Solution**: Use odometry or SLAM for better accuracy
- **For mapping**: Use SLAM to correct drift

## Complete Workflow

```bash
# Terminal 1: LiDAR
./launch_lidar_only.sh

# Terminal 2: Moving Sensor Handler
python3 moving_sensor_handler.py

# Terminal 3: Accumulator (for mapping)
python3 accumulate_pointcloud.py --output vehicle_scan.pcd

# Terminal 4: RViz
rviz2 -d ~/glitter/config/l2_fusion_simple.rviz
# Set Fixed Frame = 'map'
```

## Key Points

- ✅ **TF Updates**: Transforms update as vehicle moves
- ✅ **IMU Integration**: Uses LiDAR's built-in IMU
- ✅ **Odometry Support**: Can use vehicle odometry if available
- ✅ **Mapping Ready**: Point clouds accumulate in `map` frame
- ✅ **Motion Compensation**: Automatic via TF transforms

