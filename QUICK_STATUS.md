# Quick Sensor Status Check

## Current Status

Based on the test results:

### ✅ LiDAR Driver: INSTALLED
- Package: `unitree_lidar_ros2` is installed
- Launch file: Found at `/home/durian/ros2_ws/install/unitree_lidar_ros2/share/unitree_lidar_ros2/launch.py`
- **Action needed**: Start the driver

### ❌ LiDAR: NOT RUNNING
- No ROS topics found (`/unilidar/cloud`, `/unilidar/imu`)
- **To start**: `ros2 launch unitree_lidar_ros2 launch.py`

### ❌ RealSense Camera: NOT RUNNING
- No ROS topics found
- Direct device access blocked (likely in use or not started)
- **To start**: Launch RealSense ROS node

## Quick Start Commands

### Start LiDAR:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

### Start RealSense Camera:
(Depends on your RealSense setup - check if you have a RealSense ROS package)

### Verify Both Are Running:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep -E "unilidar|camera"
```

### Check Data Flow:
```bash
# LiDAR frequency
ros2 topic hz /unilidar/cloud

# Camera frequency (if running)
ros2 topic hz /camera/camera/color/image_raw
```

## Test Script

Run the comprehensive test:
```bash
cd ~/glitter/glitter
source venv/bin/activate
python3 test_realsense_lidar.py
```

