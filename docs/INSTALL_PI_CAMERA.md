# Pi Camera Setup - Ubuntu 24.04

## Solution: Use ROS 2 Camera Package

Since `libcamera-apps` is not available on Ubuntu 24.04, use the ROS 2 camera package instead:

```bash
sudo apt install ros-jazzy-camera-ros
```

## Test Camera

After installing, test the camera:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run camera_ros camera_ros_node
```

In another terminal, check if it's publishing:
```bash
ros2 topic hz /camera/image_raw
```

## Update Launch Script

The launch script needs to use `camera_ros` instead of `usb_cam`. 

For now, launch camera manually:

**Terminal 1 - Camera:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run camera_ros camera_ros_node
```

**Terminal 2 - Check:**
```bash
ros2 topic hz /camera/image_raw
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

## Alternative: Manual Camera Node

If `camera_ros` doesn't work, I've created a Python node that tries different approaches:

```bash
cd ~/glitter
source /opt/ros/jazzy/setup.bash
source activate_env.sh
python3 src/camera/pi_camera_node.py --ros-args -p video_device:=/dev/video20
```

Try different video devices: `/dev/video20`, `/dev/video21`, etc.

## Troubleshooting

### Check available video devices:
```bash
v4l2-ctl --list-devices
```

### Check camera permissions:
```bash
groups | grep video
# If not in video group:
sudo usermod -a -G video $USER
# Then log out and back in
```

### Test with v4l2 directly:
```bash
v4l2-ctl -d /dev/video20 --all
```
