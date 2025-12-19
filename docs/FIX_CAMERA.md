# Fix: "no cameras available" Error

## Problem
Camera is connected via ribbon cable, but `camera_ros` reports "no cameras available".

## Solution 1: Install picamera2 (Recommended)

The Pi Camera needs `picamera2` Python library to work properly:

```bash
sudo apt update
sudo apt install python3-picamera2 python3-libcamera python3-kms++
```

Then use the Python camera node:

```bash
cd ~/glitter
source /opt/ros/jazzy/setup.bash
source activate_env.sh
python3 src/camera/pi_camera_node.py
```

## Solution 2: Fix camera_ros Configuration

The `camera_ros` node might need specific parameters. Try:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run camera_ros camera_node \
  --ros-args \
  -p camera_name:=camera \
  -p camera_info_url:= \
  -p frame_id:=camera_frame
```

## Solution 3: Check Camera Detection

Verify camera is detected by system:

```bash
# Check if camera is enabled
vcgencmd get_camera 2>/dev/null || echo "vcgencmd not available"

# Check kernel messages
sudo dmesg | grep -i camera | tail -10

# Check video devices
v4l2-ctl --list-devices
```

## Solution 4: Enable Camera in Config (if needed)

If camera is not detected, edit `/boot/firmware/config.txt`:

```bash
sudo nano /boot/firmware/config.txt
```

Ensure these lines exist (they should already be there):
```
camera_auto_detect=1
```

Then reboot:
```bash
sudo reboot
```

## Solution 5: Use Python Node (Works Without libcamera-apps)

I've updated the Python camera node to use `picamera2`. Install it:

```bash
sudo apt install python3-picamera2
```

Then run:
```bash
cd ~/glitter
source /opt/ros/jazzy/setup.bash
source activate_env.sh
python3 src/camera/pi_camera_node.py
```

## Quick Test

After installing picamera2:

```bash
# Test picamera2 directly
python3 << 'EOF'
from picamera2 import Picamera2
cam = Picamera2()
config = cam.create_preview_configuration(main={"size": (640, 480)})
cam.configure(config)
cam.start()
print("Camera working!")
cam.stop()
EOF
```

If this works, the ROS node will work too.

## Update Launch Script

The launch script is already updated to use the Python node as fallback. After installing `picamera2`, run:

```bash
./launch_fusion.sh picamera
```

This will automatically use the Python node which works with picamera2.










