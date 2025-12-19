# Fix Camera "no cameras available" Error

## Quick Fix

The camera is connected but libcamera can't detect it. Here's how to fix it:

### Step 1: Install picamera2

```bash
pip3 install --break-system-packages picamera2
```

### Step 2: Test Camera

```bash
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

### Step 3: Use Python Camera Node

Instead of `camera_ros`, use the Python node:

```bash
cd ~/glitter
source /opt/ros/jazzy/setup.bash
source activate_env.sh
python3 src/camera/pi_camera_node.py
```

This will publish to `/camera/image_raw` and work with your fusion system.

### Step 4: Update Launch Script

The launch script is already updated. After installing picamera2, run:

```bash
./launch_fusion.sh picamera
```

## Why This Works

- `picamera2` directly accesses the Pi Camera hardware
- Doesn't rely on libcamera detection (which is failing)
- Works with the camera connected via ribbon cable
- Publishes to the same topic (`/camera/image_raw`) that fusion expects

## Alternative: Fix camera_ros

If you want to use `camera_ros` instead, you may need to:
1. Reboot after connecting camera
2. Check camera is enabled in firmware
3. Try different camera device paths

But the Python node with picamera2 is the most reliable solution.










