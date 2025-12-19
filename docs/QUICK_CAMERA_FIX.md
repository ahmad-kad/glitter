# Quick Fix: Camera "no cameras available"

## The Problem
Camera is connected via ribbon cable, but `camera_ros` says "no cameras available".

## The Solution: Use picamera2

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
print("✓ Camera working!")
cam.stop()
EOF
```

### Step 3: Run Camera Node

```bash
cd ~/glitter
source /opt/ros/jazzy/setup.bash
source activate_env.sh
python3 src/camera/pi_camera_node.py
```

### Step 4: Verify It's Working

In another terminal:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /camera/image_raw
# Should show ~30 Hz

ros2 run rqt_image_view rqt_image_view /camera/image_raw
# Should show camera image
```

## Why This Works

- `picamera2` directly accesses Pi Camera hardware
- Doesn't need libcamera detection (which is failing)
- Works with ribbon cable connection
- Publishes to `/camera/image_raw` (same as fusion expects)

## For Fusion System

After installing picamera2, the launch script will automatically use it:

```bash
./launch_fusion.sh picamera
```

Or launch manually:
- Terminal 1: `python3 src/camera/pi_camera_node.py`
- Terminal 2: Fusion (already running, will receive images)

That's it! The camera should work now.










