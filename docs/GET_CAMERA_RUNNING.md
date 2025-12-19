# Get Camera Running - Step by Step

## Install picamera2 (Required)

The camera needs `picamera2` to work. Install it:

```bash
cd ~/glitter
./scripts/utilities/install_picamera2_complete.sh
```

This will:
1. Install system dependencies (libcap-dev, etc.)
2. Install picamera2 via pip
3. Test the camera

**Note:** This requires sudo for system packages.

## Alternative: Manual Installation

If the script doesn't work, install manually:

```bash
# Install dependencies
sudo apt update
sudo apt install -y libcap-dev python3-dev python3-libcamera python3-kms++

# Install picamera2
pip3 install --break-system-packages picamera2
```

## Test Camera

After installation, test:

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

## Run Camera Node

Once picamera2 is installed:

```bash
cd ~/glitter
source /opt/ros/jazzy/setup.bash
source activate_env.sh
python3 src/camera/pi_camera_node.py
```

## Verify It's Publishing

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /camera/image_raw
# Should show ~30 Hz

ros2 run rqt_image_view rqt_image_view /camera/image_raw
# Should show camera image
```

## For Complete Fusion System

Once camera is working:

```bash
# Terminal 1: LiDAR (already running)
# Terminal 2: Camera
python3 src/camera/pi_camera_node.py

# Terminal 3: Fusion (already running)
# Terminal 4: RViz (already running)
```

Or use the launch script (after fixing camera_ros or using Python node):

```bash
./launch_fusion.sh picamera
```

## Troubleshooting

### "ModuleNotFoundError: No module named 'picamera2'"
- Install: `pip3 install --break-system-packages picamera2`
- May need: `sudo apt install libcap-dev` first

### "You need to install libcap development headers"
- Install: `sudo apt install libcap-dev`

### Camera still not detected
- Check ribbon cable connection
- Try rebooting: `sudo reboot`
- Check: `dmesg | grep -i camera`










