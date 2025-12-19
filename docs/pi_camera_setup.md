# Raspberry Pi Camera Setup

## Install libcamera-apps

First, install the camera tools:

```bash
sudo apt update
sudo apt install libcamera-apps
```

## Test Camera

Run the test script:

```bash
./scripts/utilities/test_pi_camera.sh
```

This will:
1. Check if libcamera is installed
2. List available cameras
3. Test camera preview
4. Capture a test image

## Use Pi Camera with ROS 2

### Option 1: Use the Pi Camera Node

I've created a ROS 2 node for Pi Camera:

```bash
cd ~/glitter
source /opt/ros/jazzy/setup.bash
source activate_env.sh  # if using venv
python3 src/camera/pi_camera_node.py
```

This publishes to `/camera/image_raw` topic.

### Option 2: Update Launch Script

The launch script needs to be updated to use Pi Camera instead of USB camera.

Edit `scripts/utilities/launch_fusion_complete.sh` and change:

```bash
# Instead of:
ros2 run usb_cam usb_cam_node_exe

# Use:
python3 src/camera/pi_camera_node.py
```

## Verify Camera is Working

1. **Check topic is publishing:**
   ```bash
   ros2 topic hz /camera/image_raw
   # Should show ~15-30 Hz
   ```

2. **View camera image:**
   ```bash
   ros2 run rqt_image_view rqt_image_view /camera/image_raw
   ```

3. **Check image data:**
   ```bash
   ros2 topic echo /camera/image_raw --once | head -20
   ```

## Troubleshooting

### libcamera-hello not found
- Install: `sudo apt install libcamera-apps`
- Verify: `which libcamera-hello`

### No cameras detected
- Check camera is connected to CSI port
- Check ribbon cable is secure
- Try: `libcamera-hello --list-cameras`

### Camera preview doesn't work
- Check if you have a display (SSH won't show preview)
- Try capturing image instead: `libcamera-jpeg -o test.jpg`

### ROS node fails
- Make sure OpenCV is installed: `pip install opencv-python`
- Check Python path: `python3 -c "import cv2; print(cv2.__version__)"`

## Integration with Fusion

Once the Pi Camera node is publishing to `/camera/image_raw`:

1. **Fusion node will automatically subscribe** (it's already waiting)
2. **Colored point clouds will start** once camera images arrive
3. **Check fusion logs** to see if it's receiving camera data:
   ```bash
   tail -f /tmp/fusion.log
   ```

## Quick Start

```bash
# Terminal 1: Install libcamera (if not installed)
sudo apt install libcamera-apps

# Terminal 2: Test camera
./scripts/utilities/test_pi_camera.sh

# Terminal 3: Launch Pi Camera ROS node
cd ~/glitter
source /opt/ros/jazzy/setup.bash
source activate_env.sh
python3 src/camera/pi_camera_node.py

# Terminal 4: Check it's working
ros2 topic hz /camera/image_raw
```










