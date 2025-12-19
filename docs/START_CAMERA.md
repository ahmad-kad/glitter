# Start Camera - Quick Guide

## Step 1: Install picamera2

Run this command (requires sudo for dependencies):

```bash
cd ~/glitter
./install_picamera2.sh
```

Or manually:
```bash
sudo apt install -y libcap-dev python3-dev python3-libcamera python3-kms++
pip3 install --break-system-packages picamera2
```

## Step 2: Start Camera

```bash
cd ~/glitter
./start_camera.sh
```

Or manually:
```bash
source /opt/ros/jazzy/setup.bash
source activate_env.sh
python3 src/camera/pi_camera_node.py
```

## Step 3: Verify

In another terminal:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /camera/image_raw
# Should show ~30 Hz
```

## That's It!

The camera will publish to `/camera/image_raw` and fusion will automatically use it.










