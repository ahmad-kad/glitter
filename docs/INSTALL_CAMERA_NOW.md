# Install Camera from Raspberry Pi Fork

## What Changed

The installation now builds picamera2 **from source** using the official Raspberry Pi repository instead of PyPI. This ensures you get:
- Latest code from Raspberry Pi
- All fixes and features
- Proper compatibility with your hardware

## Installation

Run this command (you'll need to enter your sudo password):

```bash
cd ~/glitter
./install_picamera2.sh
```

## What the Script Does

1. **Installs system dependencies** (libcap-dev, python3-libcamera, etc.)
2. **Installs Python dependencies** (v4l2-python3, pyopengl, etc.)
3. **Clones Raspberry Pi picamera2 repository** from GitHub
4. **Builds and installs from source** using `pip3 install .`
5. **Tests the camera** to verify it works

## About python3-kms++

**Note**: `python3-kms++` is not available in Ubuntu 24.04 repositories. The script will:
- Try to install it
- If it fails, continue anyway (it's optional)
- picamera2 will work without it for basic camera functionality

`python3-kms++` is used for advanced display features but is **not required** for basic camera capture.

## Build Process

The script:
- Clones: `https://github.com/raspberrypi/picamera2.git`
- Builds in: `/tmp/picamera2_build`
- Installs using: `pip3 install . --break-system-packages`

## After Installation

Start the camera:

```bash
./start_camera.sh
```

Or manually:
```bash
source /opt/ros/jazzy/setup.bash
source activate_env.sh
python3 src/camera/pi_camera_node.py
```

## Verify

In another terminal:
```bash
ros2 topic hz /camera/image_raw
# Should show ~30 Hz
```

## Why Build from Source?

- **Latest features**: Get code directly from Raspberry Pi
- **Better compatibility**: Built specifically for your system
- **Fixes**: Includes latest bug fixes not yet in PyPI
- **Control**: You can modify the code if needed

## Troubleshooting

### Package Not Found Errors

If you see "unable to locate package" for optional packages like `python3-kms++`:
- This is **normal** on Ubuntu 24.04
- The script will continue and install what it can
- picamera2 will work without optional packages

### Build Fails

If build fails:
1. Check internet connection (needs to clone from GitHub)
2. Ensure all dependencies installed: `sudo apt install -y libcap-dev python3-dev python3-libcamera build-essential git`
3. Check git is installed: `git --version`
4. Try manual build:
   ```bash
   cd /tmp
   git clone https://github.com/raspberrypi/picamera2.git
   cd picamera2
   pip3 install . --break-system-packages
   ```

### Camera Not Detected

If camera test fails:
1. Check camera is connected via ribbon cable
2. Try rebooting: `sudo reboot`
3. Check camera is enabled: `vcgencmd get_camera` (if available)
4. Verify with: `libcamera-hello --list-cameras`










