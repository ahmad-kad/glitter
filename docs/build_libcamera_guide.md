# Building libcamera and rpicam-apps from Source

## Understanding the Architecture

### What Goes Where?

```
Userspace (what we're building):
  - rpicam-apps (libcamera-hello, libcamera-vid, etc.)
  - libcamera (core library)
  - picamera2 (Python bindings)

Kernel Space (already working):
  - V4L2 drivers
  - Camera sensor drivers (imx219, imx477, etc.)
  - ISP drivers (pisp_be, rp1_cfe for Pi 5)
```

**Your kernel modules are already loaded and working!** The issue is that the userspace tools (rpicam-apps) aren't installed, and your libcamera version may not fully support Pi 5 RP1 hardware.

## Why Build from Source?

### Comparison of Approaches

| Method | Installation Time | Complexity | Pi 5 Support | Customization |
|--------|------------------|------------|--------------|---------------|
| `apt install` | 2 minutes | Low | Limited | None |
| **Build from source** | 30-60 minutes | Medium | Full | Complete |

### When to Build from Source:
1. **New hardware** (Pi 5 with RP1 chip - your case!)
2. **Latest features** (new camera sensors, algorithms)
3. **Custom tuning** (ISP parameters, color profiles)
4. **Development** (contributing patches, debugging)

### When apt install is Fine:
1. Raspberry Pi 4 or older
2. Standard camera modules (V2, HQ)
3. Just need basic functionality
4. Production deployment (want stability)

## Build Process Overview

### Time Estimates:
- Dependency installation: 5-10 minutes
- libcamera build: 15-25 minutes (meson + ninja)
- rpicam-apps build: 10-15 minutes
- Testing: 5 minutes

### Disk Space Required:
- Build dependencies: ~500 MB
- libcamera source + build: ~200 MB
- rpicam-apps source + build: ~150 MB
- **Total: ~850 MB**

## Step-by-Step Build Instructions

### Step 1: Install Build Dependencies

```bash
# Update package lists
sudo apt update

# Core build tools
sudo apt install -y \
    git \
    cmake \
    meson \
    ninja-build \
    pkg-config

# libcamera dependencies
sudo apt install -y \
    libboost-dev \
    libgnutls28-dev \
    openssl \
    libtiff5-dev \
    pybind11-dev \
    qtbase5-dev \
    libqt5core5a \
    libqt5gui5 \
    libqt5widgets5 \
    libyaml-dev \
    python3-yaml \
    python3-ply \
    python3-jinja2

# rpicam-apps dependencies
sudo apt install -y \
    libboost-program-options-dev \
    libdrm-dev \
    libexif-dev \
    libjpeg-dev \
    libpng-dev \
    libavcodec-dev \
    libavdevice-dev \
    libavformat-dev \
    libswresample-dev \
    libepoxy-dev

# Optional: For DNG (raw) support
sudo apt install -y libdng-dev
```

**Why each dependency?**
- `meson/ninja`: Modern build system (faster than make)
- `libboost`: C++ utilities
- `libgnutls28-dev`: TLS/SSL for networking
- `libtiff/libjpeg/libpng`: Image formats
- `libdrm-dev`: Direct rendering (display)
- `libavcodec/libavformat`: Video encoding (H.264, etc.)

### Step 2: Build libcamera

```bash
# Navigate to project root
cd /home/ahmad/glitter

# Create build directory
mkdir -p ~/builds
cd ~/builds

# Clone libcamera (official Raspberry Pi fork)
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera

# Configure build
# -Dpipelines=rpi/vc4,rpi/pisp = Support both Pi 4 and Pi 5
# -Dipas=rpi/vc4,rpi/pisp = Image processing algorithms for both
# -Dv4l2=true = V4L2 interface support
# -Dtest=false = Skip tests (faster build)
meson setup build \
    --buildtype=release \
    -Dpipelines=rpi/vc4,rpi/pisp \
    -Dipas=rpi/vc4,rpi/pisp \
    -Dv4l2=true \
    -Dgstreamer=disabled \
    -Dtest=false

# Build (use all CPU cores)
ninja -C build

# Install
sudo ninja -C build install
```

**Build options explained:**
- `--buildtype=release`: Optimized code (vs debug)
- `rpi/pisp`: Pi 5 ISP pipeline (RP1 chip)
- `rpi/vc4`: Pi 4 ISP pipeline (VideoCore)
- `-Dgstreamer=disabled`: We don't need GStreamer (saves time)

**Expected output:**
```
[1/150] Compiling src/libcamera/camera.cpp
[2/150] Compiling src/libcamera/camera_manager.cpp
...
[150/150] Linking libcamera.so
```

### Step 3: Build rpicam-apps

```bash
cd ~/builds

# Clone rpicam-apps
git clone https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps

# Configure build
meson setup build \
    --buildtype=release \
    -Denable_drm=true \
    -Denable_egl=true \
    -Denable_qt=true \
    -Denable_opencv=false \
    -Denable_tflite=false

# Build
ninja -C build

# Install
sudo ninja -C build install
```

**Build options explained:**
- `enable_drm/egl`: Display output (for preview)
- `enable_qt`: Qt-based GUI apps
- `enable_opencv=false`: We don't need OpenCV integration
- `enable_tflite=false`: No TensorFlow Lite (ML models)

**This installs:**
- `libcamera-hello`: Test camera, show preview
- `libcamera-vid`: Record video
- `libcamera-still`: Capture images
- `libcamera-jpeg`: Fast JPEG capture
- `libcamera-raw`: Capture raw Bayer data

### Step 4: Configure Library Paths

```bash
# Update library cache
sudo ldconfig

# Verify libraries are found
ldconfig -p | grep libcamera

# Add to environment (permanent)
echo 'export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

**Why?** libcamera installs to `/usr/local/lib` which may not be in default search path.

### Step 5: Test Installation

```bash
# List available cameras
libcamera-hello --list-cameras

# Expected output:
# Available cameras:
# 0 : imx219 [3280x2464] (/base/axi/pcie@120000/rp1/i2c@80000/imx219@10)
#     Modes: 'SRGGB10_CSI2P' : 640x480 1640x1232 3280x2464
#            'SRGGB8' : 640x480 1640x1232 3280x2464

# Test camera with 5-second preview
libcamera-hello --timeout 5000

# If successful, you'll see camera preview window!
```

### Step 6: Update picamera2

```bash
# Update picamera2 to work with new libcamera
cd /home/ahmad/glitter
source venv/bin/activate

# Install latest picamera2
pip3 install --upgrade picamera2

# Or install from source for bleeding edge:
# cd ~/builds
# git clone https://github.com/raspberrypi/picamera2.git
# cd picamera2
# python3 -m pip install -e .
```

## Verification Checklist

### ✓ Kernel Modules (Already Working)
```bash
lsmod | grep -E "video|camera"
# Should show: videodev, videobuf2, imx219, rp1_cfe, pisp_be
```

### ✓ Video Devices (Already Present)
```bash
ls /dev/video*
# Should show: /dev/video0 through /dev/video37
```

### ✓ libcamera Library
```bash
ldconfig -p | grep libcamera
# Should show: libcamera.so.0.5 or later
```

### ✓ rpicam-apps Commands
```bash
which libcamera-hello
# Should show: /usr/local/bin/libcamera-hello

libcamera-hello --version
# Should show: libcamera-hello version 1.x.x
```

### ✓ Camera Detection
```bash
libcamera-hello --list-cameras
# Should list your camera(s)
```

### ✓ Python Integration
```bash
source venv/bin/activate
python3 -c "from picamera2 import Picamera2; print(Picamera2().camera_controls)"
# Should print camera control parameters
```

## Troubleshooting

### Build Errors

#### "meson: command not found"
```bash
sudo apt install python3-pip
pip3 install --user meson
```

#### "ninja: build stopped: subcommand failed"
- **Check logs**: Look at compile errors above
- **Out of memory**: Close other applications, or add swap
- **Missing dependency**: Re-run dependency installation

### Runtime Errors

#### "libcamera.so.0: cannot open shared object file"
```bash
sudo ldconfig
export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
```

#### "no cameras available"
- **Check connection**: Camera ribbon cable properly seated?
- **Check detection**: `libcamera-hello --list-cameras`
- **Check permissions**: `groups | grep video` (add with `sudo usermod -a -G video $USER`)
- **Reboot**: Some camera changes need reboot

#### "Failed to import camera library"
```bash
# picamera2 may need reinstall
pip3 uninstall picamera2
pip3 install --no-cache-dir picamera2
```

## Performance Optimization

### Compiler Flags (Advanced)
For maximum performance, rebuild with native optimizations:

```bash
# In libcamera build directory
meson configure build -Dc_args="-O3 -march=native" -Dcpp_args="-O3 -march=native"
ninja -C build
sudo ninja -C build install
```

**Trade-off:** Faster execution, but binary won't work on other ARM CPUs.

### Memory Usage
libcamera uses DMA buffers. To allocate more CMA (contiguous memory):

```bash
# Edit /boot/firmware/config.txt
sudo nano /boot/firmware/config.txt

# Add or modify:
cma=256M  # Default is 64M, increase for high-res/high-fps

# Reboot
sudo reboot
```

## Next Steps

Once built and tested:

1. **Test with your ROS node**: Run `python3 src/camera/pi_camera_node.py`
2. **Check camera enumeration**: Should not see "list index out of range"
3. **Verify image capture**: Should successfully read frames
4. **Run fusion system**: `./launch_fusion.sh`

## Understanding Build Systems

### Why meson instead of make?

| Feature | Make | Meson |
|---------|------|-------|
| Speed | Slower | Faster (parallel by default) |
| Syntax | Complex | Readable (Python-like) |
| Dependencies | Manual | Automatic detection |
| Cross-compile | Hard | Easy |

### Build directory structure:
```
~/builds/libcamera/
  ├── src/              # Source code
  ├── include/          # Headers
  ├── build/            # Build artifacts
  │   ├── src/          # Compiled objects
  │   └── lib/          # Libraries
  └── meson.build       # Build configuration
```

**Clean builds:**
```bash
# Remove build directory and start over
rm -rf build
meson setup build --buildtype=release ...
```

## References

- [libcamera Documentation](https://libcamera.org/)
- [rpicam-apps Repository](https://github.com/raspberrypi/rpicam-apps)
- [Raspberry Pi Camera Software](https://www.raspberrypi.com/documentation/computers/camera_software.html)
- [Meson Build System](https://mesonbuild.com/)

## Key Takeaways

1. **Kernel drivers already work** - no kernel compilation needed
2. **Build userspace tools from source** for Pi 5 support
3. **meson/ninja is modern build system** - faster than autotools/make
4. **Test each step** before moving to next
5. **Library paths matter** - configure with ldconfig



