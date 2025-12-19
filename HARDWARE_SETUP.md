# Hardware Setup Guide for LiDAR-Camera Fusion

## Required Hardware
- **Raspberry Pi 5** (8GB RAM recommended)
- **Unitree L2 4D LiDAR**
- **Camera** (choose one):
  - Raspberry Pi Camera v3 (recommended)
  - Raspberry Pi Global Shutter Camera
  - Raspberry Pi AI Camera
  - USB Camera (fallback)
- **MicroSD Card**: 64GB+ (Class 10/UHS-I)
- **Power Supply**: 27W USB-C PD for Pi 5
- **Ethernet Cable**: For LiDAR connection

## System Requirements
- **OS**: Ubuntu 24.04 LTS Server (64-bit ARM)
- **ROS**: ROS 2 Jazzy Jalisco
- **Python**: 3.10+
- **Memory**: 4GB+ available RAM
- **Storage**: 32GB+ free space

---

## Step 1: Ubuntu 24.04 LTS Installation

### Download and Flash Ubuntu

```bash
# Download Ubuntu 24.04 for Raspberry Pi 5
wget https://cdimage.ubuntu.com/releases/24.04/release/ubuntu-24.04-preinstalled-server-arm64+raspi.img.xz

# Flash to SD card (replace /dev/sdX with your SD card)
xz -dc ubuntu-24.04-preinstalled-server-arm64+raspi.img.xz | sudo dd of=/dev/sdX bs=4M status=progress

# Sync and eject
sudo sync
sudo eject /dev/sdX
```

### First Boot Configuration

```bash
# SSH into Pi (default user: ubuntu, password: ubuntu)
ssh ubuntu@raspberrypi.local

# Change password on first login
# Then update system
sudo apt update && sudo apt full-upgrade -y

# Set hostname
sudo hostnamectl set-hostname glitter-fusion

# Configure timezone
sudo timedatectl set-timezone America/New_York  # Adjust to your location

# Reboot
sudo reboot
```

---

## Step 2: ROS 2 Jazzy Installation

```bash
# Set locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy
sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-dev-tools

# Install additional ROS packages
sudo apt install -y \
  ros-jazzy-sensor-msgs \
  ros-jazzy-sensor-msgs-py \
  ros-jazzy-cv-bridge \
  ros-jazzy-tf2-ros \
  ros-jazzy-message-filters \
  ros-jazzy-diagnostic-msgs \
  ros-jazzy-image-transport \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-usb-cam \
  python3-rosdep \
  python3-colcon-common-extensions

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 3: Camera Driver Installation

### For Raspberry Pi Camera v3

```bash
# Install libcamera and picamera2
sudo apt install -y \
  python3-libcamera \
  python3-picamera2 \
  python3-kms++ \
  libcap-dev \
  libjpeg-dev \
  libtiff-dev

# Test camera
libcamera-hello --list-cameras
# Should show: Available cameras

# Capture test image
libcamera-jpeg -o test.jpg

# Install ROS 2 camera bridge
sudo apt install -y ros-jazzy-image-transport ros-jazzy-image-transport-plugins
pip3 install --break-system-packages rpi-lgpio
```

### For Raspberry Pi Global Shutter Camera

```bash
# Same as Camera v3, plus:
sudo apt install -y python3-numpy python3-opencv

# Verify global shutter mode
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --all
```

### For Raspberry Pi AI Camera

```bash
# AI Camera has onboard ISP/AI processing
sudo apt install -y \
  python3-picamera2 \
  python3-hailo  # Hailo AI accelerator

# Install TensorFlow Lite (for AI features)
pip3 install --break-system-packages \
  tflite-runtime \
  pillow
```

### For USB Camera (Fallback)

```bash
# Install V4L2 tools
sudo apt install -y v4l-utils

# List available cameras
v4l2-ctl --list-devices

# Install ROS 2 USB camera node
sudo apt install -y ros-jazzy-usb-cam

# Test camera capture
ros2 run usb_cam usb_cam_node
```

---

## Step 4: Unitree L2 LiDAR Setup

### Hardware Connection

1. **Connect L2 to Raspberry Pi via Ethernet**
   - Connect Ethernet cable from L2 to Pi Ethernet port
   - L2 uses static IP: `192.168.1.62` or `192.168.1.150` (default, depends on model)

2. **Configure Network**

```bash
# Set static IP on Pi Ethernet interface
sudo nmcli connection add \
  type ethernet \
  con-name "l2-lidar" \
  ifname eth0 \
  ipv4.addresses 192.168.1.100/24 \
  ipv4.method manual

# Restart network
sudo nmcli connection down "Wired connection 1"
sudo nmcli connection up "l2-lidar"

# Test connectivity (try both common IPs)
ping 192.168.1.62
ping 192.168.1.150
```

### Install Unitree L2 Driver

**Option 1: Automated Installation (Recommended)**

```bash
cd ~/glitter
chmod +x scripts/install_unitree_l2.sh
./scripts/install_unitree_l2.sh
```

**Option 2: Manual Installation**

```bash
# Create ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone Unitree L2 ROS 2 driver (check for correct repository)
git clone https://github.com/unitreerobotics/unilidar_sdk_ros2.git
# Or if using unilidar_sdk2:
# git clone https://github.com/unitreerobotics/unilidar_sdk2.git

# Install dependencies
sudo apt update
sudo apt install -y \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    libpcl-dev \
    libyaml-cpp-dev \
    ros-jazzy-rosidl-generator-dds-idl \
    ros-jazzy-rmw-cyclonedds-cpp

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build driver (package name may vary: unitree_lidar_ros2 or unilidar_sdk)
colcon build --packages-select unitree_lidar_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Launch LiDAR and Visualize

**Quick Start:**

```bash
cd ~/glitter
./scripts/utilities/launch_lidar_rviz.sh
```

**Manual Launch:**

**Terminal 1 - LiDAR Driver:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

**Terminal 2 - RViz:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2 -d ~/glitter/config/l2_fusion.rviz
```

### Verify LiDAR is Working

```bash
# Check topics
ros2 topic list
# Should show: /unilidar/cloud (or /livox/lidar depending on driver)

# Check point cloud data
ros2 topic echo /unilidar/cloud --once

# Check data rate
ros2 topic hz /unilidar/cloud
# Expected: ~10-20 Hz depending on LiDAR configuration

# Verify point cloud structure
python3 scripts/utilities/test_pointcloud_viewer.py
```

### RViz Configuration

The RViz config (`config/l2_fusion.rviz`) is configured to show:
- **Raw_L2_LiDAR**: Point cloud from `/unilidar/cloud` with intensity coloring
- **Grid**: Reference grid for visualization
- **TF_Frames**: Coordinate frame visualization

**If Point Cloud Not Visible:**
1. **Enable Display**: **Displays** → **Raw_L2_LiDAR** → **✓ Check Enabled**
2. **Verify Topic**: Should be `/unilidar/cloud`
3. **Set Fixed Frame**: **Global Options** → **Fixed Frame** → `unilidar_lidar`
4. **Increase Point Size**: **Size (Pixels)** → `5` or `10`

See [RViz Guide](docs/rviz_guide.md) for detailed visualization instructions.

---

## Step 5: Python Dependencies

```bash
# Install from requirements.txt
pip3 install -r requirements.txt --break-system-packages

# Or manually:
pip3 install --break-system-packages \
  numpy \
  opencv-python \
  scipy \
  transforms3d \
  psutil \
  tqdm \
  lz4 \
  laspy[lazrs]
```

---

## Step 6: Hardware Validation Script

Create `/usr/local/bin/validate_hardware.sh`:

```bash
#!/bin/bash
# Hardware validation script

echo "=== Hardware Validation ==="

# Check ROS 2
echo "1. Testing ROS 2 installation..."
if ros2 --version; then
    echo "✓ ROS 2 installed"
else
    echo "✗ ROS 2 not found"
    exit 1
fi

# Check camera
echo "2. Testing camera..."
if libcamera-hello --list-cameras 2>/dev/null | grep -q "Available"; then
    echo "✓ Camera detected"
else
    echo "✗ Camera not detected"
fi

# Check LiDAR
echo "3. Testing LiDAR connectivity..."
if ping -c 1 192.168.1.150 >/dev/null 2>&1; then
    echo "✓ LiDAR reachable at 192.168.1.150"
else
    echo "✗ LiDAR not reachable"
fi

# Check Python packages
echo "4. Testing Python packages..."
python3 -c "
try:
    import numpy, cv2, scipy, transforms3d, psutil
    print('✓ Core Python packages installed')
except ImportError as e:
    print(f'✗ Missing package: {e}')
    exit 1
"

echo ""
echo "Hardware validation complete!"
```

Make it executable:
```bash
sudo chmod +x /usr/local/bin/validate_hardware.sh
```

---

## Compatibility Matrix

| Component | Ubuntu 24.04 | ROS 2 Jazzy | Raspberry Pi 5 | Status |
|-----------|--------------|--------------|----------------|--------|
| **Raspberry Pi Camera v3** | ✅ Full | ✅ via picamera2 | ✅ Native | Recommended |
| **Pi Global Shutter Camera** | ✅ Full | ✅ via picamera2 | ✅ Native | Production Ready |
| **Pi AI Camera** | ✅ Full | ⚠️ Custom bridge needed | ✅ Native | Experimental |
| **USB Camera (generic)** | ✅ Full | ✅ via usb_cam | ✅ Native | Fallback |
| **Unitree L2 LiDAR** | ✅ Full | ✅ Native driver | ✅ Ethernet | Recommended |
| **Unitree L1 LiDAR** | ✅ Full | ✅ Native driver | ✅ Serial/USB | Alternative |

---

## Performance Expectations

| Metric | Raspberry Pi 5 (8GB) | Target | Status |
|--------|---------------------|--------|--------|
| **Fusion Rate** | 15-25 FPS | 20+ FPS | ✅ Meets target |
| **Memory Usage** | 800MB-1.5GB | < 2GB | ✅ Acceptable |
| **CPU Usage** | 60-80% | < 85% | ✅ Headroom available |
| **Thermal** | 60-75°C | < 80°C | ✅ Within limits |
| **Network Latency** | 5-15ms | < 50ms | ✅ Excellent |

---

## Troubleshooting

### Camera Issues

**Problem**: Camera not detected
```bash
# Check camera connection
libcamera-hello --list-cameras

# Check permissions
sudo usermod -a -G video $USER
# Log out and back in

# Check kernel modules
lsmod | grep bcm2835
```

**Problem**: Camera low FPS
```bash
# Check camera bandwidth (picamera2)
python3 << EOF
from picamera2 import Picamera2
cam = Picamera2()
config = cam.create_preview_configuration(main={"size": (640, 480)})
cam.configure(config)
cam.start()
cam.stop()
# Should run at 30 FPS
EOF
```

### LiDAR Issues

**Problem**: Cannot reach LiDAR at 192.168.1.150
```bash
# Check network interface
ip addr show eth0

# Reset network
sudo nmcli connection down "l2-lidar"
sudo nmcli connection up "l2-lidar"

# Check firewall
sudo ufw status
sudo ufw allow from 192.168.1.0/24
```

**Problem**: Low point cloud rate
```bash
# Check topic frequency
ros2 topic hz /livox/lidar

# Check network bandwidth
iftop -i eth0  # Install: sudo apt install iftop
```

### Performance Issues

**Problem**: High CPU usage
```bash
# Check temperature
vcgencmd measure_temp

# Monitor processes
htop  # Install: sudo apt install htop

# Enable performance governor
sudo cpupower frequency-set -g performance
```

**Problem**: Memory exhaustion
```bash
# Check memory usage
free -h

# Enable swap if needed
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

---

## Next Steps

1. ✅ Hardware validated
2. ✅ Drivers installed
3. ✅ ROS 2 configured
4. → **Install fusion software** (see DEVELOPMENT_GUIDE.md)
5. → **Run calibration** (python3 src/core/calibration.py)
6. → **Test fusion** (python3 src/core/fusion.py)
