# LiDAR-Camera Fusion Guide

## Overview

This project fuses 3D LiDAR point clouds with 2D camera images to create colored 3D point clouds for mapping and visualization.

## Architecture

### Core Components

- **`utils.py`** - Mathematical utilities and data processing
- **`fusion.py`** - Real-time sensor fusion node
- **`calibration.py`** - Extrinsic parameter calibration
- **`map.py`** - Point cloud accumulation and saving
- **`simulator.py`** - Synthetic data generation for testing

### Key Algorithms

#### Coordinate Transformations
- LiDAR → Camera coordinate system conversion
- 3D → 2D projection for color sampling
- Extrinsic parameter handling (rotation + translation)

#### Data Processing
- Point cloud filtering (depth, bounds)
- Color sampling from camera images
- Temporal synchronization of sensor data

## Hardware Setup

### Supported Hardware
- **LiDAR**: Unitree L2 4D (primary) or L1 (fallback)
- **Camera**: Raspberry Pi Camera v3 (primary) or USB cameras
- **Platform**: Raspberry Pi 5 with Ubuntu 24.04 + ROS 2 Jazzy

### Connection Setup
1. Connect LiDAR to USB/Ethernet port
2. Attach camera to CSI port (Pi Camera) or USB
3. Ensure power supply can handle both devices

## Usage Workflow

### 1. System Setup
```bash
./start.sh  # Automated setup
python3 test_system.py  # Validation
```

### 2. Calibration
```bash
python3 calibration.py  # Tune extrinsic parameters
# Adjust sliders until alignment looks good
# Press 'S' to save parameters
```

### 3. Fusion
```bash
python3 fusion.py  # Start real-time fusion
# In another terminal:
ros2 run rviz2 rviz2 -d l2_fusion.rviz
```

### 4. Mapping (Optional)
```bash
python3 map.py  # Accumulate point clouds
# Ctrl+C to save and exit
```

## Configuration

### Key Parameters

#### Camera Intrinsics
- **Resolution**: 1280x720 (configurable)
- **Focal length**: Auto-calculated for Pi Camera v3
- **Principal point**: Image center

#### LiDAR Settings
- **Range**: 0.1m - 20m (configurable)
- **Topics**: `/livox/lidar` (L2) or `/unilidar/cloud` (L1)
- **Noise model**: Configurable standard deviation

#### Fusion Parameters
- **Depth filtering**: Remove points behind camera
- **Bounds checking**: Only color visible points
- **Temporal sync**: 100ms tolerance for message alignment

## 🧪 **Synthetic Testing & Simulation**

### **Simulator Features**
- **Realistic Scene Generation**: Ground planes, cubes, spheres, cylinders
- **Configurable Sensor Parameters**: LiDAR range, camera intrinsics, noise levels
- **Ground Truth Data**: Perfect correspondences between points and pixels
- **ROS-Compatible Output**: Direct integration with fusion pipeline
- **Performance Testing**: Stress test with various scene complexities

### **Simulator Usage**
```python
from simulator import SensorSimulator, SensorConfig

# Configure sensors
config = SensorConfig()
config.num_objects = 5
config.lidar_noise_std = 0.01

# Create simulator
simulator = SensorSimulator(config)

# Generate synthetic data
points_xyz, colors_bgr = simulator.generate_point_cloud()
image_bgr, metadata = simulator.generate_camera_image(points_xyz, colors_bgr)

# Use with fusion pipeline
ros_data = simulator.create_ros_messages(points_xyz, colors_bgr, image_bgr)
```

### **Automated Testing**
The test suite includes simulator validation:
```bash
python3 test_system.py  # Includes simulator test
```

### Code Reduction
- **Total lines reduced**: 33% (597 → 401 lines)
- **utils.py**: 38% reduction (217 → 134 lines)
- **fusion.py**: 36% reduction (132 → 85 lines)
- **calibration.py**: 32% reduction (133 → 91 lines)
- **map.py**: 21% reduction (115 → 91 lines)

## 1. Prerequisites & Complete Setup Guide

### **Hardware Requirements**
- Raspberry Pi 5 (8GB RAM recommended)
- Unitree L1 LiDAR
- USB Camera or Raspberry Pi Camera Module
- MicroSD card (32GB+ recommended)
- Power supply (27W USB-C PD recommended)

### **Software Requirements**
- Ubuntu Server 24.04 LTS (64-bit) for Raspberry Pi 5
- ROS 2 Jazzy Jalisco
- Python 3.10+

---

## 🛠️ **Complete Raspberry Pi 5 Setup Guide**

### **Step 1: System Preparation**

#### **Update System & Install Essentials**
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install -y curl wget git htop neofetch vim nano python3-pip python3-dev build-essential

# Install system libraries needed for ROS
sudo apt install -y libssl-dev libffi-dev libxml2-dev libxslt-dev libjpeg-dev zlib1g-dev
```

#### **Configure Locale & Timezone**
```bash
# Set locale
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8

# Set timezone (adjust to your location)
sudo timedatectl set-timezone America/New_York
```

### **Step 2: ROS 2 Jazzy Installation**

#### **Add ROS 2 Repository**
```bash
# Setup sources
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package index
sudo apt update
```

#### **Install ROS 2 Jazzy Desktop**
```bash
# Install ROS 2 Jazzy desktop (includes tools)
sudo apt install -y ros-jazzy-desktop

# Install ROS development tools
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update
```

#### **Setup ROS Environment**
```bash
# Add to .bashrc for automatic sourcing
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc  # Optional: set domain ID for multi-robot
source ~/.bashrc
```

### **Step 3: Install Project Dependencies**

#### **Create Project Workspace**
```bash
# Create workspace directory
mkdir -p ~/glitter_ws/src
cd ~/glitter_ws/src

# Copy project files here (utils.py, calibration.py, fusion.py, map.py, project_guide.md)
```

#### **Install Python Dependencies**
```bash
# Install core dependencies
sudo apt install -y python3-opencv python3-numpy python3-open3d

# Install ROS Python packages
sudo apt install -y ros-jazzy-sensor-msgs-py ros-jazzy-cv-bridge ros-jazzy-tf2-ros ros-jazzy-message-filters

# Install optional performance libraries
pip3 install scipy ros-numpy tqdm --break-system-packages
```

### **Step 4: Hardware Setup**

#### **USB Camera Setup**
```bash
# Install camera tools
sudo apt install -y v4l-utils

# Check available cameras
v4l2-ctl --list-devices

# Test camera capture
v4l2-ctl --device=/dev/video0 --all
```

#### **Pi Camera Setup (Alternative)**
```bash
# Enable camera in raspi-config (if using Raspberry Pi OS)
sudo raspi-config

# Navigate: Interfacing Options → Camera → Enable

# Install libcamera packages
sudo apt install -y python3-libcamera python3-picamera2
```

#### **Unitree L1 LiDAR Setup**
```bash
# Install serial communication tools
sudo apt install -y python3-serial minicom

# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER

# Note: Install Unitree L1 ROS driver according to their documentation
# Typically involves cloning and building their ROS package
```

---

## 📜 **Automated Setup Script**

Create `setup_rpi5.sh` in your home directory:

```bash
#!/bin/bash
# Raspberry Pi 5 LiDAR-Camera Fusion Setup Script
# Run with: bash setup_rpi5.sh

set -e

echo "🚀 Starting Raspberry Pi 5 LiDAR-Camera Fusion Setup"

# Update system
echo "📦 Updating system..."
sudo apt update && sudo apt upgrade -y

# Install essentials
echo "🔧 Installing essential tools..."
sudo apt install -y curl wget git htop neofetch vim nano python3-pip python3-dev build-essential

# ROS 2 Installation
echo "🤖 Installing ROS 2 Jazzy..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

sudo rosdep init
rosdep update

# Setup ROS environment
echo "⚙️ Configuring ROS environment..."
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc

# Install project dependencies
echo "📚 Installing project dependencies..."
sudo apt install -y python3-opencv python3-numpy python3-open3d
sudo apt install -y ros-jazzy-sensor-msgs-py ros-jazzy-cv-bridge ros-jazzy-tf2-ros ros-jazzy-message-filters

# Optional performance libraries
pip3 install scipy ros-numpy --break-system-packages

# Camera setup
echo "📷 Setting up camera..."
sudo apt install -y v4l-utils

echo "✅ Setup complete!"
echo ""
echo "🎯 Next steps:"
echo "1. Connect your hardware (LiDAR + Camera)"
echo "2. Copy project files to ~/glitter_ws/src/"
echo "3. Run tests (see testing section below)"
echo ""
echo "🔄 You may need to reboot: sudo reboot"
```

**Make executable and run:**
```bash
chmod +x start.sh
./start.sh
```

### **Alternative Usage**

#### **Quick Setup (Skip System Updates)**
```bash
./start.sh --skip-system
```

#### **Test Only Mode**
```bash
./start.sh --test-only
```

#### **Help**
```bash
./start.sh --help
```

---

## 🧪 **Testing Procedures**

### **Test 1: System Health Check**
```bash
# Check ROS installation
source /opt/ros/jazzy/setup.bash
ros2 --version

# Check Python packages
python3 -c "import cv2, numpy, open3d; print('✓ Core packages working')"

# Check optional packages
python3 -c "import scipy, ros_numpy; print('✓ Performance packages working')" || echo "⚠️ Optional packages not available"
```

### **Test 2: Camera System Test**
```bash
# Test USB camera
source /opt/ros/jazzy/setup.bash

# Start camera node
ros2 run v4l2_camera v4l2_camera_node

# In another terminal, check topics
ros2 topic list
ros2 topic echo /camera/image_raw --once
```

### **Test 3: LiDAR System Test**
```bash
# Connect LiDAR and test (adjust device path as needed)
source /opt/ros/jazzy/setup.bash

# Start LiDAR driver (adjust command based on Unitree documentation)
ros2 launch unitree_lidar_ros2 radar.launch.py

# Check topics
ros2 topic list
ros2 topic echo /unilidar/cloud --once
```

### **Test 4: Fusion System Test**
```bash
cd ~/glitter_ws/src

# Test individual components
python3 -c "import utils; print('✓ Utils working')"

# Test fusion
python3 fusion.py &
sleep 2
ros2 topic list
ros2 topic echo /unilidar/colored_cloud --once
pkill -f fusion.py
```

### **Test 5: Mapping System Test**
```bash
# Test mapping
python3 map.py --ros-args -p target_frame:=unilidar_lidar &
sleep 5
pkill -f map.py

# Check if map file was created
ls -la *.pcd
```

---

## 🔍 **Troubleshooting Guide**

### **Common Issues & Solutions**

#### **ROS Import Errors**
```bash
# Ensure ROS environment is sourced
source /opt/ros/jazzy/setup.bash

# Check Python path
export PYTHONPATH=$PYTHONPATH:/opt/ros/jazzy/lib/python3.12/site-packages
```

#### **Camera Not Detected**
```bash
# Check permissions
ls -la /dev/video*

# Add user to video group
sudo usermod -a -G video $USER
# Logout and login again
```

#### **Memory Issues on Pi 5**
```bash
# Check memory usage
free -h

# Enable swap if needed
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

#### **Performance Issues**
```bash
# Check CPU temperature
vcgencmd measure_temp

# Monitor system resources
htop
```

All scripts depend on `utils.py` for shared functionality. The optional libraries provide significant performance improvements and reduced complexity.

## 📊 **Logging & Monitoring**

### **Automatic Logging**
The system includes comprehensive logging for debugging and monitoring:

- **calibration.log**: Calibration GUI performance and parameters
- **fusion.log**: Real-time fusion processing statistics
- **map_builder.log**: Mapping progress and statistics
- **system_test.log**: Comprehensive test results and diagnostics

### **Performance Tracking**
- Frame processing times and statistics
- Point cloud processing metrics
- Memory usage and system resources
- Error tracking and recovery

### **System Testing**
Run comprehensive system validation:
```bash
python3 test_system.py
```

The test suite validates:
- ✅ System health and ROS installation
- ✅ Python dependencies and versions
- ✅ Hardware detection (cameras, LiDAR)
- ✅ Code compilation and imports
- ✅ Performance baselines
- 📊 Detailed logging and troubleshooting reports

## 2. Hardware Considerations (Read Before Flying)

Since you are building a "SLAM-like" moving system, keep these physics in mind:

| Issue | Effect on Data | Solution |
|-------|---------------|----------|
| Rolling Shutter | If you turn the robot quickly, the camera image "slants". The fusion will paint the wrong colors onto the LiDAR points. | Move slowly when turning. For production, use a Global Shutter camera. |
| Field of View | The L1 sees 360°. Your camera likely sees 60°. You will have a "flashlight" of color in a dark room. | Point the camera forward. Ignore the black points in the viewer. |
| Time Sync | If the camera is 100ms late, the color trails behind the object as you move. | The fusion node uses ApproximateTimeSynchronizer. Ensure your network isn't clogged. |

## 3. Setup & Calibration

Start Sensors:
```
ros2 launch unitree_lidar_ros2 radar.launch.py
ros2 run v4l2_camera v4l2_camera_node
```

Calibrate:
```
python3 calibration.py
```
- Align the dots with corners of walls or distinct objects.
- Press 'S' to save parameters to console.
- Copy values into fusion.py parameters.

Start Fusion:
```
python3 fusion.py
```
Topic: `/unilidar/colored_cloud`

## 4. "SLAM-like" Moving & Saving

To actually build a map as you walk, you have two options:

### Option A: The "Fake" Map (Visual Only)
Use this if you just want to see it in RViz but don't need to save a file.
- Open RViz2
- Set Fixed Frame to `unilidar_lidar`
- Add PointCloud2 display for `/unilidar/colored_cloud`
- Set Decay Time to 10000
- Walk around. The points will "stick" to the screen.

### Option B: The Real Map (Save to File)
This requires a transform. If you aren't running SLAM (like point_lio), you can only save a "snapshot" of the current view.

To Save a Snapshot (No SLAM):
```
# This will save the instantaneous view (what the lidar sees right now)
python3 map.py --ros-args -p target_frame:=unilidar_lidar
```

To Build a Map (With SLAM):
- Install and run point_lio_unilidar (this provides the map -> unilidar_lidar transform)
- Run the map builder:
```
python3 map.py --ros-args -p target_frame:=map
```
- Walk around. The script will accumulate points.
- Press Ctrl+C. It will save `colored_map.pcd` to your folder.
- View it: `open3d viewer colored_map.pcd` or use MeshLab.