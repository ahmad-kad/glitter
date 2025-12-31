#!/bin/bash

# Install RealSense SDK 2.0 and ROS2 realsense-ros package for Ubuntu 24.04 / ROS2 Jazzy
# Based on: https://github.com/IntelRealSense/realsense-ros

set -e

echo "=== Installing RealSense SDK 2.0 and ROS2 realsense-ros ==="

# Update package list
sudo apt update

# Install prerequisites
echo "Installing prerequisites..."
sudo apt install -y \
    software-properties-common \
    apt-transport-https \
    curl \
    git \
    cmake \
    build-essential \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    python3-dev \
    python3-pip

# Add Intel RealSense repository
echo "Adding Intel RealSense repository..."
curl -sSL https://packages.intelrealsense.com/librealsense6/dists/jammy/InRelease | gpg --dearmor | sudo tee /usr/share/keyrings/librealsense.gpg > /dev/null
echo "deb [signed-by=/usr/share/keyrings/librealsense.gpg] https://packages.intelrealsense.com/librealsense6/dists/jammy/main/binary-amd64/ ./" | sudo tee /etc/apt/sources.list.d/librealsense.list

# Update and install RealSense SDK
echo "Installing RealSense SDK 2.0..."
sudo apt update
sudo apt install -y \
    librealsense2-dev \
    librealsense2-utils \
    librealsense2-dbg \
    librealsense2-gl-dev

# Test SDK installation
echo "Testing RealSense SDK installation..."
if realsense-viewer --version; then
    echo "✓ RealSense SDK installed successfully"
else
    echo "✗ RealSense SDK installation failed"
    exit 1
fi

# Install ROS2 realsense-ros package
echo "Installing ROS2 realsense-ros package..."
sudo apt install -y ros-jazzy-realsense2-camera

# Verify ROS2 package installation
echo "Verifying ROS2 realsense-ros installation..."
if ros2 pkg list | grep -q realsense2; then
    echo "✓ ROS2 realsense-ros package installed"
else
    echo "✗ ROS2 realsense-ros package installation failed"
    exit 1
fi

# Install additional dependencies for RealSense ROS2
sudo apt install -y \
    ros-jazzy-realsense2-description \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher

# Install Python dependencies for RealSense
pip3 install --break-system-packages \
    pyrealsense2 \
    numpy \
    opencv-python

echo ""
echo "=== RealSense Installation Complete ==="
echo "To test the camera:"
echo "  1. Connect RealSense 435 camera via USB"
echo "  2. Run: realsense-viewer"
echo "  3. For ROS2: ros2 launch realsense2_camera rs_launch.py"
echo ""
echo "Expected topics:"
echo "  - /camera/color/image_raw"
echo "  - /camera/depth/image_rect_raw"
echo "  - /camera/depth/color/points"
echo "  - /camera/color/camera_info"
echo "  - /camera/depth/camera_info"
