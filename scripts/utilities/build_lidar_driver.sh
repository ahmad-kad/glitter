#!/bin/bash
# Build LiDAR Driver Script
# Run this to install dependencies and build the Unitree L2 LiDAR driver

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_info "Installing dependencies for Unitree L2 LiDAR driver..."

# Install PCL and ROS packages
log_info "Installing PCL and ROS dependencies..."
sudo apt update
sudo apt install -y \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    libpcl-dev \
    libyaml-cpp-dev \
    ros-jazzy-rosidl-generator-dds-idl \
    ros-jazzy-rmw-cyclonedds-cpp

log_info "Installing package dependencies via rosdep..."
cd ~/ros2_ws

# Source ROS 2 first
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    log_info "ROS 2 Jazzy sourced"
else
    log_error "ROS 2 Jazzy not found at /opt/ros/jazzy"
    exit 1
fi

rosdep install --from-paths src --ignore-src -r -y || log_warn "Some rosdep packages failed (may be OK)"

log_info "Building LiDAR driver..."
colcon build --packages-select unitree_lidar_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release

log_info "Sourcing workspace..."
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
    log_info "Workspace sourced"
else
    log_error "Workspace setup.bash not found"
    exit 1
fi

log_info "Verifying installation..."
if ros2 pkg list | grep -q "unitree_lidar_ros2"; then
    log_info "✓ Driver package found!"
else
    log_warn "Package not in ros2 pkg list, but may still work"
fi

if [ -f "install/unitree_lidar_ros2/lib/unitree_lidar_ros2/unitree_lidar_ros2_node" ]; then
    log_info "✓ Driver executable found!"
else
    log_error "Driver executable not found after build"
    exit 1
fi

log_info ""
log_info "=========================================="
log_info "Driver build complete!"
log_info "=========================================="
log_info ""
log_info "To launch LiDAR and RViz:"
log_info "  cd ~/glitter"
log_info "  ./launch_lidar_rviz.sh"
log_info ""


