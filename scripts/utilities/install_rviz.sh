#!/bin/bash
# Install RViz2 for ROS 2 Jazzy

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

log_info "Installing RViz2 for ROS 2 Jazzy..."

# Update package list
sudo apt update

# Install RViz2
log_info "Installing rviz2 package..."
sudo apt install -y ros-jazzy-rviz2

# Verify installation
log_info "Verifying installation..."
source /opt/ros/jazzy/setup.bash

if ros2 pkg list | grep -q "rviz2"; then
    log_info "✓ RViz2 installed successfully!"
    log_info ""
    log_info "You can now launch RViz:"
    log_info "  rviz2"
    log_info "  or"
    log_info "  rviz2 -d ~/glitter/config/l2_fusion.rviz"
else
    log_warn "RViz2 package not found in ros2 pkg list"
    log_warn "But it may still be installed. Try: rviz2"
fi

log_info ""
log_info "To launch LiDAR with RViz:"
log_info "  cd ~/glitter"
log_info "  ./launch_lidar_rviz.sh"

