#!/bin/bash
# Launch LiDAR + Moving Sensor Handler for Vehicle-Mounted Setup

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# Source ROS
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

log_info "Launching LiDAR for moving vehicle setup..."

# Launch LiDAR
./launch_lidar_only.sh &
LIDAR_PID=$!

log_info "LiDAR launched (PID: $LIDAR_PID)"
log_info "Waiting 3 seconds for LiDAR to start..."
sleep 3

log_info "Starting Moving Sensor Handler..."
log_info "This will update TF transforms as vehicle moves"
log_info "Press Ctrl+C to stop both"

# Launch moving sensor handler
python3 moving_sensor_handler.py &
HANDLER_PID=$!

# Wait for both
wait $LIDAR_PID $HANDLER_PID

