#!/bin/bash
# Wait for USB LiDAR device and start the node automatically
# Usage: ./wait_and_start_lidar_usb.sh [device_name]

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

DEVICE_PATTERN="${1:-ttyACM}"
ROS_DISTRO="jazzy"
MAX_WAIT=60  # Maximum seconds to wait

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_debug() { echo -e "${BLUE}[DEBUG]${NC} $1"; }

# Source ROS 2
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
else
    log_error "ROS 2 $ROS_DISTRO not found"
    exit 1
fi

# Source workspace
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

export ROS_DOMAIN_ID=0

# Function to find USB serial device
find_usb_device() {
    for dev in /dev/ttyACM* /dev/ttyUSB*; do
        if [ -e "$dev" ] && [[ "$dev" == *"$DEVICE_PATTERN"* ]]; then
            echo "$dev"
            return 0
        fi
    done
    return 1
}

# Wait for device
log_info "Waiting for USB LiDAR device (pattern: $DEVICE_PATTERN)..."
log_info "Please connect the LiDAR USB cable if not already connected."
log_info ""

DEVICE=""
WAITED=0

while [ $WAITED -lt $MAX_WAIT ]; do
    DEVICE=$(find_usb_device)
    if [ -n "$DEVICE" ]; then
        log_info "✓ Found device: $DEVICE"
        ls -la "$DEVICE"
        break
    fi
    
    if [ $((WAITED % 5)) -eq 0 ] && [ $WAITED -gt 0 ]; then
        log_debug "Still waiting... ($WAITED/$MAX_WAIT seconds)"
        log_debug "Current USB devices:"
        lsusb 2>/dev/null | grep -i "serial\|cdc\|acm" || echo "  (none found)"
    fi
    
    sleep 1
    WAITED=$((WAITED + 1))
done

if [ -z "$DEVICE" ]; then
    log_error "Device not found after $MAX_WAIT seconds"
    log_info "Please check:"
    log_info "  1. USB cable is connected"
    log_info "  2. LiDAR is powered on"
    log_info "  3. Try a different USB port"
    log_info ""
    log_info "Current USB devices:"
    lsusb 2>/dev/null | head -10
    exit 1
fi

# Check permissions
if [ ! -r "$DEVICE" ] || [ ! -w "$DEVICE" ]; then
    log_warn "Device permissions may be insufficient"
    log_info "Current permissions:"
    ls -la "$DEVICE"
    log_info "If needed, run: sudo chmod 666 $DEVICE"
    log_info "Or add user to dialout group: sudo usermod -aG dialout $USER"
fi

# Stop existing node
log_info "Stopping existing LiDAR node..."
pkill -f unitree_lidar_ros2_node || true
sleep 2

# Create launch file
LAUNCH_FILE="/tmp/lidar_usb_$(date +%s).py"
cat > "$LAUNCH_FILE" << EOF
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[
            {'initialize_type': 2},
            {'work_mode': 0},
            {'use_system_timestamp': True},
            {'range_min': 0.0},
            {'range_max': 100.0},
            {'cloud_scan_num': 18},
            {'serial_port': '$DEVICE'},
            {'baudrate': 4000000},
            {'lidar_port': 6101},
            {'lidar_ip': '192.168.1.62'},
            {'local_port': 6201},
            {'local_ip': '192.168.1.2'},
            {'cloud_frame': "unilidar_lidar"},
            {'cloud_topic': "/unilidar/cloud"},
            {'imu_frame': "unilidar_imu"},
            {'imu_topic': "/unilidar/imu"},
        ]
    )
    
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'unilidar_lidar'],
        output='screen'
    )
    
    return LaunchDescription([lidar_node, static_tf])
EOF

log_info "Launching LiDAR node with device: $DEVICE"
ros2 launch "$LAUNCH_FILE" > /tmp/lidar_usb.log 2>&1 &
LIDAR_PID=$!

log_info "LiDAR node started (PID: $LIDAR_PID)"
log_info "Log file: /tmp/lidar_usb.log"
log_info ""

# Wait and check for messages
log_info "Waiting 8 seconds for initialization..."
sleep 8

# Check if process is still running
if ! ps -p $LIDAR_PID > /dev/null 2>&1; then
    log_error "Node process died. Check logs:"
    tail -20 /tmp/lidar_usb.log
    exit 1
fi

# Check for messages
log_info "Checking for LiDAR messages..."
if timeout 5 ros2 topic hz /unilidar/cloud 2>&1 | grep -q "average rate"; then
    log_info "✓ SUCCESS! LiDAR is publishing messages!"
    timeout 3 ros2 topic hz /unilidar/cloud 2>&1 | head -5
    log_info ""
    log_info "You can now verify synchronization is working."
else
    log_warn "Node is running but not publishing yet."
    log_info "This might be normal - LiDAR may need more time to initialize."
    log_info ""
    log_info "Monitor with:"
    log_info "  tail -f /tmp/lidar_usb.log"
    log_info "  ros2 topic hz /unilidar/cloud"
fi

log_info ""
log_info "To stop: kill $LIDAR_PID"

