#!/bin/bash
# Restart LiDAR node with USB/serial configuration
# Usage: ./restart_lidar_usb.sh [serial_device]

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

SERIAL_DEVICE="${1:-/dev/ttyACM0}"
ROS_DISTRO="jazzy"

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

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

# Stop existing LiDAR node
log_info "Stopping existing LiDAR node..."
pkill -f unitree_lidar_ros2_node || true
sleep 2

# Check if serial device exists
if [ ! -e "$SERIAL_DEVICE" ]; then
    log_warn "Serial device $SERIAL_DEVICE not found"
    log_info "Available serial devices:"
    ls -la /dev/tty* 2>/dev/null | grep -E "ACM|USB" || echo "  None found"
    log_info "Waiting 5 seconds for device to be detected..."
    sleep 5
    if [ ! -e "$SERIAL_DEVICE" ]; then
        log_error "Device still not found. Please check USB connection."
        exit 1
    fi
fi

log_info "Found serial device: $SERIAL_DEVICE"
ls -la "$SERIAL_DEVICE"

# Check permissions
if [ ! -r "$SERIAL_DEVICE" ] || [ ! -w "$SERIAL_DEVICE" ]; then
    log_warn "Device may need permissions. Try: sudo chmod 666 $SERIAL_DEVICE"
fi

# Create launch file for USB mode
LAUNCH_FILE="/tmp/lidar_usb_$(date +%s).py"
cat > "$LAUNCH_FILE" << EOF
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # LiDAR node configured for USB/serial connection
    lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[
            {'initialize_type': 2},  # USB initialization
            {'work_mode': 0},        # Serial mode (0 = serial, 1 = network)
            {'use_system_timestamp': True},
            {'range_min': 0.0},
            {'range_max': 100.0},
            {'cloud_scan_num': 18},
            {'serial_port': '$SERIAL_DEVICE'},
            {'baudrate': 4000000},
            # Network params still needed but won't be used in serial mode
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
    
    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'unilidar_lidar'],
        output='screen'
    )
    
    return LaunchDescription([lidar_node, static_tf])
EOF

log_info "Launching LiDAR node with USB/serial configuration..."
log_info "Serial device: $SERIAL_DEVICE"
log_info "Baudrate: 4000000"
log_info ""

# Launch in background and capture output
ros2 launch "$LAUNCH_FILE" > /tmp/lidar_usb.log 2>&1 &
LIDAR_PID=$!

log_info "LiDAR node started (PID: $LIDAR_PID)"
log_info "Log file: /tmp/lidar_usb.log"
log_info ""
log_info "Waiting 5 seconds for initialization..."
sleep 5

# Check if node is running
if ps -p $LIDAR_PID > /dev/null; then
    log_info "Node is running. Checking for messages..."
    
    # Check topic
    if timeout 3 ros2 topic hz /unilidar/cloud 2>&1 | grep -q "average rate"; then
        log_info "✓ LiDAR is publishing messages!"
        timeout 3 ros2 topic hz /unilidar/cloud 2>&1 | head -5
    else
        log_warn "LiDAR node running but not publishing yet. Check logs:"
        log_info "  tail -f /tmp/lidar_usb.log"
        log_info "  ros2 topic list | grep unilidar"
    fi
else
    log_error "Node failed to start. Check logs:"
    log_info "  cat /tmp/lidar_usb.log"
    exit 1
fi

log_info ""
log_info "To monitor: tail -f /tmp/lidar_usb.log"
log_info "To stop: kill $LIDAR_PID"

