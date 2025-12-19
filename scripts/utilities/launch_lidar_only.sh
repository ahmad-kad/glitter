#!/bin/bash
# Launch LiDAR driver only (no RViz) - for stability testing

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

LIDAR_IP="${1:-192.168.1.62}"
LOCAL_IP="${2:-192.168.1.100}"
ROS_DISTRO="jazzy"

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Source ROS 2
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
    log_info "ROS 2 $ROS_DISTRO sourced"
else
    log_error "ROS 2 $ROS_DISTRO not found"
    exit 1
fi

# Source workspace
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
    log_info "ROS workspace sourced"
else
    log_error "ROS workspace not built"
    exit 1
fi

# Check current IP
CURRENT_IP=$(ip addr show eth0 | grep "inet 192.168.1" | awk '{print $2}' | cut -d'/' -f1)
if [ -n "$CURRENT_IP" ]; then
    log_info "Current IP on eth0: $CURRENT_IP"
    LOCAL_IP="$CURRENT_IP"
fi

# Create launch file
LAUNCH_FILE="/tmp/lidar_only_$(date +%s).py"
cat > "$LAUNCH_FILE" << EOF
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # LiDAR node only
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
            {'serial_port': '/dev/ttyACM0'},
            {'baudrate': 4000000},
            {'lidar_port': 6101},
            {'lidar_ip': '$LIDAR_IP'},
            {'local_port': 6201},
            {'local_ip': '$LOCAL_IP'},
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

log_info "Launching LiDAR driver only (no RViz)"
log_info "LiDAR IP: $LIDAR_IP"
log_info "Local IP: $LOCAL_IP"
log_info ""
log_info "To view in RViz separately:"
log_info "  rviz2 -d ~/glitter/config/l2_fusion_simple.rviz"
log_info ""

ros2 launch "$LAUNCH_FILE"

