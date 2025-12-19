#!/bin/bash
# Launch LiDAR driver and RViz for visualization
# Usage: ./launch_lidar_rviz.sh [lidar_ip] [local_ip]

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Configuration
LIDAR_IP="${1:-192.168.1.62}"  # Default from launch file
LOCAL_IP="${2:-192.168.1.100}"  # Current system IP
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

# Check ROS 2
if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    log_error "ROS 2 $ROS_DISTRO not found at /opt/ros/$ROS_DISTRO"
    exit 1
fi

# Source ROS 2
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
    log_info "ROS 2 $ROS_DISTRO sourced"
else
    log_error "ROS 2 $ROS_DISTRO not found at /opt/ros/$ROS_DISTRO"
    exit 1
fi

# Check workspace
if [ ! -f "$HOME/ros2_ws/install/setup.bash" ]; then
    log_error "ROS workspace not built. Run: cd ~/ros2_ws && colcon build"
    exit 1
fi

# Source workspace
source "$HOME/ros2_ws/install/setup.bash"
log_info "ROS workspace sourced"

# Verify sourcing worked
if ! command -v ros2 &> /dev/null; then
    log_error "ros2 command not found after sourcing. Check ROS installation."
    exit 1
fi

# Check if rviz2 is available
if ! ros2 pkg list 2>/dev/null | grep -q "rviz2"; then
    log_warn "rviz2 package not found"
    log_warn "Installing rviz2..."
    sudo apt update
    sudo apt install -y ros-jazzy-rviz2 || {
        log_error "Failed to install rviz2"
        log_error "Install manually: sudo apt install -y ros-jazzy-rviz2"
        log_warn "Continuing without RViz - LiDAR driver will still launch"
        SKIP_RVIZ=true
    }
    # Re-source after installation
    source /opt/ros/$ROS_DISTRO/setup.bash
fi

# Check if package exists
if ! ros2 pkg list 2>/dev/null | grep -q "unitree_lidar_ros2"; then
    log_warn "unitree_lidar_ros2 package not found in ros2 pkg list"
    log_warn "This may be normal - checking if executable exists..."
    
    # Check if executable exists directly
    if [ ! -f "$HOME/ros2_ws/install/unitree_lidar_ros2/lib/unitree_lidar_ros2/unitree_lidar_ros2_node" ]; then
        log_error "LiDAR driver executable not found. Rebuilding..."
        cd ~/ros2_ws
        # Ensure ROS is sourced before building
        source /opt/ros/$ROS_DISTRO/setup.bash
        colcon build --packages-select unitree_lidar_ros2
        source "$HOME/ros2_ws/install/setup.bash"
    else
        log_info "Executable found, continuing..."
    fi
fi

# Test network connectivity
log_info "Testing connectivity to LiDAR at $LIDAR_IP..."
if ping -c 1 -W 2 "$LIDAR_IP" >/dev/null 2>&1; then
    log_info "✓ LiDAR reachable at $LIDAR_IP"
else
    log_warn "⚠ Cannot ping LiDAR at $LIDAR_IP"
    log_warn "  This may be normal if LiDAR uses UDP only"
    log_warn "  Continuing anyway..."
fi

# Check current IP
CURRENT_IP=$(ip addr show eth0 | grep "inet 192.168.1" | awk '{print $2}' | cut -d'/' -f1)
if [ -n "$CURRENT_IP" ]; then
    log_info "Current IP on eth0: $CURRENT_IP"
    if [ "$LOCAL_IP" != "$CURRENT_IP" ]; then
        log_warn "Local IP mismatch: specified $LOCAL_IP but system has $CURRENT_IP"
        log_warn "Using system IP: $CURRENT_IP"
        LOCAL_IP="$CURRENT_IP"
    fi
else
    log_warn "Not on 192.168.1.x network. LiDAR may not work."
fi

# Create custom launch file with correct IPs
LAUNCH_FILE="/tmp/lidar_launch_$(date +%s).py"
cat > "$LAUNCH_FILE" << EOF
import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # LiDAR node with custom IPs
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
    
    # Static TF broadcaster - create base_link -> unilidar_lidar transform
    # This fixes the "could not transform to base_link" error
    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'unilidar_lidar'],
        output='screen'
    )
    
    # Also create map -> base_link for completeness
    static_tf_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )
    
    nodes = [lidar_node, static_tf_base, static_tf_map]
    
    # Add RViz if available
    try:
        rviz_config = os.path.expanduser('~/glitter/config/l2_fusion.rviz')
        if not os.path.exists(rviz_config):
            # Use default RViz config from package
            package_path = subprocess.check_output(['ros2', 'pkg', 'prefix', 'unitree_lidar_ros2']).decode('utf-8').rstrip()
            rviz_config = os.path.join(package_path, 'share', 'unitree_lidar_ros2', 'view.rviz')
        
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
        nodes.append(rviz_node)
    except Exception as e:
        print(f"Warning: Could not add RViz node: {e}")
        print("Launching LiDAR driver only. Install rviz2: sudo apt install -y ros-jazzy-rviz2")
    
    return LaunchDescription(nodes)
EOF

log_info "Created launch file: $LAUNCH_FILE"
log_info "Configuration:"
log_info "  LiDAR IP: $LIDAR_IP"
log_info "  Local IP: $LOCAL_IP"
log_info "  Topic: /unilidar/cloud"

if [ "${SKIP_RVIZ:-false}" = "true" ]; then
    log_warn "Launching LiDAR driver only (RViz not available)"
    log_warn "To install RViz: sudo apt install -y ros-jazzy-rviz2"
    log_warn "Then launch RViz manually: rviz2 -d ~/glitter/config/l2_fusion.rviz"
else
    log_info "Launching LiDAR driver and RViz..."
fi

log_info "Press Ctrl+C to stop"
log_info ""

# Launch
ros2 launch "$LAUNCH_FILE"

