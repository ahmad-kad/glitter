#!/bin/bash
# Unitree L2 LiDAR Installation Script
# Compatible with Ubuntu 24.04 LTS and ROS 2 Humble

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Configuration
ROS_DISTRO="jazzy"
LIDAR_IP="192.168.1.62"
LIDAR_REPO="https://github.com/unitreerobotics/unilidar_sdk2.git"

configure_network() {
    log_info "Configuring network for Unitree L2..."

    # Check if running on Raspberry Pi (likely to have Ethernet)
    if [[ $(uname -m) == "aarch64" ]]; then
        INTERFACE="eth0"
    else
        # On desktop, might be different
        INTERFACE=$(ip route | grep default | awk '{print $5}' | head -1)
        if [[ -z "$INTERFACE" ]]; then
            INTERFACE="eth0"  # fallback
        fi
    fi

    log_info "Configuring interface: $INTERFACE"

    # Create NetworkManager connection for L2
    sudo nmcli connection add \
        type ethernet \
        con-name "l2-lidar" \
        ifname "$INTERFACE" \
        ipv4.addresses "192.168.1.2/24" \
        ipv4.method manual \
        ipv4.dns "8.8.8.8,8.8.4.4"

    # Set as highest priority
    sudo nmcli connection modify "l2-lidar" connection.autoconnect-priority 100

    # Bring down and up to apply
    sudo nmcli connection down "$INTERFACE" 2>/dev/null || true
    sleep 2
    sudo nmcli connection up "l2-lidar"

    # Verify configuration
    log_info "Network configuration:"
    ip addr show "$INTERFACE" | grep "inet "

    log_success "Network configured for L2 LiDAR"
}

test_connectivity() {
    log_info "Testing L2 connectivity..."

    # Wait a moment for network to settle
    sleep 3

    # Test ping
    if ping -c 3 -W 2 "$LIDAR_IP" >/dev/null 2>&1; then
        log_success "L2 LiDAR reachable at $LIDAR_IP"

        # Get additional info
        log_info "L2 Information:"
        ping -c 1 "$LIDAR_IP" | head -2

    else
        log_error "Cannot reach L2 LiDAR at $LIDAR_IP"
        log_warning "Please check:"
        log_warning "1. L2 is powered on and connected via Ethernet"
        log_warning "2. Ethernet cable is properly connected"
        log_warning "3. L2 IP address is correct (default: $LIDAR_IP)"
        log_warning "4. Firewall is not blocking connection"

        # Show network troubleshooting info
        log_info "Network troubleshooting:"
        ip route
        nmcli connection show --active

        exit 1
    fi
}

install_driver() {
    log_info "Installing Unitree L2 ROS 2 driver..."

    # Create ROS workspace if it doesn't exist
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src

    # Clone driver repository
    if [[ -d "unilidar_sdk2" ]]; then
        log_info "Driver already cloned, updating..."
        cd unilidar_sdk2
        git pull
        cd ..
    else
        log_info "Cloning L2 driver repository..."
        git clone "$LIDAR_REPO"
    fi

    # Install dependencies
    log_info "Installing driver dependencies..."

    # Install required system packages for L2
    sudo apt install -y \
        ros-$ROS_DISTRO-rosidl-generator-dds-idl \
        ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
        libyaml-cpp-dev \
        libpcl-dev

    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y

    # Build driver
    log_info "Building L2 driver..."
    colcon build --packages-select unitree_lidar_ros2 --event-handlers console_cohesion+ --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

    # Source workspace
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    source ~/ros2_ws/install/setup.bash

    log_success "L2 driver installed and built"
}

test_driver() {
    log_info "Testing L2 driver..."

    # Source ROS and workspace
    source /opt/ros/$ROS_DISTRO/setup.bash
    source ~/ros2_ws/install/setup.bash

    # Launch driver in background
    timeout 10s ros2 launch unitree_lidar_ros2 launch.py &
    LAUNCH_PID=$!

    # Wait a moment for launch
    sleep 3

    # Check if topics are available
    local lidar_topics
    lidar_topics=$(ros2 topic list | grep -E "(livox/lidar|pointcloud|unitree_lidar)" | head -1)

    if [[ -n "$lidar_topics" ]]; then
        log_success "L2 driver launched successfully - found topic: $lidar_topics"

        # Check topic frequency
        log_info "Testing data rate..."
        timeout 5 ros2 topic hz "$lidar_topics" | grep -E "average rate|min:|max:" || \
            log_warning "Could not measure topic frequency"

        # Get a sample message
        log_info "Getting sample point cloud..."
        timeout 3 ros2 topic echo "$lidar_topics" --once --no-arr > /dev/null && \
            log_success "Point cloud data received" || \
            log_warning "No point cloud data received (may be normal)"

        # Kill launch process
        kill $LAUNCH_PID 2>/dev/null || true
        wait $LAUNCH_PID 2>/dev/null || true

    else
        log_error "L2 driver failed to launch - no topics found"
        kill $LAUNCH_PID 2>/dev/null || true

        log_warning "Troubleshooting steps:"
        log_warning "1. Check network connectivity to L2"
        log_warning "2. Verify ROS 2 environment is sourced"
        log_warning "3. Check driver build logs: cd ~/ros2_ws && colcon build --packages-select unitree_lidar_ros2"
        log_warning "4. Verify L2 firmware version is compatible"

        exit 1
    fi
}

create_lidar_config() {
    log_info "Creating LiDAR configuration..."

    mkdir -p config

    # LiDAR configuration
    cat > config/lidar_config.yaml << EOF
# Unitree L2 LiDAR Configuration
# Adjust these parameters based on your setup

lidar_ip: "$LIDAR_IP"
local_ip: "192.168.1.2"
port: 60001

# ROS topic names
pointcloud_topic: "/livox/lidar"
imu_topic: "/livox/imu"

# Processing parameters
min_range: 0.1      # Minimum detection range (meters)
max_range: 50.0     # Maximum detection range (meters)
intensity_min: 0    # Minimum intensity threshold
intensity_max: 255  # Maximum intensity threshold

# Coordinate frame
frame_id: "livox_frame"

# Timing parameters
sync_threshold: 0.1  # Maximum time difference for sync (seconds)
timeout: 2.0         # Connection timeout (seconds)

# Data filtering
remove_outliers: true
outlier_threshold: 2.0  # Standard deviations
voxel_filter_size: 0.05  # Voxel filter resolution (meters)
EOF

    log_success "LiDAR configuration created at config/lidar_config.yaml"
}

create_launch_files() {
    log_info "Creating optimized launch files..."

    mkdir -p launch

    # Main launch file
    cat > launch/l2_fusion.launch.py << EOF
#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # L2 LiDAR driver
        Node(
            package='unitree_lidar_ros2',
            executable='unitree_lidar_ros2_node',
            name='l2_lidar_driver',
            parameters=[{
                'lidar_ip': '$LIDAR_IP',
                'local_ip': '192.168.1.2',
                'port': 60001,
                'frame_id': 'livox_frame'
            }],
            output='screen'
        ),

        # Camera driver (will be added based on camera type)
        # Node(
        #     package='your_camera_package',
        #     executable='camera_node',
        #     name='camera_driver',
        #     parameters=[{
        #         'camera_info_url': 'package://glitter/config/camera_calibration.yaml'
        #     }]
        # ),

        # LiDAR-Camera fusion
        Node(
            package='glitter',
            executable='fusion_node',
            name='lidar_camera_fusion',
            parameters=[{
                'camera_calibration_file': 'config/camera_calibration.yaml',
                'lidar_config_file': 'config/lidar_config.yaml',
                'use_infrastructure': True
            }],
            output='screen'
        )
    ])
EOF

    chmod +x launch/l2_fusion.launch.py
    log_success "Launch files created in launch/ directory"
}

print_usage() {
    echo "Unitree L2 LiDAR Setup Script"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --network-only    Configure network only"
    echo "  --driver-only     Install driver only (skip network)"
    echo "  --test-only       Test existing installation"
    echo "  --help           Show this help"
    echo ""
    echo "Examples:"
    echo "  $0                Full installation (network + driver + test)"
    echo "  $0 --network-only Configure network only"
    echo "  $0 --test-only    Test existing installation"
}

main() {
    echo "======================================"
    echo "Unitree L2 LiDAR Setup for Ubuntu 24.04"
    echo "======================================"

    # Parse arguments
    case "${1:-}" in
        --help|-h)
            print_usage
            exit 0
            ;;
        --network-only)
            log_info "Network configuration only"
            configure_network
            test_connectivity
            exit 0
            ;;
        --driver-only)
            log_info "Driver installation only"
            install_driver
            exit 0
            ;;
        --test-only)
            log_info "Testing existing installation"
            test_connectivity
            test_driver
            exit 0
            ;;
        *)
            # Full installation
            ;;
    esac

    # Check if ROS 2 is available
    if ! command -v ros2 >/dev/null 2>&1; then
        log_error "ROS 2 not found. Please run setup_ubuntu_24.sh first."
        exit 1
    fi

    # Full installation
    configure_network
    test_connectivity
    install_driver
    test_driver
    create_lidar_config
    create_launch_files

    log_success "Unitree L2 setup complete!"
    echo ""
    echo "Quick start:"
    echo "  cd ~/ros2_ws && source install/setup.bash"
    echo "  ros2 launch unitree_lidar_ros2 launch.py"
    echo ""
    echo "Test fusion:"
    echo "  cd ~/glitter && python3 src/core/fusion.py"
    echo ""
    echo "For integrated launch:"
    echo "  ros2 launch ~/glitter/launch/l2_fusion.launch.py"
}

main "$@"
