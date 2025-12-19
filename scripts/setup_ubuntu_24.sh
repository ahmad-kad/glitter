#!/bin/bash
# Automated Ubuntu 24.04 LTS Setup for LiDAR-Camera Fusion
# Compatible with Raspberry Pi 5, Unitree L2, and various cameras

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
PROJECT_NAME="glitter"
ROS_DISTRO="jazzy"
UBUNTU_VERSION="24.04"

# Functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_root() {
    if [[ $EUID -eq 0 ]]; then
        log_error "Don't run as root. Use a regular user with sudo access."
        exit 1
    fi
}

check_ubuntu_version() {
    if ! grep -q "$UBUNTU_VERSION" /etc/os-release; then
        log_warning "This script is optimized for Ubuntu $UBUNTU_VERSION. You may experience issues."
        read -p "Continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
}

# Main installation
main() {
    log_info "Starting LiDAR-Camera Fusion Setup for Ubuntu $UBUNTU_VERSION"

    check_root
    check_ubuntu_version

    # Update system
    log_info "Updating system packages..."
    sudo apt update && sudo apt full-upgrade -y

    # Install essential tools
    log_info "Installing essential tools..."
    sudo apt install -y \
        curl \
        wget \
        git \
        htop \
        neofetch \
        vim \
        nano \
        build-essential \
        cmake \
        pkg-config \
        python3-pip \
        python3-dev \
        python3-setuptools \
        software-properties-common \
        locales \
        chrony \
        ufw

    # Configure locale
    log_info "Configuring system locale..."
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # Configure timezone
    log_info "Setting timezone..."
    sudo timedatectl set-timezone UTC  # Or your preferred timezone

    # Install ROS 2 Jazzy
    install_ros2

    # Install camera support
    install_camera_support

    # Install Python dependencies
    install_python_deps

    # Configure system for real-time performance
    configure_realtime

    # Create workspace and clone project
    setup_workspace

    # Final validation
    validate_installation

    log_success "Setup complete! Run: cd ~/$PROJECT_NAME && python3 tests/test_system.py"
}

install_ros2() {
    log_info "Installing ROS 2 $ROS_DISTRO..."

    # Add ROS 2 repository
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update

    # Install ROS 2 base (not desktop to save space)
    sudo apt install -y ros-$ROS_DISTRO-ros-base

    # Install additional ROS packages
    sudo apt install -y \
        ros-$ROS_DISTRO-sensor-msgs \
        ros-$ROS_DISTRO-sensor-msgs-py \
        ros-$ROS_DISTRO-cv-bridge \
        ros-$ROS_DISTRO-tf2-ros \
        ros-$ROS_DISTRO-message-filters \
        ros-$ROS_DISTRO-diagnostic-msgs \
        ros-$ROS_DISTRO-image-transport \
        ros-$ROS_DISTRO-image-transport-plugins \
        ros-$ROS_DISTRO-usb-cam \
        python3-rosdep \
        python3-colcon-common-extensions

    # Initialize rosdep
    sudo rosdep init
    rosdep update

    # Add to bashrc
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
    source /opt/ros/$ROS_DISTRO/setup.bash

    log_success "ROS 2 $ROS_DISTRO installed"
}

install_camera_support() {
    log_info "Installing camera support..."

    # V4L2 tools (for USB cameras)
    sudo apt install -y v4l-utils

    # Pi Camera support (if on Raspberry Pi)
    if [[ $(uname -m) == "aarch64" ]]; then
        log_info "Detected ARM64 architecture - installing Pi Camera support..."

        # libcamera and picamera2 for Pi Camera v3, Global Shutter, AI Camera
        sudo apt install -y \
            python3-libcamera \
            python3-picamera2 \
            python3-kms++ \
            libcap-dev

        # Additional packages for AI Camera
        sudo apt install -y python3-hailo || log_warning "Hailo AI package not available"
    fi

    log_success "Camera support installed"
}

install_python_deps() {
    log_info "Installing Python dependencies..."

    # Install from requirements.txt
    if [[ -f "requirements.txt" ]]; then
        pip3 install -r requirements.txt --break-system-packages
    else
        # Fallback: install core packages
        pip3 install --break-system-packages \
            numpy \
            opencv-python \
            scipy \
            transforms3d \
            psutil \
            tqdm \
            lz4
    fi

    # Optional: laspy for LAZ compression
    pip3 install --break-system-packages laspy || log_warning "laspy not available"

    log_success "Python dependencies installed"
}

configure_realtime() {
    log_info "Configuring system for real-time performance..."

    # Set CPU governor to performance (if available)
    if [[ -f "/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor" ]]; then
        echo "performance" | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor || true
    fi

    # Increase file descriptor limits
    echo "* soft nofile 65536" | sudo tee -a /etc/security/limits.conf
    echo "* hard nofile 65536" | sudo tee -a /etc/security/limits.conf

    # Configure chrony for better time synchronization
    sudo systemctl enable chrony
    sudo systemctl start chrony

    # Disable unnecessary services for embedded systems
    sudo systemctl disable bluetooth.service 2>/dev/null || true
    sudo systemctl disable avahi-daemon.service 2>/dev/null || true

    # Enable firewall but allow ROS
    sudo ufw --force enable
    sudo ufw allow 11311  # ROS master
    sudo ufw allow ssh

    log_success "Real-time configuration complete"
}

setup_workspace() {
    log_info "Setting up development workspace..."

    # Create workspace structure
    mkdir -p ~/$PROJECT_NAME
    cd ~/$PROJECT_NAME

    # Create ROS 2 workspace
    mkdir -p ros2_ws/src

    # Clone project (assuming it's available)
    # git clone <repo-url> .

    log_success "Workspace setup complete"
}

validate_installation() {
    log_info "Validating installation..."

    # Test ROS 2
    if ros2 --version >/dev/null 2>&1; then
        log_success "ROS 2 working"
    else
        log_error "ROS 2 not working"
        exit 1
    fi

    # Test Python packages
    python3 << EOF
try:
    import numpy, cv2, scipy, transforms3d, psutil
    print("Core Python packages: OK")
except ImportError as e:
    print(f"Missing Python package: {e}")
    exit(1)
EOF

    if [[ $? -eq 0 ]]; then
        log_success "Python packages working"
    else
        log_error "Python packages not working"
        exit 1
    fi

    # Test camera (if available)
    if command -v libcamera-hello >/dev/null 2>&1; then
        if libcamera-hello --list-cameras 2>/dev/null | grep -q "Available"; then
            log_success "Camera detected"
        else
            log_warning "No camera detected (may be normal if not connected)"
        fi
    fi

    log_success "Installation validation complete"
}

# Run main installation
main "$@"
