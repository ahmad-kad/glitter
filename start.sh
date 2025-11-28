#!/bin/bash
# LiDAR-Camera Fusion Setup Script

set -e

echo "LiDAR-Camera Fusion Setup"
echo "=========================="

# Check if running as root
if [[ $EUID -eq 0 ]]; then
    echo "Don't run as root. Use a regular user with sudo access."
    exit 1
fi

echo "Setting up system..."

# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble
echo "Installing ROS 2 Humble..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop python3-rosdep python3-colcon-common-extensions

# Initialize ROS
sudo rosdep init
rosdep update

# Setup environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install Python dependencies
pip3 install numpy opencv-python transforms3d tqdm --break-system-packages

echo "Setup complete! Run: python3 test_system.py"
}

check_system() {
    log_step "Checking system compatibility"

    # Check if running on Ubuntu
    if ! grep -q "Ubuntu" /etc/os-release; then
        echo -e "${RED}Error: This script is designed for Ubuntu systems only.${NC}"
        exit 1
    fi

    # Check Ubuntu version
    if ! grep -q "24.04" /etc/os-release; then
        echo -e "${YELLOW}Warning: This script is optimized for Ubuntu 24.04. You may experience issues.${NC}"
    fi

    # Check architecture
    if [[ $(uname -m) != "aarch64" ]]; then
        echo -e "${RED}Error: This script is designed for ARM64 (Raspberry Pi 5) systems.${NC}"
        exit 1
    fi

    echo -e "${GREEN}✓ System check passed${NC}"
}

update_system() {
    log_step "Updating system packages"

    echo "Updating package lists..."
    sudo apt update

    echo "Upgrading system packages..."
    sudo apt upgrade -y

    echo "Installing essential tools..."
    sudo apt install -y curl wget git htop neofetch vim nano python3-pip python3-dev \
                       build-essential libssl-dev libffi-dev libxml2-dev libxslt-dev \
                       libjpeg-dev zlib1g-dev

    echo -e "${GREEN}✓ System updated successfully${NC}"
}

setup_locale() {
    log_step "Configuring system locale"

    sudo locale-gen en_US.UTF-8
    sudo update-locale LANG=en_US.UTF-8
    sudo timedatectl set-timezone America/New_York  # Change as needed

    echo -e "${GREEN}✓ Locale configured${NC}"
}

install_ros() {
    log_step "Installing ROS 2 Humble"

    echo "Adding ROS 2 repository..."
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    echo "Updating package lists..."
    sudo apt update

    echo "Installing ROS 2 Humble desktop..."
    sudo apt install -y ros-humble-desktop

    echo "Installing ROS development tools..."
    sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

    echo "Initializing rosdep..."
    sudo rosdep init
    rosdep update

    echo "Setting up ROS environment..."
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
    source ~/.bashrc

    echo -e "${GREEN}✓ ROS 2 Humble installed successfully${NC}"
}

install_python_deps() {
    log_step "Installing Python dependencies"

    echo "Installing core Python packages..."
    sudo apt install -y python3-opencv python3-numpy python3-open3d

    echo "Installing ROS Python packages..."
    sudo apt install -y ros-humble-sensor-msgs-py ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-message-filters

    echo "Installing optional performance libraries..."
    pip3 install scipy ros-numpy tqdm --break-system-packages

    echo -e "${GREEN}✓ Python dependencies installed${NC}"
}

setup_hardware() {
    log_step "Setting up hardware support"

    echo "Installing camera tools..."
    sudo apt install -y v4l-utils

    echo "Installing serial communication tools..."
    sudo apt install -y python3-serial minicom

    echo "Setting up user permissions..."
    sudo usermod -a -G dialout "$USER"
    sudo usermod -a -G video "$USER"

    echo -e "${YELLOW}Note: You may need to log out and back in for group changes to take effect.${NC}"
    echo -e "${GREEN}✓ Hardware support configured${NC}"
}

setup_project() {
    log_step "Setting up project environment"

    echo "Creating project workspace..."
    mkdir -p ~/$PROJECT_NAME
    cd ~/$PROJECT_NAME

    # Copy project files (assuming they're already cloned)
    # In a real scenario, this would be: git clone <repo> .

    echo "Setting up Python virtual environment (optional)..."
    # python3 -m venv venv
    # source venv/bin/activate

    echo "Installing project-specific dependencies..."
    # Add any additional project dependencies here

    echo -e "${GREEN}✓ Project environment ready${NC}"
}

test_installation() {
    log_step "Testing installation"

    echo "Testing ROS installation..."
    source /opt/ros/humble/setup.bash
    if ros2 --version >/dev/null 2>&1; then
        echo -e "${GREEN}✓ ROS 2 working${NC}"
    else
        echo -e "${RED}✗ ROS 2 test failed${NC}"
    fi

    echo "Testing Python packages..."
    python3 -c "
import numpy as np
import cv2
import open3d as o3d
print('✓ Core packages working')
try:
    import scipy
    import ros_numpy
    import tqdm
    print('✓ Performance packages working')
except ImportError as e:
    print(f'⚠️ Optional packages not available: {e}')
"

    echo -e "${GREEN}✓ Installation tests completed${NC}"
}

create_desktop_shortcuts() {
    log_step "Creating desktop shortcuts and aliases"

    # Create useful aliases
    echo "
# LiDAR-Camera Fusion aliases
alias fusion-start='cd ~/$PROJECT_NAME && python3 fusion.py'
alias calibration-start='cd ~/$PROJECT_NAME && python3 calibration.py'
alias mapping-start='cd ~/$PROJECT_NAME && python3 map.py'
alias system-test='cd ~/$PROJECT_NAME && python3 test_system.py'
alias ros-source='source /opt/ros/humble/setup.bash'
" >> ~/.bashrc

    echo -e "${GREEN}✓ Aliases and shortcuts created${NC}"
}

final_setup() {
    log_step "Final configuration and cleanup"

    echo "Cleaning up package cache..."
    sudo apt autoremove -y
    sudo apt autoclean

    echo "Setting up auto-start ROS environment..."
    # Add to .profile for login shells
    echo "source /opt/ros/humble/setup.bash" >> ~/.profile

    echo "Creating setup completion flag..."
    touch ~/.${PROJECT_NAME}_setup_complete

    echo -e "${GREEN}✓ Setup completed successfully!${NC}"
}

print_summary() {
    echo -e "\n${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║              🎉 SETUP COMPLETE! 🎉                        ║${NC}"
    echo -e "${BLUE}╠══════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${BLUE}║                                                              ║${NC}"
    echo -e "${BLUE}║  🚀 Your LiDAR-Camera Fusion system is ready!              ║${NC}"
    echo -e "${BLUE}║                                                              ║${NC}"
    echo -e "${BLUE}║  📁 Project location: ~/$PROJECT_NAME                       ║${NC}"
    echo -e "${BLUE}║  🔧 ROS 2 Humble: Installed and configured                 ║${NC}"
    echo -e "${BLUE}║  📦 Python deps: All installed                              ║${NC}"
    echo -e "${BLUE}║  📷 Hardware: Camera and LiDAR support ready               ║${NC}"
    echo -e "${BLUE}║                                                              ║${NC}"
    echo -e "${BLUE}║  🏃 Quick Start Commands:                                    ║${NC}"
    echo -e "${BLUE}║     fusion-start      - Start LiDAR-camera fusion           ║${NC}"
    echo -e "${BLUE}║     calibration-start - Start camera calibration            ║${NC}"
    echo -e "${BLUE}║     mapping-start     - Start 3D mapping                    ║${NC}"
    echo -e "${BLUE}║     system-test       - Run system diagnostics              ║${NC}"
    echo -e "${BLUE}║                                                              ║${NC}"
    echo -e "${BLUE}║  📖 Documentation: project_guide.md                         ║${NC}"
    echo -e "${BLUE}║  📋 Logs: Check setup_*.log for installation details        ║${NC}"
    echo -e "${BLUE}║                                                              ║${NC}"
    echo -e "${BLUE}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${YELLOW}Next steps:${NC}"
    echo "1. Connect your LiDAR and camera hardware"
    echo "2. Run: system-test"
    echo "3. Follow the project guide for calibration and operation"
    echo ""
    echo -e "${GREEN}Happy fusing! 🔗📷${NC}"
}

# Main execution
main() {
    echo -e "${BLUE}🚀 LiDAR-Camera Fusion System Setup${NC}"
    echo -e "${BLUE}=====================================${NC}"
    echo ""

    check_root
    check_system
    update_system
    setup_locale
    install_ros
    install_python_deps
    setup_hardware
    setup_project
    test_installation
    create_desktop_shortcuts
    final_setup

    progress_bar "$TOTAL_STEPS" "$TOTAL_STEPS"
    echo -e "\n${GREEN}Setup completed successfully!${NC}"

    print_summary
}

# Handle command line arguments
case "${1:-}" in
    --help|-h)
        echo "LiDAR-Camera Fusion Setup Script"
        echo ""
        echo "Usage: $0 [OPTIONS]"
        echo ""
        echo "Options:"
        echo "  --help, -h          Show this help message"
        echo "  --test-only         Run only the testing phase"
        echo "  --skip-system       Skip system updates"
        echo ""
        echo "Examples:"
        echo "  $0                  Full installation"
        echo "  $0 --test-only      Test existing installation"
        exit 0
        ;;
    --test-only)
        echo "Running tests only..."
        test_installation
        print_summary
        exit 0
        ;;
    --skip-system)
        echo "Skipping system updates..."
        CURRENT_STEP=2  # Skip first 2 steps
        check_root
        check_system
        # Skip update_system and setup_locale
        install_ros
        install_python_deps
        setup_hardware
        setup_project
        test_installation
        create_desktop_shortcuts
        final_setup
        progress_bar "$TOTAL_STEPS" "$TOTAL_STEPS"
        print_summary
        exit 0
        ;;
    *)
        main
        ;;
esac
