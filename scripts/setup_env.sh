#!/bin/bash
# Project Environment Setup Script
# Creates virtual environment and installs dependencies

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
VENV_DIR="$PROJECT_DIR/venv"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check Python version
check_python() {
    log_info "Checking Python version..."
    if ! command -v python3 &> /dev/null; then
        log_error "Python 3 not found. Please install Python 3.10+"
        exit 1
    fi
    
    PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
    log_info "Found Python $PYTHON_VERSION"
    
    # Check if version is >= 3.10
    MAJOR=$(echo $PYTHON_VERSION | cut -d'.' -f1)
    MINOR=$(echo $PYTHON_VERSION | cut -d'.' -f2)
    
    if [ "$MAJOR" -lt 3 ] || ([ "$MAJOR" -eq 3 ] && [ "$MINOR" -lt 10 ]); then
        log_error "Python 3.10+ required. Found $PYTHON_VERSION"
        exit 1
    fi
    
    # Check if venv module is available
    if ! python3 -m venv --help &> /dev/null; then
        log_error "python3-venv package not installed"
        log_error "Please install it with: sudo apt install python3-venv"
        log_error "Or for Python 3.12: sudo apt install python3.12-venv"
        exit 1
    fi
}

# Create virtual environment
create_venv() {
    log_info "Creating virtual environment..."
    
    if [ -d "$VENV_DIR" ]; then
        log_warn "Virtual environment already exists at $VENV_DIR"
        read -p "Remove and recreate? (y/N): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$VENV_DIR"
            log_info "Removed existing virtual environment"
        else
            log_info "Using existing virtual environment"
            return
        fi
    fi
    
    python3 -m venv "$VENV_DIR"
    log_info "Virtual environment created at $VENV_DIR"
}

# Install dependencies
install_dependencies() {
    log_info "Installing Python dependencies..."
    
    # Activate virtual environment
    source "$VENV_DIR/bin/activate"
    
    # Upgrade pip
    log_info "Upgrading pip..."
    pip install --upgrade pip setuptools wheel
    
    # Install requirements
    if [ -f "$PROJECT_DIR/requirements.txt" ]; then
        log_info "Installing from requirements.txt..."
        pip install -r "$PROJECT_DIR/requirements.txt"
    else
        log_error "requirements.txt not found!"
        exit 1
    fi
    
    # Install ROS 2 Python packages if ROS is available
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        log_info "ROS 2 detected. Installing ROS Python packages..."
        # Note: ROS packages are typically installed via apt, but we can check
        python3 -c "import rclpy" 2>/dev/null && log_info "rclpy available" || log_warn "rclpy not available (install ROS 2 packages via apt)"
    else
        log_warn "ROS 2 not detected. Install ROS 2 Jazzy for full functionality."
    fi
    
    log_info "Dependencies installed successfully"
}

# Verify installation
verify_installation() {
    log_info "Verifying installation..."
    
    source "$VENV_DIR/bin/activate"
    
    # Test core imports
    log_info "Testing core packages..."
    python3 << EOF
import sys
errors = []

try:
    import numpy
    print("  ✓ numpy")
except ImportError as e:
    errors.append(f"numpy: {e}")

try:
    import cv2
    print("  ✓ opencv-python")
except ImportError as e:
    errors.append(f"opencv-python: {e}")

try:
    import scipy
    print("  ✓ scipy")
except ImportError as e:
    errors.append(f"scipy: {e}")

try:
    import transforms3d
    print("  ✓ transforms3d")
except ImportError as e:
    errors.append(f"transforms3d: {e}")

try:
    import psutil
    print("  ✓ psutil")
except ImportError as e:
    errors.append(f"psutil: {e}")

try:
    import lz4
    print("  ✓ lz4")
except ImportError as e:
    errors.append(f"lz4: {e}")

if errors:
    print("\n✗ Missing packages:")
    for err in errors:
        print(f"  - {err}")
    sys.exit(1)
else:
    print("\n✓ All core packages installed successfully")
EOF

    if [ $? -eq 0 ]; then
        log_info "Installation verification complete"
    else
        log_error "Installation verification failed"
        exit 1
    fi
}

# Main execution
main() {
    log_info "Starting project environment setup..."
    log_info "Project directory: $PROJECT_DIR"
    
    cd "$PROJECT_DIR"
    
    check_python
    create_venv
    install_dependencies
    verify_installation
    
    log_info ""
    log_info "=========================================="
    log_info "Environment setup complete!"
    log_info "=========================================="
    log_info ""
    log_info "To activate the virtual environment:"
    log_info "  source $VENV_DIR/bin/activate"
    log_info ""
    log_info "Or use the activation script:"
    log_info "  source activate_env.sh"
    log_info ""
    log_info "To test LiDAR:"
    log_info "  source activate_env.sh"
    log_info "  python3 tests/hardware_tests/test_lidar.py"
    log_info ""
}

# Run main function
main "$@"

