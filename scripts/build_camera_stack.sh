#!/bin/bash
#
# Build libcamera and rpicam-apps from source for Raspberry Pi 5
# This script installs dependencies, builds, and installs the camera stack
#
# Usage: ./scripts/build_camera_stack.sh
#
# Estimated time: 30-60 minutes
# Disk space required: ~850 MB
#

set -e  # Exit on error
set -u  # Exit on undefined variable

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
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

# Check if running on Raspberry Pi
check_raspberry_pi() {
    log_info "Checking system..."
    if ! grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null && ! uname -r | grep -q "raspi"; then
        log_warning "This doesn't appear to be a Raspberry Pi"
        log_warning "Continuing anyway, but build may fail on non-Pi hardware"
    fi
}

# Step 1: Install build dependencies
install_dependencies() {
    log_info "=========================================="
    log_info "Step 1/6: Installing Build Dependencies"
    log_info "=========================================="
    log_info "This will install ~500MB of packages..."
    echo ""
    
    log_info "Updating package lists..."
    sudo apt update
    
    log_info "Installing core build tools..."
    sudo apt install -y \
        git \
        cmake \
        meson \
        ninja-build \
        pkg-config
    
    log_info "Installing libcamera dependencies..."
    sudo apt install -y \
        libboost-dev \
        libgnutls28-dev \
        openssl \
        libtiff5-dev \
        pybind11-dev \
        qtbase5-dev \
        libqt5core5a \
        libqt5gui5 \
        libqt5widgets5 \
        libyaml-dev \
        python3-yaml \
        python3-ply \
        python3-jinja2
    
    log_info "Installing rpicam-apps dependencies..."
    sudo apt install -y \
        libboost-program-options-dev \
        libdrm-dev \
        libexif-dev \
        libjpeg-dev \
        libpng-dev \
        libavcodec-dev \
        libavdevice-dev \
        libavformat-dev \
        libswresample-dev \
        libepoxy-dev
    
    log_success "Dependencies installed!"
    echo ""
}

# Step 2: Build libcamera
build_libcamera() {
    log_info "=========================================="
    log_info "Step 2/6: Building libcamera"
    log_info "=========================================="
    log_info "This will take 15-25 minutes..."
    echo ""
    
    # Create build directory
    BUILD_DIR="$HOME/builds"
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    
    # Clone or update libcamera
    if [ -d "libcamera" ]; then
        log_info "Updating existing libcamera repository..."
        cd libcamera
        git pull
    else
        log_info "Cloning libcamera repository..."
        git clone https://github.com/raspberrypi/libcamera.git
        cd libcamera
    fi
    
    # Clean previous build if exists
    if [ -d "build" ]; then
        log_info "Cleaning previous build..."
        rm -rf build
    fi
    
    log_info "Configuring libcamera build..."
    log_info "  - Build type: release (optimized)"
    log_info "  - Pipelines: rpi/vc4 (Pi 4), rpi/pisp (Pi 5)"
    log_info "  - V4L2: enabled"
    log_info "  - Tests: disabled (faster build)"
    
    meson setup build \
        --buildtype=release \
        -Dpipelines=rpi/vc4,rpi/pisp \
        -Dipas=rpi/vc4,rpi/pisp \
        -Dv4l2=true \
        -Dgstreamer=disabled \
        -Dtest=false
    
    log_info "Building libcamera (using all CPU cores)..."
    log_info "Watch for compilation progress [1/N] ... [N/N]"
    echo ""
    
    ninja -C build
    
    log_info "Installing libcamera..."
    sudo ninja -C build install
    
    log_success "libcamera built and installed!"
    echo ""
}

# Step 3: Build rpicam-apps
build_rpicam_apps() {
    log_info "=========================================="
    log_info "Step 3/6: Building rpicam-apps"
    log_info "=========================================="
    log_info "This will take 10-15 minutes..."
    echo ""
    
    BUILD_DIR="$HOME/builds"
    cd "$BUILD_DIR"
    
    # Clone or update rpicam-apps
    if [ -d "rpicam-apps" ]; then
        log_info "Updating existing rpicam-apps repository..."
        cd rpicam-apps
        git pull
    else
        log_info "Cloning rpicam-apps repository..."
        git clone https://github.com/raspberrypi/rpicam-apps.git
        cd rpicam-apps
    fi
    
    # Clean previous build if exists
    if [ -d "build" ]; then
        log_info "Cleaning previous build..."
        rm -rf build
    fi
    
    log_info "Configuring rpicam-apps build..."
    log_info "  - DRM/EGL: enabled (display output)"
    log_info "  - Qt: enabled (GUI apps)"
    log_info "  - OpenCV: disabled (not needed)"
    log_info "  - TensorFlow Lite: disabled (not needed)"
    
    meson setup build \
        --buildtype=release \
        -Denable_drm=enabled \
        -Denable_egl=enabled \
        -Denable_qt=enabled \
        -Denable_opencv=disabled \
        -Denable_tflite=disabled
    
    log_info "Building rpicam-apps..."
    echo ""
    
    ninja -C build
    
    log_info "Installing rpicam-apps..."
    sudo ninja -C build install
    
    log_success "rpicam-apps built and installed!"
    log_info "Installed commands:"
    log_info "  - libcamera-hello (test camera)"
    log_info "  - libcamera-vid (record video)"
    log_info "  - libcamera-still (capture images)"
    log_info "  - libcamera-jpeg (fast JPEG capture)"
    log_info "  - libcamera-raw (capture raw Bayer data)"
    echo ""
}

# Step 4: Configure library paths
configure_library_paths() {
    log_info "=========================================="
    log_info "Step 4/6: Configuring Library Paths"
    log_info "=========================================="
    echo ""
    
    log_info "Updating library cache..."
    sudo ldconfig
    
    log_info "Verifying libcamera libraries..."
    if ldconfig -p | grep -q libcamera; then
        log_success "libcamera libraries found in cache"
        ldconfig -p | grep libcamera | head -3
    else
        log_error "libcamera libraries NOT found!"
        log_error "You may need to add /usr/local/lib to LD_LIBRARY_PATH"
    fi
    
    # Add to bashrc if not already present
    if ! grep -q "LD_LIBRARY_PATH.*usr/local/lib" ~/.bashrc; then
        log_info "Adding library path to ~/.bashrc..."
        echo '' >> ~/.bashrc
        echo '# libcamera library path' >> ~/.bashrc
        echo 'export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH' >> ~/.bashrc
        log_info "Added to ~/.bashrc (will take effect in new terminals)"
    else
        log_info "Library path already in ~/.bashrc"
    fi
    
    # Export for current session
    export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
    
    log_success "Library paths configured!"
    echo ""
}

# Step 5: Test installation
test_installation() {
    log_info "=========================================="
    log_info "Step 5/6: Testing Installation"
    log_info "=========================================="
    echo ""
    
    # Export library path for this test
    export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
    
    log_info "Checking libcamera-hello command..."
    if command -v libcamera-hello &> /dev/null; then
        log_success "libcamera-hello found at: $(which libcamera-hello)"
        
        log_info "Checking version..."
        libcamera-hello --version || true
    else
        log_error "libcamera-hello not found in PATH!"
        log_error "Installation may have failed"
        return 1
    fi
    
    log_info "Listing available cameras..."
    if libcamera-hello --list-cameras 2>&1 | grep -q "Available cameras"; then
        log_success "Camera detected!"
        libcamera-hello --list-cameras
    else
        log_warning "No cameras detected or error occurred"
        log_warning "Output:"
        libcamera-hello --list-cameras 2>&1 || true
        log_warning ""
        log_warning "This might be okay if:"
        log_warning "  1. Camera is not connected yet"
        log_warning "  2. Camera needs a reboot to be detected"
        log_warning "  3. Camera ribbon cable is loose"
    fi
    
    echo ""
}

# Step 6: Update picamera2
update_picamera2() {
    log_info "=========================================="
    log_info "Step 6/6: Updating picamera2"
    log_info "=========================================="
    echo ""
    
    VENV_PATH="/home/ahmad/glitter/venv"
    
    if [ -d "$VENV_PATH" ]; then
        log_info "Activating virtual environment..."
        source "$VENV_PATH/bin/activate"
        
        log_info "Upgrading picamera2..."
        pip3 install --upgrade picamera2
        
        log_info "Testing picamera2 import..."
        if python3 -c "from picamera2 import Picamera2; print('picamera2 imported successfully')" 2>&1; then
            log_success "picamera2 working!"
        else
            log_warning "picamera2 import failed - may need reinstall"
        fi
        
        deactivate
    else
        log_warning "Virtual environment not found at $VENV_PATH"
        log_warning "Skipping picamera2 update"
        log_info "You can update manually later with:"
        log_info "  source venv/bin/activate && pip3 install --upgrade picamera2"
    fi
    
    echo ""
}

# Print summary
print_summary() {
    log_info "=========================================="
    log_success "Build Complete!"
    log_info "=========================================="
    echo ""
    
    log_info "What was built:"
    log_info "  ✓ libcamera (core library)"
    log_info "  ✓ rpicam-apps (command-line tools)"
    log_info "  ✓ picamera2 (Python bindings - updated)"
    echo ""
    
    log_info "Test your camera:"
    log_info "  $ libcamera-hello --list-cameras"
    log_info "  $ libcamera-hello --timeout 5000  # 5-second preview"
    echo ""
    
    log_info "If camera not detected:"
    log_info "  1. Check camera ribbon cable connection"
    log_info "  2. Try: sudo reboot"
    log_info "  3. After reboot: libcamera-hello --list-cameras"
    echo ""
    
    log_info "Next steps:"
    log_info "  1. Test camera with: libcamera-hello"
    log_info "  2. Test ROS node: source activate_env.sh && python3 src/camera/pi_camera_node.py"
    log_info "  3. Run fusion: ./launch_fusion.sh"
    echo ""
    
    log_info "Build artifacts saved in: ~/builds/"
    log_info "Documentation: docs/build_libcamera_guide.md"
    echo ""
}

# Main execution
main() {
    log_info "=========================================="
    log_info "libcamera + rpicam-apps Build Script"
    log_info "=========================================="
    log_info "This will build and install camera stack from source"
    log_info "Estimated time: 30-60 minutes"
    log_info "Disk space: ~850 MB"
    echo ""
    
    # Confirm before starting
    read -p "Continue? (y/N) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        log_info "Build cancelled"
        exit 0
    fi
    
    # Record start time
    START_TIME=$(date +%s)
    
    # Run build steps
    check_raspberry_pi
    install_dependencies
    build_libcamera
    build_rpicam_apps
    configure_library_paths
    test_installation
    update_picamera2
    
    # Calculate elapsed time
    END_TIME=$(date +%s)
    ELAPSED=$((END_TIME - START_TIME))
    MINUTES=$((ELAPSED / 60))
    SECONDS=$((ELAPSED % 60))
    
    print_summary
    
    log_success "Total build time: ${MINUTES}m ${SECONDS}s"
    echo ""
}

# Run main function
main "$@"



