#!/bin/bash
# Install picamera2 from Raspberry Pi fork (source build)
# This script requires sudo for system packages

# Don't exit on error for optional packages
set +e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

PICAMERA2_REPO="https://github.com/raspberrypi/picamera2.git"
BUILD_DIR="/tmp/picamera2_build"

echo "=========================================="
echo "Installing picamera2 from Raspberry Pi fork"
echo "=========================================="
echo ""

# Step 1: Install system dependencies
echo -e "${YELLOW}[1/4] Installing system dependencies...${NC}"
echo "This requires sudo - you'll be prompted for password"
echo ""

sudo apt update

# Required packages
REQUIRED_PACKAGES=(
    libcap-dev
    python3-dev
    python3-libcamera
    python3-pyqt5
    python3-numpy
    build-essential
    git
)

# Optional packages (may not be available on all systems)
OPTIONAL_PACKAGES=(
    python3-kms++
)

echo "Installing required packages..."
sudo apt install -y "${REQUIRED_PACKAGES[@]}"

echo ""
echo "Installing optional packages (may fail on some systems)..."
PYKMS_AVAILABLE=false
for pkg in "${OPTIONAL_PACKAGES[@]}"; do
    if sudo apt install -y "$pkg" 2>/dev/null; then
        echo "  ✓ $pkg installed"
        PYKMS_AVAILABLE=true
    else
        echo "  ⚠ $pkg not available - will build pykms from source"
    fi
done

# Build pykms from source if package not available
if [ "$PYKMS_AVAILABLE" = false ]; then
    echo ""
    echo -e "${YELLOW}Building pykms from source (required for picamera2)...${NC}"
    
    # Check if we need to install kms++ development libraries
    if ! dpkg -l | grep -q libkms; then
        echo "Installing kms++ development libraries..."
        sudo apt install -y \
            libkms++-dev \
            meson \
            ninja-build \
            2>/dev/null || echo "  ⚠ Some kms++ dependencies may be missing"
    fi
    
    PYKMS_BUILD_DIR="/tmp/pykms_build"
    if [ -d "$PYKMS_BUILD_DIR" ]; then
        rm -rf "$PYKMS_BUILD_DIR"
    fi
    
    echo "Cloning pykms repository..."
    if git clone https://github.com/raspberrypi/pykms.git "$PYKMS_BUILD_DIR" 2>/dev/null; then
        cd "$PYKMS_BUILD_DIR"
        echo "Building pykms..."
        if meson setup build 2>/dev/null && ninja -C build 2>/dev/null; then
            sudo ninja -C build install 2>/dev/null && echo "  ✓ pykms built and installed" || echo "  ⚠ pykms build completed but install may have issues"
        else
            echo "  ⚠ pykms build failed - picamera2 may work without it for basic features"
        fi
        cd - > /dev/null
    else
        echo "  ⚠ Failed to clone pykms - picamera2 may work without it for basic features"
    fi
fi

echo -e "${GREEN}✓${NC} System dependencies installed"
echo ""

# Re-enable strict error handling for critical steps
set -e

# Step 2: Install Python dependencies
echo -e "${YELLOW}[2/4] Installing Python dependencies...${NC}"
pip3 install --break-system-packages \
    v4l2-python3 \
    pyopengl \
    piexif \
    simplejpeg \
    PiDNG

echo -e "${GREEN}✓${NC} Python dependencies installed"
echo ""

# Step 3: Clone and build from Raspberry Pi fork
echo -e "${YELLOW}[3/4] Cloning Raspberry Pi picamera2 repository...${NC}"

# Clean up any existing build directory
if [ -d "$BUILD_DIR" ]; then
    echo "Removing existing build directory..."
    rm -rf "$BUILD_DIR"
fi

# Clone repository
echo "Cloning from: $PICAMERA2_REPO"
git clone "$PICAMERA2_REPO" "$BUILD_DIR"

cd "$BUILD_DIR"
echo "Current branch: $(git branch --show-current)"
echo "Latest commit: $(git log -1 --oneline)"

echo ""
echo -e "${BLUE}Building and installing picamera2 from source...${NC}"
pip3 install . --break-system-packages

echo -e "${GREEN}✓${NC} picamera2 built and installed from source"
echo ""

# Clean up build directory (optional - keep for debugging if needed)
# rm -rf "$BUILD_DIR"

# Step 4: Test installation
echo -e "${YELLOW}[4/4] Testing camera...${NC}"
python3 << 'TESTEOF'
import sys

# Try importing picamera2 - handle missing pykms gracefully
try:
    from picamera2 import Picamera2
    print("✓ picamera2 imported")
except ImportError as e:
    if "pykms" in str(e).lower():
        print("⚠ picamera2 import warning: pykms not available")
        print("  Attempting to continue - pykms is only needed for display features")
        try:
            # Try to import without pykms
            import os
            os.environ['PICAMERA2_SKIP_PYKMS'] = '1'
            from picamera2 import Picamera2
            print("✓ picamera2 imported (without pykms)")
        except Exception as e2:
            print(f"✗ Cannot import picamera2: {e2}")
            print("\nTo fix: Install pykms from source:")
            print("  git clone https://github.com/raspberrypi/pykms.git")
            print("  cd pykms && meson setup build && ninja -C build && sudo ninja -C build install")
            sys.exit(1)
    else:
        print(f"✗ Import error: {e}")
        sys.exit(1)

# Test camera functionality
try:
    cam = Picamera2()
    print("✓ Camera object created")
    
    config = cam.create_preview_configuration(main={"size": (640, 480)})
    cam.configure(config)
    print("✓ Camera configured")
    
    cam.start()
    print("✓ Camera started")
    
    frame = cam.capture_array()
    print(f"✓ Frame captured: {frame.shape}")
    
    cam.stop()
    print("✓ Camera stopped")
    print("\n✅ SUCCESS: Camera is working!")
except Exception as e:
    print(f"⚠ Camera test failed: {e}")
    print("Camera may not be connected or needs reboot")
    print("Error details:", str(e))
    sys.exit(1)
TESTEOF

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}=========================================="
    echo "Camera is ready!"
    echo "==========================================${NC}"
    echo ""
    echo "Start camera node:"
    echo "  ./start_camera.sh"
    echo ""
    echo "Or manually:"
    echo "  source /opt/ros/jazzy/setup.bash"
    echo "  source activate_env.sh"
    echo "  python3 src/camera/pi_camera_node.py"
else
    echo ""
    echo -e "${RED}Camera test failed${NC}"
    echo "Check camera connection and try rebooting"
fi
