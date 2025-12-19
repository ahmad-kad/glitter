#!/bin/bash
# Install pykms from source (required for picamera2)
# pykms is built as part of kms++ using the -Dpykms=true option

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

KMSXX_REPO="https://github.com/tomba/kmsxx.git"
KMSXX_BUILD_DIR="/tmp/kmsxx_build"

echo "=========================================="
echo "Installing kms++ and pykms from source"
echo "=========================================="
echo ""

# Step 1: Install dependencies
echo -e "${YELLOW}[1/3] Installing build dependencies...${NC}"
sudo apt update
sudo apt install -y \
    build-essential \
    python3-dev \
    meson \
    ninja-build \
    git \
    libdrm-dev \
    libevdev-dev \
    libboost-dev \
    pkg-config

echo -e "${GREEN}✓${NC} Build dependencies installed"
echo ""

# Step 2: Build kms++ with pykms enabled
echo -e "${YELLOW}[2/3] Building kms++ and pykms from source...${NC}"
if [ -d "$KMSXX_BUILD_DIR" ]; then
    echo "Removing existing kms++ build directory..."
    rm -rf "$KMSXX_BUILD_DIR"
fi

echo "Cloning kms++ repository..."
git clone "$KMSXX_REPO" "$KMSXX_BUILD_DIR"
cd "$KMSXX_BUILD_DIR"

echo "Current branch: $(git branch --show-current)"
echo "Latest commit: $(git log -1 --oneline)"
echo ""

echo "Building kms++ with pykms Python bindings enabled..."
echo "Using: meson setup build --buildtype=release -Dpykms=enabled"
meson setup build --buildtype=release -Dpykms=enabled
ninja -C build
sudo ninja -C build install
sudo ldconfig  # Update library cache

echo -e "${GREEN}✓${NC} kms++ and pykms built and installed"
cd - > /dev/null
echo ""

# Step 3: Test installation
echo -e "${YELLOW}[3/3] Testing pykms installation...${NC}"
python3 << 'TESTEOF'
try:
    import pykms
    print("✓ pykms imported successfully")
    if hasattr(pykms, '__version__'):
        print(f"  Version: {pykms.__version__}")
except ImportError as e:
    print(f"✗ Failed to import pykms: {e}")
    print("\nTroubleshooting:")
    print("  1. Check if pykms was built: ls /tmp/kmsxx_build/build/py")
    print("  2. Check Python path: python3 -c 'import sys; print(sys.path)'")
    print("  3. May need to set PYTHONPATH or reinstall")
    exit(1)
TESTEOF

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}=========================================="
    echo "pykms is ready!"
    echo "==========================================${NC}"
    echo ""
    echo "Now test picamera2:"
    echo "  python3 -c 'from picamera2 import Picamera2; print(\"OK\")'"
else
    echo ""
    echo -e "${RED}pykms installation failed${NC}"
    echo "Check the build logs above for errors"
    exit 1
fi










