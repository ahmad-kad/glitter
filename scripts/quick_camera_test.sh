#!/bin/bash
#
# Quick camera test script
# Tests if libcamera and camera hardware are working
#
# Usage: ./scripts/quick_camera_test.sh
#

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "=========================================="
echo "Quick Camera Test"
echo "=========================================="
echo ""

# Export library path
export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH

# Check if libcamera-hello exists
if ! command -v libcamera-hello &> /dev/null; then
    echo -e "${RED}✗ libcamera-hello not found${NC}"
    echo "  Run: ./scripts/build_camera_stack.sh"
    exit 1
fi

echo -e "${GREEN}✓ libcamera-hello found${NC}"

# Check version
echo ""
echo "Version info:"
libcamera-hello --version

# List cameras
echo ""
echo "Detecting cameras..."
echo ""

if libcamera-hello --list-cameras 2>&1 | grep -q "Available cameras"; then
    echo -e "${GREEN}✓ Camera detected!${NC}"
    echo ""
    libcamera-hello --list-cameras
    echo ""
    echo "=========================================="
    echo "Camera is working!"
    echo "=========================================="
    echo ""
    echo "Test preview:"
    echo "  $ libcamera-hello --timeout 5000"
    echo ""
    echo "Capture image:"
    echo "  $ libcamera-still -o test.jpg"
    echo ""
    echo "Record video:"
    echo "  $ libcamera-vid -t 10000 -o test.h264"
    echo ""
else
    echo -e "${RED}✗ No cameras detected${NC}"
    echo ""
    libcamera-hello --list-cameras 2>&1 || true
    echo ""
    echo "Troubleshooting:"
    echo "  1. Check camera ribbon cable is connected"
    echo "  2. Make sure cable is in correct orientation"
    echo "  3. Try: sudo reboot"
    echo "  4. After reboot, run this test again"
    echo ""
    exit 1
fi



