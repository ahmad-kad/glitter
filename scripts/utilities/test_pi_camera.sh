#!/bin/bash
# Test Raspberry Pi Camera (CSI port)

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "=========================================="
echo "Raspberry Pi Camera Test"
echo "=========================================="
echo ""

# Test 1: Check libcamera installation
echo -e "${GREEN}[1] Checking libcamera installation...${NC}"
if command -v libcamera-hello >/dev/null 2>&1; then
    echo -e "  ${GREEN}✓${NC} libcamera-hello installed"
    LIB_VERSION=$(libcamera-hello --version 2>&1 | head -1)
    echo "  Version: $LIB_VERSION"
else
    echo -e "  ${RED}✗${NC} libcamera-hello not found"
    echo "  Install: sudo apt install libcamera-apps"
    echo ""
    echo "  After installing, run this script again"
    exit 1
fi
echo ""

# Test 2: List available cameras
echo -e "${GREEN}[2] Listing available cameras...${NC}"
libcamera-hello --list-cameras 2>&1
echo ""

# Test 3: Test camera preview
echo -e "${GREEN}[3] Testing camera preview (2 seconds)...${NC}"
echo "  You should see a camera preview window"
timeout 3 libcamera-hello -t 2000 2>&1 | head -10
if [ $? -eq 0 ]; then
    echo -e "  ${GREEN}✓${NC} Camera preview works"
else
    echo -e "  ${RED}✗${NC} Camera preview failed"
fi
echo ""

# Test 4: Capture test image
echo -e "${GREEN}[4] Capturing test image...${NC}"
libcamera-jpeg -o /tmp/pi_camera_test.jpg --width 1280 --height 720 2>&1
if [ -f /tmp/pi_camera_test.jpg ]; then
    SIZE=$(ls -lh /tmp/pi_camera_test.jpg | awk '{print $5}')
    echo -e "  ${GREEN}✓${NC} Image captured: /tmp/pi_camera_test.jpg ($SIZE)"
    echo "  View with: eog /tmp/pi_camera_test.jpg"
else
    echo -e "  ${RED}✗${NC} Failed to capture image"
fi
echo ""

# Test 5: Check video devices
echo -e "${GREEN}[5] Checking video devices...${NC}"
if command -v v4l2-ctl >/dev/null 2>&1; then
    echo "  Video devices:"
    v4l2-ctl --list-devices 2>/dev/null | head -15
else
    echo "  v4l2-ctl not available"
fi
echo ""

# Summary
echo "=========================================="
echo "Summary"
echo "=========================================="
echo ""

if [ -f /tmp/pi_camera_test.jpg ]; then
    echo -e "${GREEN}✓ Camera hardware is working!${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Use the Pi Camera ROS 2 node:"
    echo "     python3 src/camera/pi_camera_node.py"
    echo "  2. Or update launch script to use Pi Camera"
else
    echo -e "${RED}✗ Camera test failed${NC}"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Install libcamera: sudo apt install libcamera-apps"
    echo "  2. Check camera is connected to CSI port"
    echo "  3. Check camera ribbon cable is secure"
fi










