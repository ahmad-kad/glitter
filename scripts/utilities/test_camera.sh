#!/bin/bash
# Comprehensive Camera Test Script

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "=========================================="
echo "Camera Test - Raspberry Pi"
echo "=========================================="
echo ""

# Test 1: Check video devices
echo -e "${BLUE}[1] Checking video devices...${NC}"
if ls /dev/video* 1>/dev/null 2>&1; then
    VIDEO_COUNT=$(ls -1 /dev/video* | wc -l)
    echo -e "  ${GREEN}✓${NC} Found $VIDEO_COUNT video devices"
    echo "  Devices:"
    ls -1 /dev/video* | head -10
    echo ""
    
    if command -v v4l2-ctl >/dev/null 2>&1; then
        echo "  Device details:"
        v4l2-ctl --list-devices 2>/dev/null | head -20
    else
        echo -e "  ${YELLOW}⚠${NC} v4l2-ctl not installed (install: sudo apt install v4l-utils)"
    fi
else
    echo -e "  ${RED}✗${NC} No video devices found"
fi
echo ""

# Test 2: Check ROS 2 camera packages
echo -e "${BLUE}[2] Checking ROS 2 camera packages...${NC}"
source /opt/ros/jazzy/setup.bash 2>/dev/null

if ros2 pkg list 2>/dev/null | grep -q "camera_ros"; then
    echo -e "  ${GREEN}✓${NC} ros-jazzy-camera-ros installed"
    CAMERA_ROS_AVAILABLE=true
else
    echo -e "  ${YELLOW}⚠${NC} ros-jazzy-camera-ros not installed"
    echo "    Install: sudo apt install ros-jazzy-camera-ros"
    CAMERA_ROS_AVAILABLE=false
fi

if ros2 pkg list 2>/dev/null | grep -q "usb_cam"; then
    echo -e "  ${GREEN}✓${NC} ros-jazzy-usb-cam installed"
    USB_CAM_AVAILABLE=true
else
    echo -e "  ${YELLOW}⚠${NC} ros-jazzy-usb-cam not installed"
    USB_CAM_AVAILABLE=false
fi
echo ""

# Test 3: Test OpenCV camera access
echo -e "${BLUE}[3] Testing OpenCV camera access...${NC}"
python3 << 'PYEOF'
import cv2
import sys

devices_to_try = [0, 10, 11, 12, 20, 21]
found_devices = []

for dev in devices_to_try:
    try:
        cap = cv2.VideoCapture(dev)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                h, w = frame.shape[:2]
                found_devices.append((dev, w, h))
            cap.release()
    except:
        pass

if found_devices:
    print("  ✓ OpenCV can access:")
    for dev, w, h in found_devices:
        print(f"    /dev/video{dev}: {w}x{h}")
else:
    print("  ✗ OpenCV cannot access any camera devices")
PYEOF
echo ""

# Test 4: Test ROS 2 camera_ros (if available)
if [ "$CAMERA_ROS_AVAILABLE" = true ]; then
    echo -e "${BLUE}[4] Testing ROS 2 camera_ros...${NC}"
    echo "  Launching camera_ros for 3 seconds..."
    
    timeout 3 ros2 run camera_ros camera_ros_node > /tmp/camera_ros_test.log 2>&1 &
    CAMERA_PID=$!
    sleep 2
    
    if ros2 topic list 2>/dev/null | grep -q "/camera/image_raw"; then
        RATE=$(timeout 2 ros2 topic hz /camera/image_raw 2>/dev/null | grep "average rate" | awk '{print $3}')
        if [ -n "$RATE" ]; then
            echo -e "  ${GREEN}✓${NC} camera_ros is publishing at ${RATE} Hz"
        else
            echo -e "  ${YELLOW}⚠${NC} camera_ros topic exists but no data yet"
        fi
    else
        echo -e "  ${RED}✗${NC} camera_ros not publishing"
        echo "  Check logs: tail /tmp/camera_ros_test.log"
    fi
    
    kill $CAMERA_PID 2>/dev/null || true
    wait $CAMERA_PID 2>/dev/null || true
else
    echo -e "${BLUE}[4] Skipping camera_ros test (not installed)${NC}"
fi
echo ""

# Test 5: Test Python camera node
echo -e "${BLUE}[5] Testing Python camera node...${NC}"
if [ -f ~/glitter/src/camera/pi_camera_node.py ]; then
    echo "  Node exists: src/camera/pi_camera_node.py"
    echo "  Test manually: python3 src/camera/pi_camera_node.py"
else
    echo -e "  ${YELLOW}⚠${NC} Python camera node not found"
fi
echo ""

# Test 6: Check camera permissions
echo -e "${BLUE}[6] Checking camera permissions...${NC}"
if groups | grep -q video; then
    echo -e "  ${GREEN}✓${NC} User is in video group"
else
    echo -e "  ${YELLOW}⚠${NC} User not in video group"
    echo "    Fix: sudo usermod -a -G video $USER"
    echo "    Then log out and back in"
fi

# Check device permissions
if ls /dev/video* 1>/dev/null 2>&1; then
    FIRST_DEV=$(ls -1 /dev/video* | head -1)
    if [ -r "$FIRST_DEV" ]; then
        echo -e "  ${GREEN}✓${NC} Can read $FIRST_DEV"
    else
        echo -e "  ${RED}✗${NC} Cannot read $FIRST_DEV (permission denied)"
    fi
fi
echo ""

# Summary
echo "=========================================="
echo "Summary & Recommendations"
echo "=========================================="
echo ""

if [ "$CAMERA_ROS_AVAILABLE" = true ]; then
    echo -e "${GREEN}Recommended: Use camera_ros${NC}"
    echo ""
    echo "Launch camera:"
    echo "  source /opt/ros/jazzy/setup.bash"
    echo "  ros2 run camera_ros camera_ros_node"
    echo ""
    echo "Check it's working:"
    echo "  ros2 topic hz /camera/image_raw"
    echo "  ros2 run rqt_image_view rqt_image_view /camera/image_raw"
elif [ "$USB_CAM_AVAILABLE" = true ]; then
    echo -e "${YELLOW}Alternative: Use usb_cam${NC}"
    echo ""
    echo "Launch camera:"
    echo "  source /opt/ros/jazzy/setup.bash"
    echo "  ros2 run usb_cam usb_cam_node_exe"
else
    echo -e "${RED}No ROS camera packages installed${NC}"
    echo ""
    echo "Install one of:"
    echo "  sudo apt install ros-jazzy-camera-ros  # For Pi Camera"
    echo "  sudo apt install ros-jazzy-usb-cam    # For USB camera"
fi

echo ""
echo "For fusion system:"
echo "  ./launch_fusion.sh picamera  # If camera_ros installed"
echo "  ./launch_fusion.sh usb       # If usb_cam installed"










