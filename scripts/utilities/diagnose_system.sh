#!/bin/bash
# System Diagnostic Script

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "=========================================="
echo "System Diagnostic Report"
echo "=========================================="
echo ""

echo -e "${GREEN}[1] USB Devices:${NC}"
lsusb | grep -v "Linux Foundation" | grep -v "root hub" || echo "  No USB devices found"
echo ""

echo -e "${GREEN}[2] Video Devices:${NC}"
if ls /dev/video* 1>/dev/null 2>&1; then
    echo "  Found: $(ls -1 /dev/video* | wc -l) video devices"
    if command -v v4l2-ctl >/dev/null 2>&1; then
        v4l2-ctl --list-devices 2>/dev/null | head -10
    fi
else
    echo -e "  ${RED}No video devices${NC}"
fi
echo ""

echo -e "${GREEN}[3] Camera Software:${NC}"
if command -v libcamera-hello >/dev/null 2>&1; then
    echo -e "  ${GREEN}✓${NC} libcamera installed"
else
    echo -e "  ${YELLOW}⚠${NC} libcamera-hello not installed"
fi

if source /opt/ros/jazzy/setup.bash 2>/dev/null && ros2 pkg list 2>/dev/null | grep -q "usb_cam"; then
    echo -e "  ${GREEN}✓${NC} usb_cam ROS package installed"
else
    echo -e "  ${YELLOW}⚠${NC} usb_cam ROS package not installed"
fi
echo ""

echo -e "${GREEN}[4] Network:${NC}"
echo "  eth0: $(ip link show eth0 2>/dev/null | grep -oP 'state \K\S+' || echo 'unknown')"
if ip link show eth0 2>/dev/null | grep -q "state UP"; then
    ip addr show eth0 | grep "inet " || echo "    No IP"
else
    echo -e "    ${RED}eth0 is DOWN - LiDAR cannot connect${NC}"
fi

echo "  wlan0: $(ip link show wlan0 2>/dev/null | grep -oP 'state \K\S+' || echo 'unknown')"
ip addr show wlan0 2>/dev/null | grep "inet " || echo "    No IP"

if ! ip addr show | grep -q "192.168.1"; then
    echo -e "  ${RED}✗ No 192.168.1.x network - LiDAR needs this${NC}"
fi

if ping -c 1 -W 1 192.168.1.62 >/dev/null 2>&1; then
    echo -e "  ${GREEN}✓${NC} LiDAR reachable at 192.168.1.62"
elif ping -c 1 -W 1 192.168.1.150 >/dev/null 2>&1; then
    echo -e "  ${GREEN}✓${NC} LiDAR reachable at 192.168.1.150"
else
    echo -e "  ${RED}✗${NC} LiDAR not reachable"
fi
echo ""

echo -e "${GREEN}[5] ROS 2:${NC}"
if [ -f /opt/ros/jazzy/setup.bash ]; then
    echo -e "  ${GREEN}✓${NC} ROS 2 Jazzy installed"
    source /opt/ros/jazzy/setup.bash 2>/dev/null
    if ros2 topic list 2>/dev/null | grep -q "unilidar\|camera"; then
        echo "  Active topics:"
        ros2 topic list 2>/dev/null | grep -E "unilidar|camera"
    fi
fi
echo ""

echo -e "${GREEN}[6] Running Processes:${NC}"
if pgrep -f "lidar|camera|fusion|rviz" >/dev/null; then
    ps aux | grep -E "lidar|camera|fusion|rviz" | grep -v grep
else
    echo -e "  ${YELLOW}No processes running${NC}"
fi










