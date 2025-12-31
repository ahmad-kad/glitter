#!/bin/bash

# Integration test script for RealSense 435 + Unitree L2 setup
# Verifies that both sensors work together and can be visualized in RViz

set -e

echo "=== Testing RealSense 435 + Unitree L2 Integration ==="

# Source ROS 2
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test results
TESTS_PASSED=0
TESTS_FAILED=0

# Function to report test results
test_result() {
    local test_name="$1"
    local result="$2"
    local details="$3"

    if [ "$result" -eq 0 ]; then
        echo -e "${GREEN}✓ PASS${NC}: $test_name"
        ((TESTS_PASSED++))
    else
        echo -e "${RED}✗ FAIL${NC}: $test_name"
        if [ -n "$details" ]; then
            echo -e "${RED}  Details: $details${NC}"
        fi
        ((TESTS_FAILED++))
    fi
}

echo "Running pre-flight checks..."
echo ""

# Test 1: Check hardware connectivity
echo "1. Hardware Connectivity Tests"

# Check RealSense camera
if lsusb | grep -q "8086:0b07"; then
    test_result "RealSense 435 USB connection" 0
else
    test_result "RealSense 435 USB connection" 1 "Camera not detected (USB ID: 8086:0b07)"
fi

# Check L2 LiDAR connectivity
if ping -c 1 -W 2 192.168.1.150 >/dev/null 2>&1 || ping -c 1 -W 2 192.168.1.62 >/dev/null 2>&1; then
    test_result "Unitree L2 LiDAR network connection" 0
else
    test_result "Unitree L2 LiDAR network connection" 1 "Cannot reach LiDAR at expected IPs (192.168.1.150/62)"
fi

# Test 2: Check ROS2 installation
echo ""
echo "2. ROS2 Installation Tests"

if ros2 --version >/dev/null 2>&1; then
    test_result "ROS2 Jazzy installation" 0
else
    test_result "ROS2 Jazzy installation" 1 "ROS2 not found in PATH"
fi

# Check required packages
if ros2 pkg list | grep -q realsense2_camera; then
    test_result "realsense2_camera package" 0
else
    test_result "realsense2_camera package" 1 "Package not installed. Run: sudo apt install ros-jazzy-realsense2-camera"
fi

if ros2 pkg list | grep -q unitree_lidar; then
    test_result "Unitree L2 ROS2 driver" 0
else
    test_result "Unitree L2 ROS2 driver" 1 "Driver not found. Check ~/ros2_ws installation"
fi

echo ""
echo "3. Sensor Driver Tests"
echo "Note: These tests require the sensors to be physically connected and powered on"

# Test RealSense driver (quick test)
echo "Testing RealSense driver..."
timeout 10 ros2 launch realsense2_camera rs_launch.py enable_color:=false enable_depth:=true enable_pointcloud:=false >/dev/null 2>&1 &
REALSENSE_PID=$!
sleep 5

if ros2 topic list 2>/dev/null | grep -q "/camera/depth"; then
    test_result "RealSense driver startup" 0
else
    test_result "RealSense driver startup" 1 "No camera topics detected"
fi

kill $REALSENSE_PID 2>/dev/null || true
sleep 2

# Test L2 LiDAR driver (quick test)
echo "Testing L2 LiDAR driver..."
timeout 10 ros2 launch unitree_lidar_ros2 launch.py >/dev/null 2>&1 &
LIDAR_PID=$!
sleep 5

if ros2 topic list 2>/dev/null | grep -q "/unilidar"; then
    test_result "L2 LiDAR driver startup" 0
else
    test_result "L2 LiDAR driver startup" 1 "No LiDAR topics detected"
fi

kill $LIDAR_PID 2>/dev/null || true
sleep 2

echo ""
echo "4. Configuration File Tests"

# Check configuration files exist
if [ -f "config/realsense/realsense_launch.py" ]; then
    test_result "RealSense configuration files" 0
else
    test_result "RealSense configuration files" 1 "Missing config/realsense/realsense_launch.py"
fi

if [ -f "config/tf/static_transforms.launch.py" ]; then
    test_result "TF transform configuration" 0
else
    test_result "TF transform configuration" 1 "Missing config/tf/static_transforms.launch.py"
fi

if [ -f "config/realsense_l2_combined.rviz" ]; then
    test_result "RViz configuration" 0
else
    test_result "RViz configuration" 1 "Missing config/realsense_l2_combined.rviz"
fi

echo ""
echo "5. Launch Script Tests"

# Check launch scripts exist and are executable
if [ -x "scripts/launch_realsense_l2.sh" ]; then
    test_result "Combined launch script" 0
else
    test_result "Combined launch script" 1 "Missing or not executable: scripts/launch_realsense_l2.sh"
fi

if [ -x "scripts/launch_realsense_l2_manual.sh" ]; then
    test_result "Manual launch script" 0
else
    test_result "Manual launch script" 1 "Missing or not executable: scripts/launch_realsense_l2_manual.sh"
fi

echo ""
echo "=== Integration Test Summary ==="
echo "Tests Passed: $TESTS_PASSED"
echo "Tests Failed: $TESTS_FAILED"

if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "${GREEN}✓ All tests passed! Ready to launch RealSense + L2 setup${NC}"
    echo ""
    echo "Next steps:"
    echo "1. Ensure both devices are connected and powered on"
    echo "2. Run: ./scripts/launch_realsense_l2_manual.sh"
    echo "3. Or run: ./scripts/launch_realsense_l2.sh"
    echo "4. Check RViz for aligned point clouds"
else
    echo -e "${YELLOW}⚠ Some tests failed. Please resolve issues before proceeding.${NC}"
    echo ""
    echo "Common fixes:"
    echo "- Install RealSense: ./scripts/install_realsense.sh"
    echo "- Configure RealSense: ./scripts/configure_realsense.sh"
    echo "- Setup TF frames: ./scripts/setup_tf_frames.sh"
fi

echo ""
echo "For detailed sensor verification:"
echo "- L2 LiDAR: ./scripts/utilities/check_lidar_data.sh"
echo "- RealSense: realsense-viewer"
