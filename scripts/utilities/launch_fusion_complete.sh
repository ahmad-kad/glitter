#!/bin/bash
# Launch Complete Fusion System: LiDAR + Camera + Fusion + RViz
# Usage: ./launch_fusion_complete.sh [camera_type]
# camera_type: usb (default), picamera, or none

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Get camera type from argument
CAMERA_TYPE=${1:-usb}

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Complete Fusion System Launcher${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Source ROS 2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo -e "${GREEN}✓${NC} ROS 2 Jazzy sourced"
else
    echo -e "${RED}✗${NC} ROS 2 not found at /opt/ros/jazzy"
    exit 1
fi

# Source ROS workspace if it exists
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo -e "${GREEN}✓${NC} ROS workspace sourced"
fi

# Check if LiDAR driver is available
if ! ros2 pkg list | grep -q "unitree_lidar_ros2\|unilidar_sdk"; then
    echo -e "${YELLOW}⚠${NC} LiDAR driver package not found"
    echo "   Make sure you've built the driver:"
    echo "   cd ~/ros2_ws && colcon build --packages-select unitree_lidar_ros2"
fi

# Function to cleanup on exit
cleanup() {
    echo ""
    echo -e "${YELLOW}Shutting down...${NC}"
    kill $LIDAR_PID $CAMERA_PID $FUSION_PID $RVIZ_PID $TF_PID $TF2_PID 2>/dev/null || true
    wait $LIDAR_PID $CAMERA_PID $FUSION_PID $RVIZ_PID $TF_PID $TF2_PID 2>/dev/null || true
    echo -e "${GREEN}All processes stopped${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Launch LiDAR Driver
echo ""
echo -e "${GREEN}[1/4] Launching LiDAR Driver...${NC}"
ros2 launch unitree_lidar_ros2 launch.py > /tmp/lidar.log 2>&1 &
LIDAR_PID=$!
echo "   LiDAR PID: $LIDAR_PID"
sleep 3

# Check if LiDAR is publishing
if timeout 5 ros2 topic list | grep -q "/unilidar/cloud"; then
    echo -e "${GREEN}✓${NC} LiDAR publishing to /unilidar/cloud"
else
    echo -e "${YELLOW}⚠${NC} LiDAR topic not found yet (may take a few seconds)"
fi

# Launch Camera Driver
echo ""
echo -e "${GREEN}[2/4] Launching Camera Driver...${NC}"
if [ "$CAMERA_TYPE" = "usb" ]; then
    echo "   Using USB camera"
    ros2 run usb_cam usb_cam_node_exe > /tmp/camera.log 2>&1 &
    CAMERA_PID=$!
elif [ "$CAMERA_TYPE" = "picamera" ]; then
    echo "   Using Pi Camera (ROS 2 camera_ros)"
    # Try ROS 2 camera_ros package first
        if ros2 pkg list 2>/dev/null | grep -q "camera_ros"; then
            ros2 run camera_ros camera_node > /tmp/camera.log 2>&1 &
            CAMERA_PID=$!
    else
        # Fallback to Python node
        echo -e "${YELLOW}⚠${NC} camera_ros not found, using Python node"
        cd "$PROJECT_DIR"
        if [ -f activate_env.sh ]; then
            source activate_env.sh
        fi
        python3 src/camera/pi_camera_node.py > /tmp/camera.log 2>&1 &
        CAMERA_PID=$!
    fi
elif [ "$CAMERA_TYPE" = "none" ]; then
    echo "   Skipping camera (LiDAR only mode)"
    CAMERA_PID=""
else
    echo -e "${RED}✗${NC} Unknown camera type: $CAMERA_TYPE"
    echo "   Use: usb, picamera, or none"
    kill $LIDAR_PID 2>/dev/null || true
    exit 1
fi

if [ -n "$CAMERA_PID" ]; then
    echo "   Camera PID: $CAMERA_PID"
    sleep 2
    
    # Check if camera is publishing
    if timeout 5 ros2 topic list | grep -q "/camera/image_raw"; then
        echo -e "${GREEN}✓${NC} Camera publishing to /camera/image_raw"
    else
        echo -e "${YELLOW}⚠${NC} Camera topic not found yet (may take a few seconds)"
    fi
fi

# Launch Fusion Node
echo ""
echo -e "${GREEN}[3/4] Launching Fusion Node...${NC}"
cd "$PROJECT_DIR"

# Activate Python environment if it exists
if [ -f activate_env.sh ]; then
    source activate_env.sh
    echo "   Python environment activated"
fi

# Check if fusion.py exists
if [ ! -f src/core/fusion.py ]; then
    echo -e "${RED}✗${NC} Fusion node not found at src/core/fusion.py"
    kill $LIDAR_PID $CAMERA_PID 2>/dev/null || true
    exit 1
fi

python3 src/core/fusion.py > /tmp/fusion.log 2>&1 &
FUSION_PID=$!
echo "   Fusion PID: $FUSION_PID"
sleep 3

# Check if fusion is publishing
if timeout 5 ros2 topic list | grep -q "/unilidar/colored_cloud"; then
    echo -e "${GREEN}✓${NC} Fusion publishing to /unilidar/colored_cloud"
else
    echo -e "${YELLOW}⚠${NC} Colored cloud topic not found yet"
    echo "   Check /tmp/fusion.log for errors"
fi

# Launch Static TF (for frame support)
echo ""
echo -e "${GREEN}[3.5/4] Launching Static TF...${NC}"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link > /tmp/tf.log 2>&1 &
TF_PID=$!
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link unilidar_lidar >> /tmp/tf.log 2>&1 &
TF2_PID=$!
echo "   TF PIDs: $TF_PID, $TF2_PID"
sleep 1

# Launch RViz
echo ""
echo -e "${GREEN}[4/4] Launching RViz...${NC}"
if [ -f "$PROJECT_DIR/config/l2_fusion.rviz" ]; then
    rviz2 -d "$PROJECT_DIR/config/l2_fusion.rviz" > /tmp/rviz.log 2>&1 &
    RVIZ_PID=$!
    echo "   RViz PID: $RVIZ_PID"
    echo -e "${GREEN}✓${NC} RViz launched with config"
else
    echo -e "${YELLOW}⚠${NC} Config file not found, launching RViz without config"
    rviz2 > /tmp/rviz.log 2>&1 &
    RVIZ_PID=$!
    echo "   RViz PID: $RVIZ_PID"
fi

# Summary
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}All systems launched!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Processes running:"
echo "  - LiDAR:    PID $LIDAR_PID"
[ -n "$CAMERA_PID" ] && echo "  - Camera:   PID $CAMERA_PID"
echo "  - Fusion:    PID $FUSION_PID"
echo "  - TF:       PIDs $TF_PID, $TF2_PID"
echo "  - RViz:     PID $RVIZ_PID"
echo ""
echo "Topics to check:"
echo "  - LiDAR:    /unilidar/cloud"
[ -n "$CAMERA_PID" ] && echo "  - Camera:   /camera/image_raw"
echo "  - Colored:  /unilidar/colored_cloud"
echo ""
echo -e "${YELLOW}In RViz:${NC}"
echo "  1. Enable 'Fused_PointCloud' display"
echo "  2. Set Fixed Frame to 'unilidar_lidar'"
echo "  3. Set Color Transformer to 'RGB8'"
echo "  4. Adjust point size to 5-10 pixels"
echo ""
echo -e "${YELLOW}Logs:${NC}"
echo "  - LiDAR:  /tmp/lidar.log"
[ -n "$CAMERA_PID" ] && echo "  - Camera: /tmp/camera.log"
echo "  - Fusion: /tmp/fusion.log"
echo "  - RViz:   /tmp/rviz.log"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop all processes${NC}"
echo ""

# Wait for user interrupt
wait










