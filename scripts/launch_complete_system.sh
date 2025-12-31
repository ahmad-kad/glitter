#!/bin/bash
# Complete launch script: sensors + fusion + RViz

set -e

echo "=========================================="
echo "Complete LiDAR-Camera Fusion System"
echo "=========================================="

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Source ROS 2
source /opt/ros/jazzy/setup.bash
source ~/glitter/install/setup.bash

# Set Python path
export PYTHONPATH="$HOME/glitter/glitter:$PYTHONPATH"

# Function to cleanup on exit
cleanup() {
    echo ""
    echo -e "${YELLOW}Shutting down all processes...${NC}"
    kill $LIDAR_PID $CAMERA_PID $TF_PID $FUSION_PID $RVIZ_PID 2>/dev/null || true
    wait $LIDAR_PID $CAMERA_PID $TF_PID $FUSION_PID $RVIZ_PID 2>/dev/null || true
    echo -e "${GREEN}All processes stopped${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Step 1: Launch LiDAR
echo ""
echo -e "${GREEN}[1/5] Launching Unitree L2 LiDAR...${NC}"
ros2 launch unitree_lidar_ros2 launch.py > /tmp/lidar.log 2>&1 &
LIDAR_PID=$!
echo "LiDAR PID: $LIDAR_PID"
sleep 3

# Check LiDAR
if ros2 topic list 2>/dev/null | grep -q "/unilidar/cloud"; then
    echo -e "${GREEN}✓${NC} LiDAR publishing to /unilidar/cloud"
else
    echo -e "${YELLOW}⚠${NC} LiDAR topic not found yet"
fi

# Step 2: Launch Camera
echo ""
echo -e "${GREEN}[2/5] Launching RealSense Camera...${NC}"
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    enable_depth:=true \
    enable_pointcloud:=true \
    align_depth:=true \
    color_width:=640 \
    color_height:=480 \
    color_fps:=30.0 \
    depth_width:=640 \
    depth_height:=480 \
    depth_fps:=30.0 \
    pointcloud_texture_index:=0 \
    ordered_pc:=false \
    enable_sync:=true \
    diagnostics_period:=0.0 \
    base_frame_id:=camera_link > /tmp/camera.log 2>&1 &
CAMERA_PID=$!
echo "Camera PID: $CAMERA_PID"
sleep 3

# Check Camera
if ros2 topic list 2>/dev/null | grep -q "/camera/camera/color/image_raw"; then
    echo -e "${GREEN}✓${NC} Camera publishing to /camera/camera/color/image_raw"
else
    echo -e "${YELLOW}⚠${NC} Camera topic not found yet"
fi

# Step 3: Launch TF transforms
echo ""
echo -e "${GREEN}[3/5] Launching TF Transforms...${NC}"
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 unilidar_lidar camera_link > /tmp/tf.log 2>&1 &
TF_PID=$!
echo "TF PID: $TF_PID"
sleep 1

# Step 4: Launch Fusion Node
echo ""
echo -e "${GREEN}[4/5] Launching Fusion Node...${NC}"
/usr/bin/python3 ~/glitter/glitter/core/fusion.py \
    --ros-args \
    --param camera_matrix:="[615.0, 0.0, 310.0, 0.0, 615.0, 240.0, 0.0, 0.0, 1.0]" \
    --param extrinsic_trans:="[0.0, 0.0, 0.1]" \
    --param extrinsic_rot:="[0.0, 0.0, 0.0]" \
    --param lidar_topic:="/unilidar/cloud" \
    --param geometric_prioritization:=false \
    --param max_points_per_frame:=0 \
    --param geometric_weight:=0.7 > /tmp/fusion.log 2>&1 &
FUSION_PID=$!
echo "Fusion PID: $FUSION_PID"
sleep 2

# Check Fusion
if ros2 topic list 2>/dev/null | grep -q "/unilidar/colored_cloud"; then
    echo -e "${GREEN}✓${NC} Fusion publishing to /unilidar/colored_cloud"
else
    echo -e "${YELLOW}⚠${NC} Fusion topic not found yet"
fi

# Step 5: Launch RViz
echo ""
echo -e "${GREEN}[5/5] Launching RViz...${NC}"
rviz2 -d ~/glitter/config/realsense_l2_combined.rviz > /tmp/rviz.log 2>&1 &
RVIZ_PID=$!
echo "RViz PID: $RVIZ_PID"

# Summary
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}System Launch Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Running Processes:"
echo "  - LiDAR:     PID $LIDAR_PID"
echo "  - Camera:    PID $CAMERA_PID"
echo "  - TF:        PID $TF_PID"
echo "  - Fusion:    PID $FUSION_PID"
echo "  - RViz:      PID $RVIZ_PID"
echo ""
echo "Topics Available:"
echo "  - LiDAR:           /unilidar/cloud"
echo "  - Camera RGB:      /camera/camera/color/image_raw"
echo "  - Camera Depth:    /camera/camera/depth/image_rect_raw"
echo "  - Colored Fusion:  /unilidar/colored_cloud"
echo ""
echo "RViz Configuration:"
echo "  - Fixed Frame: unilidar_lidar"
echo "  - Raw LiDAR: white point cloud"
echo "  - Colored Fusion: RGB point cloud"
echo ""
echo -e "${YELLOW}In RViz:${NC}"
echo "  1. Enable 'Raw_L2_LiDAR' display for uncolored points"
echo "  2. Enable 'Fused_PointCloud' display for colored points"
echo "  3. Set Color Transformer to 'RGB8' for colored display"
echo "  4. Adjust point sizes to 5-10 pixels"
echo ""
echo -e "${YELLOW}For better alignment:${NC}"
echo "  Run: ./scripts/launch_calibration.sh"
echo ""
echo -e "${YELLOW}Logs:${NC}"
echo "  - LiDAR:  /tmp/lidar.log"
echo "  - Camera: /tmp/camera.log"
echo "  - Fusion: /tmp/fusion.log"
echo "  - RViz:   /tmp/rviz.log"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop all processes${NC}"

# Wait for user interrupt
wait
