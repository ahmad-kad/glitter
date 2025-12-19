#!/bin/bash
# Comprehensive Point Cloud Verification Script

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "=========================================="
echo "Point Cloud Verification"
echo "=========================================="
echo ""

# 1. Check if topic exists
echo "1. Checking topic existence..."
if ros2 topic list | grep -q "/unilidar/cloud"; then
    echo -e "${GREEN}✓ Topic /unilidar/cloud exists${NC}"
else
    echo -e "${RED}✗ Topic /unilidar/cloud NOT found${NC}"
    exit 1
fi

# 2. Check publisher/subscriber count
echo ""
echo "2. Checking publisher/subscriber status..."
TOPIC_INFO=$(ros2 topic info /unilidar/cloud -v 2>&1)
PUB_COUNT=$(echo "$TOPIC_INFO" | grep "Publisher count" | awk '{print $3}')
SUB_COUNT=$(echo "$TOPIC_INFO" | grep "Subscription count" | awk '{print $3}')

if [ "$PUB_COUNT" -gt 0 ]; then
    echo -e "${GREEN}✓ Publisher count: $PUB_COUNT${NC}"
else
    echo -e "${RED}✗ No publishers! LiDAR driver not running?${NC}"
    exit 1
fi

if [ "$SUB_COUNT" -gt 0 ]; then
    echo -e "${GREEN}✓ Subscription count: $SUB_COUNT (RViz is connected!)${NC}"
else
    echo -e "${YELLOW}⚠ Subscription count: $SUB_COUNT (RViz not subscribed)${NC}"
    echo "   Make sure RViz is running and display is enabled"
fi

# 3. Check topic frequency
echo ""
echo "3. Checking data rate..."
FREQ=$(timeout 3 ros2 topic hz /unilidar/cloud 2>&1 | grep "average rate" | tail -1 | awk '{print $3}')
if [ -n "$FREQ" ]; then
    echo -e "${GREEN}✓ Publishing at ~${FREQ} Hz${NC}"
else
    echo -e "${YELLOW}⚠ Could not measure frequency${NC}"
fi

# 4. Check message structure
echo ""
echo "4. Checking message structure..."
MSG_INFO=$(timeout 3 ros2 topic echo /unilidar/cloud --once 2>&1 | head -30)
if echo "$MSG_INFO" | grep -q "frame_id: unilidar_lidar"; then
    echo -e "${GREEN}✓ Frame ID: unilidar_lidar${NC}"
else
    echo -e "${YELLOW}⚠ Frame ID check failed${NC}"
fi

WIDTH=$(echo "$MSG_INFO" | grep "width:" | head -1 | awk '{print $2}')
HEIGHT=$(echo "$MSG_INFO" | grep "height:" | head -1 | awk '{print $2}')

if [ -n "$WIDTH" ] && [ "$WIDTH" -gt 0 ]; then
    echo -e "${GREEN}✓ Point cloud has data: ${WIDTH} points${NC}"
    if [ "$WIDTH" -gt 1000 ]; then
        echo -e "${GREEN}  (Good point count - should be visible)${NC}"
    elif [ "$WIDTH" -gt 100 ]; then
        echo -e "${YELLOW}  (Low point count - may be sparse)${NC}"
    else
        echo -e "${RED}  (Very low point count - check LiDAR)${NC}"
    fi
else
    echo -e "${RED}✗ No points in cloud!${NC}"
    exit 1
fi

# 5. Check TF frames
echo ""
echo "5. Checking TF frames..."
if ros2 topic list | grep -q "/tf"; then
    echo -e "${GREEN}✓ TF topic exists${NC}"
    if timeout 2 ros2 run tf2_ros tf2_echo map unilidar_lidar 2>&1 | grep -q "At time"; then
        echo -e "${GREEN}✓ TF transform map -> unilidar_lidar exists${NC}"
    else
        echo -e "${YELLOW}⚠ TF transform not found (may be OK if using Fixed Frame)${NC}"
    fi
else
    echo -e "${YELLOW}⚠ No TF topic (may be OK)${NC}"
fi

# 6. Summary
echo ""
echo "=========================================="
echo "Summary"
echo "=========================================="

if [ "$PUB_COUNT" -gt 0 ] && [ -n "$WIDTH" ] && [ "$WIDTH" -gt 0 ]; then
    echo -e "${GREEN}✓ Point cloud is PUBLISHING and has data${NC}"
    if [ "$SUB_COUNT" -gt 0 ]; then
        echo -e "${GREEN}✓ RViz is SUBSCRIBED${NC}"
        echo ""
        echo "Everything looks good! Point cloud should be visible in RViz."
        echo ""
        echo "If you don't see it in RViz:"
        echo "  1. Check Fixed Frame = 'unilidar_lidar'"
        echo "  2. Check Raw_L2_LiDAR display is ENABLED"
        echo "  3. Check Topic = '/unilidar/cloud'"
        echo "  4. Try increasing Size (Pixels) to 5-10"
    else
        echo -e "${YELLOW}⚠ RViz is NOT subscribed${NC}"
        echo ""
        echo "In RViz:"
        echo "  1. Displays → Raw_L2_LiDAR → Enable checkbox"
        echo "  2. Displays → Raw_L2_LiDAR → Topic = '/unilidar/cloud'"
        echo "  3. Global Options → Fixed Frame = 'unilidar_lidar'"
    fi
else
    echo -e "${RED}✗ Point cloud has issues${NC}"
    echo "Check LiDAR driver is running and connected"
fi

echo ""

