#!/bin/bash
# Camera test with visual window display

source /opt/ros/jazzy/setup.bash

echo "=========================================="
echo "Camera Test with Visual Display"
echo "=========================================="
echo ""

# Check if libcamera is available for visual preview
if command -v libcamera-hello >/dev/null 2>&1; then
    echo "Using libcamera for camera preview..."
    echo "A camera preview window should appear"
    echo ""

    # Try libcamera preview first
    libcamera-hello --timeout 5000 --width 640 --height 480 &
    LIBCAMERA_PID=$!
    sleep 2

    if kill -0 $LIBCAMERA_PID 2>/dev/null; then
        echo "✓ Camera preview window opened"
        echo "Close the window to continue..."
        wait $LIBCAMERA_PID
    else
        echo "✗ Camera preview failed - trying ROS camera..."
    fi
else
    echo "libcamera not available - trying ROS camera with rqt_image_view..."
fi

echo ""
echo "=========================================="
echo "ROS Camera Test"
echo "=========================================="
echo ""

echo "Launching ROS camera node..."
echo "This will run for 15 seconds with visual display"
echo ""

# Launch camera
ros2 run camera_ros camera_node &
CAMERA_PID=$!

echo "Camera PID: $CAMERA_PID"
echo "Waiting 3 seconds for initialization..."
sleep 3

# Check if topic exists
if ros2 topic list 2>/dev/null | grep -q "/camera/image_raw"; then
    echo "✓ Camera topic exists: /camera/image_raw"

    # Check publishing
    echo ""
    echo "Checking publishing rate..."
    timeout 3 ros2 topic hz /camera/image_raw 2>&1 | head -10

    # Check topic info
    echo ""
    echo "Topic info:"
    ros2 topic info /camera/image_raw 2>/dev/null

    echo ""
    echo "=========================================="
    echo "Camera Status"
    echo "=========================================="

    PUB_COUNT=$(ros2 topic info /camera/image_raw 2>/dev/null | grep "Publisher count" | awk '{print $3}')

    if [ "$PUB_COUNT" = "1" ]; then
        echo "✓ Camera is publishing!"
        echo ""
        echo "Opening camera image viewer window..."
        echo "Close the image viewer window to continue the test"
        echo ""

        # Launch rqt_image_view to show camera feed
        ros2 run rqt_image_view rqt_image_view /camera/image_raw &
        VIEWER_PID=$!

        # Wait for viewer to start
        sleep 2

        # Run for 10 seconds to show the camera feed
        echo "Camera viewer running for 10 seconds..."
        echo "You should see the camera image in the viewer window"
        sleep 8

        # Close viewer
        kill $VIEWER_PID 2>/dev/null || true
        echo ""
        echo "✓ Camera viewer test complete"

    else
        echo "⚠ Camera topic exists but not publishing yet"
        echo "  Publisher count: $PUB_COUNT"
        echo "  Check logs or wait a bit longer"
    fi
else
    echo "✗ Camera topic not found"
    echo "  Check if camera_ros is installed:"
    echo "    sudo apt install ros-jazzy-camera-ros"
    echo ""
    echo "Alternative: Try the Pi Camera node:"
    echo "  python3 src/camera/pi_camera_node.py"
fi

echo ""
echo "=========================================="
echo "Visual Camera Test"
echo "=========================================="
echo ""
echo "Launching visual camera test window..."
echo "This will show a test pattern or camera feed in a window"
echo ""

# Run the visual camera test
python3 visual_camera_test.py

# Cleanup ROS camera if it's still running
if [ ! -z "$CAMERA_PID" ]; then
    kill $CAMERA_PID 2>/dev/null || true
    wait $CAMERA_PID 2>/dev/null || true
fi

echo ""
echo "Camera test complete!"







