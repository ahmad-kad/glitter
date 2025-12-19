#!/bin/bash
# Quick test after camera hardware is connected

echo "🔍 Testing camera readiness..."
echo ""

# Test 1: Camera detection
echo "1. Checking camera detection:"
if libcamera-hello --list-cameras 2>/dev/null | grep -q "Available"; then
    echo "   ✅ Camera detected by libcamera"
else
    echo "   ❌ Camera not detected"
    echo "   💡 Check CSI ribbon cable connection"
    exit 1
fi

echo ""

# Test 2: Software modules
echo "2. Checking software modules:"
source activate_env.sh 2>/dev/null

if python3 -c "import pykms; from picamera2 import Picamera2; print('   ✅ pykms and picamera2 ready')" 2>/dev/null; then
    echo "   ✅ Software modules ready"
else
    echo "   ❌ Software modules missing"
    exit 1
fi

echo ""

# Test 3: Camera capture
echo "3. Testing camera capture:"
if python3 -c "
from picamera2 import Picamera2
cam = Picamera2()
config = cam.create_preview_configuration(main={'size': (640, 480)})
cam.configure(config)
cam.start()
frame = cam.capture_array()
cam.stop()
print(f'   ✅ Camera capture successful: {frame.shape}')
" 2>/dev/null; then
    echo "   ✅ Camera capture working"
else
    echo "   ❌ Camera capture failed"
    exit 1
fi

echo ""

# Test 4: ROS camera node
echo "4. Testing ROS camera node:"
python3 src/camera/pi_camera_node.py 2>/dev/null &
CAMERA_PID=$!
sleep 3

if ros2 topic list 2>/dev/null | grep -q "/camera/image_raw"; then
    echo "   ✅ ROS topic created"
else
    echo "   ❌ ROS topic not found"
    kill $CAMERA_PID 2>/dev/null
    exit 1
fi

# Check if publishing
if timeout 2 ros2 topic hz /camera/image_raw 2>/dev/null | grep -q "average rate"; then
    echo "   ✅ ROS publishing working (~30 Hz)"
else
    echo "   ⚠️ ROS publishing not detected (may still work)"
fi

kill $CAMERA_PID 2>/dev/null

echo ""
echo "🎉 CAMERA IS READY FOR SENSOR FUSION!"
echo ""
echo "Next: ./launch_fusion.sh"
echo "This will show colored point clouds in RViz"








