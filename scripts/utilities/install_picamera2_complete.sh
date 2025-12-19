#!/bin/bash
# Complete picamera2 installation with all dependencies

echo "Installing picamera2 dependencies..."

# Install system dependencies needed to build picamera2
sudo apt update
sudo apt install -y \
    libcap-dev \
    python3-dev \
    python3-pip \
    python3-setuptools \
    build-essential \
    libcamera-dev \
    python3-libcamera \
    python3-kms++

echo ""
echo "Installing picamera2 via pip..."
pip3 install --break-system-packages picamera2

echo ""
echo "Testing installation..."
python3 << 'EOF'
try:
    from picamera2 import Picamera2
    print("✓ picamera2 imported successfully")
    
    # Try to create camera object
    cam = Picamera2()
    print("✓ Camera object created")
    
    # Try to configure
    config = cam.create_preview_configuration(main={"size": (640, 480)})
    cam.configure(config)
    print("✓ Camera configured")
    
    # Try to start
    cam.start()
    print("✓ Camera started")
    
    # Capture test
    frame = cam.capture_array()
    print(f"✓ Frame captured: {frame.shape}")
    
    cam.stop()
    print("✓ Camera stopped")
    print("\nSUCCESS: picamera2 is working!")
except ImportError as e:
    print(f"✗ Import error: {e}")
    exit(1)
except Exception as e:
    print(f"✗ Error: {e}")
    print("Camera may not be connected or enabled")
    exit(1)
EOF

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ picamera2 is ready!"
    echo ""
    echo "You can now run:"
    echo "  python3 src/camera/pi_camera_node.py"
else
    echo ""
    echo "Installation completed but camera test failed"
    echo "Check camera connection and try rebooting"
fi










