#!/bin/bash
# Install picamera2 for Pi Camera

echo "Installing picamera2..."

# Install via pip (works on Ubuntu 24.04)
pip3 install --break-system-packages picamera2 2>&1 | tail -10

# Test
python3 -c "from picamera2 import Picamera2; print('picamera2 installed')" 2>&1











