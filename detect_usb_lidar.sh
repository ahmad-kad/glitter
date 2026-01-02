#!/bin/bash
# Detect USB LiDAR device
# This script monitors for USB serial devices that might be the LiDAR

echo "Checking for USB serial devices..."
echo ""

# Check common device names
DEVICES=("/dev/ttyACM0" "/dev/ttyACM1" "/dev/ttyUSB0" "/dev/ttyUSB1" "/dev/ttyUSB2")

FOUND=false
for dev in "${DEVICES[@]}"; do
    if [ -e "$dev" ]; then
        echo "✓ Found: $dev"
        ls -la "$dev"
        FOUND=true
    fi
done

if [ "$FOUND" = false ]; then
    echo "No USB serial devices found in common locations."
    echo ""
    echo "Please check:"
    echo "1. Is the LiDAR USB cable connected?"
    echo "2. Is the LiDAR powered on?"
    echo "3. Try unplugging and replugging the USB cable"
    echo ""
    echo "All /dev/tty* devices:"
    ls -la /dev/tty* 2>/dev/null | grep -v "pts\|console\|printk" | head -20
    echo ""
    echo "USB devices:"
    lsusb 2>/dev/null | head -10
fi

