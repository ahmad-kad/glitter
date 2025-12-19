#!/usr/bin/env python3
"""
Visual Camera Test - Shows camera feed or test pattern in OpenCV window
"""

import cv2
import numpy as np
import time
import sys

def create_test_pattern(width=640, height=480):
    """Create a test pattern image when no camera is available"""
    # Create a colorful test pattern
    img = np.zeros((height, width, 3), dtype=np.uint8)

    # Add some color gradients and text
    for y in range(height):
        for x in range(width):
            # Create a gradient pattern
            r = int((x / width) * 255)
            g = int((y / height) * 255)
            b = int(((x + y) / (width + height)) * 255)
            img[y, x] = [b, g, r]

    # Add text
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, "Camera Test Pattern", (50, 50), font, 1, (255, 255, 255), 2)
    cv2.putText(img, "No camera connected", (50, 100), font, 0.7, (255, 255, 255), 2)
    cv2.putText(img, "Connect camera to CSI port", (50, 130), font, 0.7, (255, 255, 255), 2)
    cv2.putText(img, "Run: sudo apt install libcamera-apps", (50, 160), font, 0.7, (255, 255, 255), 2)
    cv2.putText(img, "Press 'q' to quit", (50, height - 50), font, 0.7, (255, 255, 255), 2)

    # Add some animated elements
    current_time = time.time()
    center_x = int(width/2 + np.sin(current_time) * 50)
    center_y = int(height/2 + np.cos(current_time) * 50)
    cv2.circle(img, (center_x, center_y), 20, (0, 255, 0), -1)

    return img

def test_camera_devices():
    """Test available camera devices and return working one"""
    devices_to_try = ['/dev/video0', '/dev/video1', '/dev/video2', '/dev/video3',
                     '/dev/video4', '/dev/video5', 0, 1, 2]  # Try both device paths and indices

    for device in devices_to_try:
        try:
            print(f"Testing camera device: {device}")
            cap = cv2.VideoCapture(device)

            if cap.isOpened():
                # Try to read a frame
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    print(f"✓ Camera working on device: {device}")
                    print(f"  Frame shape: {frame.shape}")
                    return cap
                else:
                    print(f"✗ Device {device} opened but cannot read frames")
                    cap.release()
            else:
                print(f"✗ Cannot open device: {device}")

        except Exception as e:
            print(f"✗ Error testing device {device}: {e}")
            continue

    print("✗ No working camera devices found")
    return None

def main():
    print("==========================================")
    print("Visual Camera Test")
    print("==========================================")
    print("Testing camera devices...")
    print("A window should appear showing camera feed or test pattern")
    print("Press 'q' or close window to exit")
    print("")

    # Try to find a working camera
    cap = test_camera_devices()

    window_name = "Camera Test"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 640, 480)

    print("")
    print("Opening camera test window...")
    print("Close the window to exit the test")
    print("")

    try:
        start_time = time.time()
        frame_count = 0

        while True:
            if cap is not None:
                # Read from camera
                ret, frame = cap.read()
                if ret and frame is not None:
                    # Resize if needed
                    if frame.shape[1] != 640:
                        frame = cv2.resize(frame, (640, 480))

                    # Add some overlay info
                    fps = frame_count / (time.time() - start_time) if time.time() > start_time else 0
                    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, "Press 'q' to quit", (10, frame.shape[0] - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    cv2.imshow(window_name, frame)
                    frame_count += 1
                else:
                    print("Lost camera connection, switching to test pattern...")
                    cap.release()
                    cap = None
            else:
                # Show test pattern
                test_img = create_test_pattern()
                cv2.imshow(window_name, test_img)

            # Check for quit key
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC
                break

            # Auto-exit after 30 seconds if no interaction
            if time.time() - start_time > 30:
                print("Auto-exiting after 30 seconds...")
                break

    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error during camera test: {e}")
    finally:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        print("")
        print("Camera test complete!")

if __name__ == "__main__":
    main()


