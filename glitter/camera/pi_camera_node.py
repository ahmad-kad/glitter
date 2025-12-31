#!/usr/bin/env python3
"""
Raspberry Pi Camera ROS 2 Node
Publishes camera images to /camera/image_raw topic
Uses picamera2 library for Pi Camera access (recommended)
Falls back to OpenCV if picamera2 not available
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import time

class PiCameraNode(Node):
    def __init__(self):
        super().__init__('pi_camera_node')
        
        # Publisher
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('use_picamera2', True)
        
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        self.use_picamera2 = self.get_parameter('use_picamera2').get_parameter_value().bool_value
        
        # Initialize camera
        self.camera = None
        self.camera_type = None  # 'picamera2' or 'opencv'
        self._init_camera()
        
        # Timer for publishing
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Pi Camera Node started: {self.width}x{self.height} @ {self.fps} FPS')
        
    def _init_camera(self):
        """Initialize camera using picamera2 (preferred) or OpenCV fallback"""
        if self.use_picamera2:
            try:
                from picamera2 import Picamera2
                self.camera = Picamera2()
                
                # Configure camera
                config = self.camera.create_preview_configuration(
                    main={"size": (self.width, self.height), "format": "RGB888"}
                )
                self.camera.configure(config)
                self.camera.start()
                
                self.camera_type = 'picamera2'
                self.get_logger().info('Camera initialized with picamera2')
                return
            except ImportError:
                self.get_logger().warn('picamera2 not available - install: pip3 install --break-system-packages picamera2')
                self.get_logger().warn('Trying OpenCV fallback...')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize picamera2: {e}')
                self.get_logger().warn('Trying OpenCV fallback...')
        
        # Fallback: Try OpenCV with video devices
        self.get_logger().info('Trying OpenCV with video devices...')
        # Try CSI camera devices first (rp1-cfe platform), then others
        devices_to_try = ['/dev/video0', '/dev/video1', '/dev/video2', '/dev/video3',
                         '/dev/video4', '/dev/video5', '/dev/video6', '/dev/video7',
                         '/dev/video19', '/dev/video20', '/dev/video21']
        
        for device in devices_to_try:
            try:
                self.get_logger().info(f'Trying to open {device}...')
                cap = cv2.VideoCapture(device)
                if cap.isOpened():
                    self.get_logger().info(f'{device} opened successfully, testing read...')
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        self.get_logger().info(f'{device} can capture frames: {frame.shape}')
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                        cap.set(cv2.CAP_PROP_FPS, self.fps)
                        self.camera = cap
                        self.camera_type = 'opencv'
                        self.get_logger().info(f'Camera initialized successfully on {device} using OpenCV')
                        return
                    else:
                        self.get_logger().warn(f'{device} opened but cannot read frames')
                        cap.release()
                else:
                    self.get_logger().info(f'{device} could not be opened')
            except Exception as e:
                self.get_logger().error(f'Exception opening {device}: {e}')
                continue
        
        self.get_logger().error('Failed to initialize camera')
        self.get_logger().error('Options:')
        self.get_logger().error('  1. Install picamera2: pip3 install --break-system-packages picamera2')
        self.get_logger().error('  2. Check camera is connected via ribbon cable')
        self.get_logger().error('  3. Try rebooting after connecting camera')
        self.camera = None
        
    def timer_callback(self):
        """Capture and publish camera image"""
        if self.camera is None:
            return
            
        try:
            if self.camera_type == 'picamera2':
                # picamera2 method
                frame = self.camera.capture_array()
                # picamera2 returns RGB, convert to BGR for OpenCV/ROS
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                # OpenCV method
                ret, frame = self.camera.read()
                if not ret or frame is None:
                    self.get_logger().warn('Failed to read frame from camera')
                    return
            
            # Resize if needed
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height))
            
            # Convert to ROS message
            ros_image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_frame'
            
            # Publish
            self.publisher.publish(ros_image)
                    
        except Exception as e:
            self.get_logger().error(f'Failed to capture image: {e}')
            
    def destroy_node(self):
        """Cleanup camera on shutdown"""
        if self.camera is not None:
            if self.camera_type == 'picamera2':
                if hasattr(self.camera, 'stop'):
                    self.camera.stop()
            else:
                if hasattr(self.camera, 'release'):
                    self.camera.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PiCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
