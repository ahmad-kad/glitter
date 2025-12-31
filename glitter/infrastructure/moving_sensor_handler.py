#!/usr/bin/env python3
"""
Moving Sensor Handler - For Vehicle-Mounted LiDAR
Updates TF transforms as the vehicle moves using IMU/Odometry
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist, Vector3, Quaternion
import tf2_ros
import numpy as np
from transforms3d.euler import euler2quat, quat2euler
from transforms3d.quaternions import quat_multiply, quat_inverse
import time
from collections import deque

class MovingSensorHandler(Node):
    """
    Handles moving sensor by updating TF transforms based on IMU/Odometry
    """
    
    def __init__(self):
        super().__init__('moving_sensor_handler')
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Current pose (in map frame)
        self.current_pose = np.eye(4)  # 4x4 transformation matrix
        self.current_velocity = np.zeros(6)  # [vx, vy, vz, wx, wy, wz]
        self.last_imu_time = None
        self.last_odom_time = None
        
        # IMU data buffer
        self.imu_buffer = deque(maxlen=100)
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/unilidar/imu',
            self.imu_callback,
            10
        )
        
        # Optional: Odometry subscriber (if available from vehicle)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  # Standard ROS odometry topic
            self.odom_callback,
            10
        )
        
        # Timer to publish TF
        self.create_timer(0.1, self.publish_transform)  # 10 Hz
        
        self.get_logger().info('Moving Sensor Handler initialized')
        self.get_logger().info('Subscribed to: /unilidar/imu')
        self.get_logger().info('Subscribed to: /odom (if available)')
        self.get_logger().info('Publishing: map -> base_link -> unilidar_lidar')
    
    def imu_callback(self, msg: Imu):
        """Process IMU data for orientation and angular velocity"""
        try:
            # Extract orientation
            q = msg.orientation
            orientation = np.array([q.w, q.x, q.y, q.z])  # [w, x, y, z] format
            
            # Extract angular velocity
            ang_vel = msg.angular_velocity
            angular_velocity = np.array([ang_vel.x, ang_vel.y, ang_vel.z])
            
            # Extract linear acceleration (for velocity estimation)
            lin_acc = msg.linear_acceleration
            linear_acceleration = np.array([lin_acc.x, lin_acc.y, lin_acc.z])
            
            # Store IMU data
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.imu_buffer.append({
                'timestamp': timestamp,
                'orientation': orientation,
                'angular_velocity': angular_velocity,
                'linear_acceleration': linear_acceleration
            })
            
            # Update orientation in current pose
            self._update_orientation(orientation)
            
            # Update angular velocity
            self.current_velocity[3:6] = angular_velocity
            
            self.last_imu_time = timestamp
            
        except Exception as e:
            self.get_logger().error(f'Error processing IMU: {e}')
    
    def odom_callback(self, msg: Odometry):
        """Process odometry data for position and velocity"""
        try:
            # Extract position
            pos = msg.pose.pose.position
            position = np.array([pos.x, pos.y, pos.z])
            
            # Extract orientation
            q = msg.pose.pose.orientation
            orientation = np.array([q.w, q.x, q.y, q.z])
            
            # Extract velocity
            twist = msg.twist.twist
            linear_vel = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
            angular_vel = np.array([twist.angular.x, twist.angular.y, twist.angular.z])
            
            # Update pose
            self._update_pose_from_odom(position, orientation)
            
            # Update velocity
            self.current_velocity[0:3] = linear_vel
            self.current_velocity[3:6] = angular_vel
            
            self.last_odom_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
        except Exception as e:
            self.get_logger().error(f'Error processing odometry: {e}')
    
    def _update_orientation(self, quaternion):
        """Update orientation part of pose matrix"""
        # Convert quaternion to rotation matrix
        w, x, y, z = quaternion
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])
        
        # Update rotation part of pose
        self.current_pose[:3, :3] = R
    
    def _update_pose_from_odom(self, position, orientation):
        """Update full pose from odometry"""
        # Update position
        self.current_pose[:3, 3] = position
        
        # Update orientation
        self._update_orientation(orientation)
    
    def _integrate_motion(self, dt):
        """Integrate velocity to update position (dead reckoning)"""
        if dt <= 0:
            return
        
        # Extract linear and angular velocity
        v = self.current_velocity[0:3]  # Linear velocity
        w = self.current_velocity[3:6]  # Angular velocity
        
        # Update position (simple integration)
        # In vehicle frame, assume forward motion
        # Transform velocity to world frame
        R = self.current_pose[:3, :3]
        v_world = R @ v
        
        # Update position
        self.current_pose[:3, 3] += v_world * dt
        
        # Update orientation from angular velocity
        if np.linalg.norm(w) > 1e-6:
            # Convert angular velocity to quaternion delta
            angle = np.linalg.norm(w) * dt
            axis = w / np.linalg.norm(w)
            
            # Create rotation quaternion
            q_delta = euler2quat(axis[0] * angle, axis[1] * angle, axis[2] * angle, 'sxyz')
            
            # Get current orientation as quaternion
            R_current = self.current_pose[:3, :3]
            # Convert rotation matrix to quaternion (simplified)
            trace = np.trace(R_current)
            if trace > 0:
                s = np.sqrt(trace + 1.0) * 2
                w_curr = 0.25 * s
                x_curr = (R_current[2, 1] - R_current[1, 2]) / s
                y_curr = (R_current[0, 2] - R_current[2, 0]) / s
                z_curr = (R_current[1, 0] - R_current[0, 1]) / s
                q_current = np.array([w_curr, x_curr, y_curr, z_curr])
            else:
                # Fallback: use IMU orientation if available
                if self.imu_buffer:
                    q_current = self.imu_buffer[-1]['orientation']
                else:
                    return
            
            # Multiply quaternions
            q_new = quat_multiply(q_current, q_delta)
            self._update_orientation(q_new)
    
    def publish_transform(self):
        """Publish TF transforms for moving sensor"""
        current_time = self.get_clock().now()
        
        # If we have velocity and time since last update, integrate motion
        if self.last_imu_time is not None:
            current_timestamp = time.time()
            dt = current_timestamp - self.last_imu_time if self.last_imu_time else 0.1
            
            # If no odometry, use IMU for dead reckoning
            if self.last_odom_time is None and len(self.imu_buffer) >= 2:
                self._integrate_motion(dt)
        
        # Publish map -> base_link transform
        transform_map_base = TransformStamped()
        transform_map_base.header.stamp = current_time.to_msg()
        transform_map_base.header.frame_id = 'map'
        transform_map_base.child_frame_id = 'base_link'
        
        # Set translation
        transform_map_base.transform.translation.x = float(self.current_pose[0, 3])
        transform_map_base.transform.translation.y = float(self.current_pose[1, 3])
        transform_map_base.transform.translation.z = float(self.current_pose[2, 3])
        
        # Set rotation (from rotation matrix to quaternion)
        R = self.current_pose[:3, :3]
        trace = np.trace(R)
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        else:
            # Use IMU orientation if available
            if self.imu_buffer:
                q = self.imu_buffer[-1]['orientation']
                w, x, y, z = q[0], q[1], q[2], q[3]
            else:
                w, x, y, z = 1.0, 0.0, 0.0, 0.0  # Identity
        
        transform_map_base.transform.rotation.w = float(w)
        transform_map_base.transform.rotation.x = float(x)
        transform_map_base.transform.rotation.y = float(y)
        transform_map_base.transform.rotation.z = float(z)
        
        self.tf_broadcaster.sendTransform(transform_map_base)
        
        # Publish base_link -> unilidar_lidar (static offset)
        # This is the mounting offset of LiDAR on vehicle
        transform_base_lidar = TransformStamped()
        transform_base_lidar.header.stamp = current_time.to_msg()
        transform_base_lidar.header.frame_id = 'base_link'
        transform_base_lidar.child_frame_id = 'unilidar_lidar'
        
        # LiDAR mounting offset (adjust these for your setup)
        # Default: LiDAR at origin of base_link
        transform_base_lidar.transform.translation.x = 0.0
        transform_base_lidar.transform.translation.y = 0.0
        transform_base_lidar.transform.translation.z = 0.0
        
        # No rotation (LiDAR aligned with vehicle)
        transform_base_lidar.transform.rotation.w = 1.0
        transform_base_lidar.transform.rotation.x = 0.0
        transform_base_lidar.transform.rotation.y = 0.0
        transform_base_lidar.transform.rotation.z = 0.0
        
        self.tf_broadcaster.sendTransform(transform_base_lidar)

def main(args=None):
    rclpy.init(args=args)
    node = MovingSensorHandler()
    
    try:
        print("\n" + "="*50)
        print("Moving Sensor Handler Running")
        print("="*50)
        print("This node updates TF transforms as the vehicle moves")
        print("Uses IMU for orientation, Odometry for position")
        print("Press Ctrl+C to stop")
        print("="*50 + "\n")
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()












