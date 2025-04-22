#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math


class GimbalTfBroadcaster(Node):
    """Broadcast TF frames for the gimbal joints and camera optical frame"""

    def __init__(self):
        super().__init__('gimbal_tf_broadcaster')
        
        # Initialize state variables
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        
        # Add tracking for last valid angles
        self.last_valid_yaw = 0.0
        self.last_valid_pitch = 0.0
        self.last_valid_roll = 0.0
        
        # Parameters for validation
        self.declare_parameter('max_angle_jump', 15.0)  # Max jump in degrees
        self.max_angle_jump = math.radians(self.get_parameter('max_angle_jump').value)
        
        # Create subscribers
        self.gimbal_state_sub = self.create_subscription(
            Float32MultiArray, 
            'siyi_a8/gimbal_state', 
            self.gimbal_state_callback, 
            10)
            
        # Create transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Timer for publishing transforms
        self.timer = self.create_timer(0.05, self.publish_transforms)  # 20Hz
        
        self.get_logger().info('Gimbal TF broadcaster started with angle validation')
        
    def gimbal_state_callback(self, msg):
        """Process gimbal state updates with validation"""
        if len(msg.data) >= 3:
            # Get raw angle values
            raw_yaw_deg = msg.data[0]
            raw_pitch_deg = msg.data[1]
            raw_roll_deg = msg.data[2]
            
            # Convert to radians
            new_yaw = math.radians(raw_yaw_deg)
            new_pitch = math.radians(raw_pitch_deg)
            new_roll = math.radians(raw_roll_deg)
            
            # Log raw values for debugging
            self.get_logger().debug(f"Raw angles (deg): yaw={raw_yaw_deg:.1f}, pitch={raw_pitch_deg:.1f}, roll={raw_roll_deg:.1f}")
            
            # Detect suspicious zero values when previous values were non-zero
            # SIYI protocol sometimes sends zeros for invalid measurements
            if (abs(raw_pitch_deg) < 0.1 and abs(self.last_valid_pitch - new_pitch) > self.max_angle_jump):
                self.get_logger().info(f"Rejecting suspicious pitch value: {raw_pitch_deg:.1f}°")
                new_pitch = self.pitch  # Keep previous value
            else:
                self.last_valid_pitch = new_pitch
                self.pitch = new_pitch
                
            # Similar checks for yaw and roll
            if (abs(raw_yaw_deg) < 0.1 and abs(self.last_valid_yaw - new_yaw) > self.max_angle_jump):
                self.get_logger().info(f"Rejecting suspicious yaw value: {raw_yaw_deg:.1f}°")
                new_yaw = self.yaw
            else:
                self.last_valid_yaw = new_yaw
                self.yaw = new_yaw
                
            if (abs(raw_roll_deg) < 0.1 and abs(self.last_valid_roll - new_roll) > self.max_angle_jump):
                self.get_logger().info(f"Rejecting suspicious roll value: {raw_roll_deg:.1f}°")
                new_roll = self.roll
            else:
                self.last_valid_roll = new_roll
                self.roll = new_roll
            
    def publish_transforms(self):
        """Publish all the gimbal-related transforms"""
        now = self.get_clock().now().to_msg()
        
        # 1. Publish yaw link transform
        yaw_tf = TransformStamped()
        yaw_tf.header.stamp = now
        yaw_tf.header.frame_id = 'gimbal_base_link'
        yaw_tf.child_frame_id = 'gimbal_yaw_link'
        yaw_tf.transform.translation.x = 0.0
        yaw_tf.transform.translation.y = 0.0
        yaw_tf.transform.translation.z = -0.022
        
        # Only apply yaw rotation around Z
        yaw_quat = self.quaternion_from_euler(0, 0, self.yaw)
        yaw_tf.transform.rotation.x = yaw_quat[0]
        yaw_tf.transform.rotation.y = yaw_quat[1]
        yaw_tf.transform.rotation.z = yaw_quat[2]
        yaw_tf.transform.rotation.w = yaw_quat[3]
        self.tf_broadcaster.sendTransform(yaw_tf)
        
        # 2. Publish roll link transform
        roll_tf = TransformStamped()
        roll_tf.header.stamp = now
        roll_tf.header.frame_id = 'gimbal_yaw_link'
        roll_tf.child_frame_id = 'gimbal_roll_link'
        roll_tf.transform.translation.x = -0.022
        roll_tf.transform.translation.y = 0.001
        roll_tf.transform.translation.z = -0.0322
        
        # Only apply roll rotation around X
        roll_quat = self.quaternion_from_euler(self.roll, 0, 0)
        roll_tf.transform.rotation.x = roll_quat[0]
        roll_tf.transform.rotation.y = roll_quat[1]
        roll_tf.transform.rotation.z = roll_quat[2]
        roll_tf.transform.rotation.w = roll_quat[3]
        self.tf_broadcaster.sendTransform(roll_tf)
        
        # 3. Publish pitch link transform
        pitch_tf = TransformStamped()
        pitch_tf.header.stamp = now
        pitch_tf.header.frame_id = 'gimbal_roll_link'
        pitch_tf.child_frame_id = 'gimbal_pitch_link'
        pitch_tf.transform.translation.x = 0.03
        pitch_tf.transform.translation.y = -0.00065
        pitch_tf.transform.translation.z = 0.0002
        
        # Only apply pitch rotation around Y
        pitch_quat = self.quaternion_from_euler(0, self.pitch, 0)
        pitch_tf.transform.rotation.x = pitch_quat[0]
        pitch_tf.transform.rotation.y = pitch_quat[1]
        pitch_tf.transform.rotation.z = pitch_quat[2]
        pitch_tf.transform.rotation.w = pitch_quat[3]
        self.tf_broadcaster.sendTransform(pitch_tf)
        
        # 4. Connect gimbal_camera_link to siyi_camera_optical_frame
        # This maps the URDF camera link to the camera driver's frame
        camera_tf = TransformStamped()
        camera_tf.header.stamp = now
        camera_tf.header.frame_id = 'gimbal_camera_link'
        camera_tf.child_frame_id = 'siyi_camera_optical_frame'
        
        # Apply the correct rotation to align the camera optical frame properly
        # Rotate 90 degrees around X axis to get standard optical frame orientation
        # (Z forward along optical axis, X right, Y down)
        camera_quat = self.quaternion_from_euler(math.pi/2, 0, 0)
        camera_tf.transform.rotation.x = camera_quat[0]
        camera_tf.transform.rotation.y = camera_quat[1]
        camera_tf.transform.rotation.z = camera_quat[2]
        camera_tf.transform.rotation.w = camera_quat[3]
        
        self.tf_broadcaster.sendTransform(camera_tf)
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        # ZYX rotation sequence (yaw, pitch, roll)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr

        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = GimbalTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 