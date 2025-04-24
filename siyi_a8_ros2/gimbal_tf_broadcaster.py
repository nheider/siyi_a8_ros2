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
        
        # Create subscribers
        self.gimbal_state_sub = self.create_subscription(
            Float32MultiArray, 
            'siyi_a8/gimbal_state', 
            self.gimbal_state_callback, 
            10)
            
        # Create transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Start publishing transforms immediately at 10Hz
        self.transform_timer = self.create_timer(0.1, self.publish_transforms)
        
        self.get_logger().info('Gimbal TF broadcaster started')
    
    def gimbal_state_callback(self, msg):
        """Process gimbal state updates"""
        if len(msg.data) >= 3:
            # Get angle values directly from the camera node (already filtered there)
            raw_yaw_deg = msg.data[0]
            raw_pitch_deg = msg.data[1]
            raw_roll_deg = msg.data[2]
            
            # Convert to radians
            self.yaw = math.radians(raw_yaw_deg)
            self.pitch = -math.radians(raw_pitch_deg)  # Invert pitch to match ROS convention
            self.roll = math.radians(raw_roll_deg)
            
            # Log raw values for debugging
            self.get_logger().debug(f"Using angles (deg): yaw={raw_yaw_deg:.1f}, pitch={raw_pitch_deg:.1f}, roll={raw_roll_deg:.1f}")
            
    def publish_transforms(self):
        """Publish only the dynamic gimbal-related transforms"""
        now = self.get_clock().now().to_msg()
        
        # 1. Publish yaw link transform
        yaw_tf = TransformStamped()
        yaw_tf.header.stamp = now
        yaw_tf.header.frame_id = 'gimbal_base_link'
        yaw_tf.child_frame_id = 'gimbal_yaw_link'
        
        # No translation - this comes from URDF static transforms
        yaw_tf.transform.translation.x = 0.0
        yaw_tf.transform.translation.y = 0.0
        yaw_tf.transform.translation.z = 0.0
        
        # Apply yaw rotation around Z
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
        
        # No translation - this comes from URDF static transforms
        roll_tf.transform.translation.x = 0.0
        roll_tf.transform.translation.y = 0.0
        roll_tf.transform.translation.z = 0.0
        
        # Apply roll rotation around X
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
        
        # No translation - this comes from URDF static transforms
        pitch_tf.transform.translation.x = 0.0
        pitch_tf.transform.translation.y = 0.0
        pitch_tf.transform.translation.z = 0.0
        
        # Apply pitch rotation around Y
        pitch_quat = self.quaternion_from_euler(0, self.pitch, 0)
        pitch_tf.transform.rotation.x = pitch_quat[0]
        pitch_tf.transform.rotation.y = pitch_quat[1]
        pitch_tf.transform.rotation.z = pitch_quat[2]
        pitch_tf.transform.rotation.w = pitch_quat[3]
        self.tf_broadcaster.sendTransform(pitch_tf)
    
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