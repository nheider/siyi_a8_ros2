#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
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
        
        # Create transform listener to get static transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Give TF system time to initialize
        self.create_timer(1.0, self.init_transform_lookup)
        
        # Cached translation values
        self.translations = {
            'gimbal_base_link_to_yaw': None,
            'yaw_to_roll': None,
            'roll_to_pitch': None,
            'pitch_to_camera': None,
        }
        
        # Timer for publishing transforms
        self.transform_timer = None  # We'll set this after initialization
        
        self.get_logger().info('Gimbal TF broadcaster started')
    
    def init_transform_lookup(self):
        """Initialize by looking up the static transforms from URDF"""
        try:
            # Get translations from URDF static TFs
            yaw_tf = self.tf_buffer.lookup_transform(
                'gimbal_base_link', 'gimbal_yaw_link', rclpy.time.Time())
            self.translations['gimbal_base_link_to_yaw'] = yaw_tf.transform.translation
            
            roll_tf = self.tf_buffer.lookup_transform(
                'gimbal_yaw_link', 'gimbal_roll_link', rclpy.time.Time())
            self.translations['yaw_to_roll'] = roll_tf.transform.translation
            
            pitch_tf = self.tf_buffer.lookup_transform(
                'gimbal_roll_link', 'gimbal_pitch_link', rclpy.time.Time())
            self.translations['roll_to_pitch'] = pitch_tf.transform.translation
            
            camera_tf = self.tf_buffer.lookup_transform(
                'gimbal_pitch_link', 'gimbal_camera_link', rclpy.time.Time())
            self.translations['pitch_to_camera'] = camera_tf.transform.translation
            
            # Start publishing transforms after we've got the initial data
            self.transform_timer = self.create_timer(0.1, self.publish_transforms)  # 10Hz
            self.get_logger().info('Successfully retrieved static transforms')
            
        except Exception as e:
            self.get_logger().warn(f'Failed to get static transforms: {str(e)}. Will retry...')
        
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
        """Publish all the gimbal-related transforms"""
        now = self.get_clock().now().to_msg()
        
        # Only publish if we've successfully initialized translations
        if None in self.translations.values():
            self.get_logger().warn("Translations not yet initialized, skipping transform publish")
            return
        
        # 1. Publish yaw link transform
        yaw_tf = TransformStamped()
        yaw_tf.header.stamp = now
        yaw_tf.header.frame_id = 'gimbal_base_link'
        yaw_tf.child_frame_id = 'gimbal_yaw_link'
        
        # Copy translation from static TF
        yaw_tf.transform.translation.x = self.translations['gimbal_base_link_to_yaw'].x
        yaw_tf.transform.translation.y = self.translations['gimbal_base_link_to_yaw'].y
        yaw_tf.transform.translation.z = self.translations['gimbal_base_link_to_yaw'].z
        
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
        
        # Copy translation from static TF
        roll_tf.transform.translation.x = self.translations['yaw_to_roll'].x
        roll_tf.transform.translation.y = self.translations['yaw_to_roll'].y
        roll_tf.transform.translation.z = self.translations['yaw_to_roll'].z
        
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
        
        # Copy translation from static TF
        pitch_tf.transform.translation.x = self.translations['roll_to_pitch'].x
        pitch_tf.transform.translation.y = self.translations['roll_to_pitch'].y
        pitch_tf.transform.translation.z = self.translations['roll_to_pitch'].z
        
        # Apply pitch rotation around Y
        pitch_quat = self.quaternion_from_euler(0, self.pitch, 0)
        pitch_tf.transform.rotation.x = pitch_quat[0]
        pitch_tf.transform.rotation.y = pitch_quat[1]
        pitch_tf.transform.rotation.z = pitch_quat[2]
        pitch_tf.transform.rotation.w = pitch_quat[3]
        self.tf_broadcaster.sendTransform(pitch_tf)
        
        # 4. Connect pitch_link to camera_link
        camera_link_tf = TransformStamped()
        camera_link_tf.header.stamp = now
        camera_link_tf.header.frame_id = 'gimbal_pitch_link'
        camera_link_tf.child_frame_id = 'gimbal_camera_link'
        
        # Copy translation from static TF
        camera_link_tf.transform.translation.x = self.translations['pitch_to_camera'].x
        camera_link_tf.transform.translation.y = self.translations['pitch_to_camera'].y
        camera_link_tf.transform.translation.z = self.translations['pitch_to_camera'].z
        
        # Identity quaternion (no rotation)
        camera_link_tf.transform.rotation.w = 1.0
        camera_link_tf.transform.rotation.x = 0.0
        camera_link_tf.transform.rotation.y = 0.0
        camera_link_tf.transform.rotation.z = 0.0
        self.tf_broadcaster.sendTransform(camera_link_tf)
        
        # 5. Connect gimbal_camera_link to gimbal_camera_optical_frame
        camera_tf = TransformStamped()
        camera_tf.header.stamp = now
        camera_tf.header.frame_id = 'gimbal_camera_link'
        camera_tf.child_frame_id = 'gimbal_camera_optical_frame'
        
        # Fix the orientation so that:
        # - z is forward (along optical axis)
        # - x is right (not left)
        # - y is down (image not upside down)
        optical_quat = self.quaternion_from_euler(-math.pi/2, 0, -math.pi/2)
        camera_tf.transform.rotation.x = optical_quat[0]
        camera_tf.transform.rotation.y = optical_quat[1]
        camera_tf.transform.rotation.z = optical_quat[2]
        camera_tf.transform.rotation.w = optical_quat[3]
        
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