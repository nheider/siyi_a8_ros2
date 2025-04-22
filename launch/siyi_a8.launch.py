#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('siyi_a8_ros2')
    
    # Define launch arguments
    camera_ip_arg = DeclareLaunchArgument(
        'camera_ip',
        default_value='192.168.144.25',
        description='IP address of the SIYI A8 camera'
    )
    
    command_port_arg = DeclareLaunchArgument(
        'command_port',
        default_value='37260',
        description='UDP port for camera commands'
    )
    
    rtsp_port_arg = DeclareLaunchArgument(
        'rtsp_port',
        default_value='8554',
        description='RTSP port for video streaming'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='10.0',
        description='Update rate in Hz for gimbal info'
    )
    
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='1280',
        description='Image width'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='720',
        description='Image height'
    )
    
    look_at_distance_arg = DeclareLaunchArgument(
        'look_at_distance',
        default_value='10.0',
        description='Distance in meters for the camera_look_at frame'
    )
    
    # Define the camera node
    siyi_camera_node = Node(
        package='siyi_a8_ros2',
        executable='siyi_camera_node',
        name='siyi_a8_camera_node',
        parameters=[{
            'camera_ip': LaunchConfiguration('camera_ip'),
            'command_port': LaunchConfiguration('command_port'),
            'rtsp_port': LaunchConfiguration('rtsp_port'),
            'update_rate': LaunchConfiguration('update_rate'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
        }],
        output='screen'
    )
    
    gimbal_tf_node = Node(
        package='siyi_a8_ros2',
        executable='gimbal_tf_broadcaster',
        name='gimbal_tf_broadcaster',
        parameters=[{
            'look_at_distance': LaunchConfiguration('look_at_distance'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        camera_ip_arg,
        command_port_arg,
        rtsp_port_arg,
        update_rate_arg,
        image_width_arg,
        image_height_arg,
        look_at_distance_arg,
        siyi_camera_node,
        gimbal_tf_node
    ])