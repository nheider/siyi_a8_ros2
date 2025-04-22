#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String, Bool
import cv2
import numpy as np
import socket
import threading
import time
import binascii
from cv_bridge import CvBridge
from .siyi_message import SIYIMessage
import math
from geometry_msgs.msg import TransformStamped

class SIYICameraNode(Node):
    """
    ROS2 node for SIYI A8 Mini camera and gimbal interface
    """
    
    def __init__(self):
        super().__init__('siyi_a8_camera_node')

        # Declare parameters
        self.declare_parameter('camera_ip', '192.168.144.25')
        self.declare_parameter('command_port', 37260)
        self.declare_parameter('rtsp_port', 8554)
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)

        # Get parameters
        self.camera_ip = self.get_parameter('camera_ip').value
        self.command_port = self.get_parameter('command_port').value
        self.rtsp_port = self.get_parameter('rtsp_port').value
        self.update_rate = self.get_parameter('update_rate').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value

        # Initialize variables
        self.bridge = CvBridge()
        self.siyi_msg = SIYIMessage()
        self.udp_socket = None
        self.rtsp_camera = None
        self.gimbal_angles = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}
        self.zoom_level = 1.0
        self.is_recording = False
        self.gimbal_mode = "unknown"
        self.connected = False
        self.stop_threads = False
        self.gst_pipeline = None  # Store reference to GStreamer pipeline

        # Initialize QoS profile for camera stream
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.image_pub = self.create_publisher(Image, 'siyi_a8/image_raw', camera_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'siyi_a8/camera_info', camera_qos)
        self.gimbal_state_pub = self.create_publisher(Float32MultiArray, 'siyi_a8/gimbal_state', 10)
        self.zoom_pub = self.create_publisher(Float32MultiArray, 'siyi_a8/zoom_level', 10)
        self.status_pub = self.create_publisher(String, 'siyi_a8/status', 10)

        # Subscribers
        self.gimbal_control_sub = self.create_subscription(
            Twist, 'siyi_a8/gimbal_control', self.gimbal_control_callback, 10)
        self.gimbal_angle_sub = self.create_subscription(
            Twist, 'siyi_a8/gimbal_angle', self.gimbal_angle_callback, 10)
        self.zoom_control_sub = self.create_subscription(
            Float32MultiArray, 'siyi_a8/zoom_control', self.zoom_control_callback, 10)
        self.command_sub = self.create_subscription(
            String, 'siyi_a8/command', self.command_callback, 10)

        # Timers
        self.create_timer(1.0 / self.update_rate, self.update_gimbal_info)
        self.create_timer(5.0, self.check_connection)

        self.get_logger().info(f"Connecting to SIYI A8 Mini at {self.camera_ip}")
        self.init_connections()

    def init_connections(self):
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.settimeout(1.0)

            self.udp_thread = threading.Thread(target=self.udp_receiver)
            self.udp_thread.daemon = True
            self.udp_thread.start()

            self.setup_rtsp()

            self.send_command(self.siyi_msg.firmware_version_msg())
            self.send_command(self.siyi_msg.gimbal_info_msg())
            self.send_command(self.siyi_msg.maximum_zoom_msg())

            self.connected = True
            self.get_logger().info("Connected to SIYI A8 Mini camera")

        except Exception as e:
            self.get_logger().error(f"Failed to connect to camera: {str(e)}")
            self.connected = False

    def setup_rtsp(self):
        try:
            # Release any existing pipeline properly
            self.release_gstreamer_pipeline()
            
            # Create GStreamer pipeline with improved parameters
            rtsp_url = f"rtsp://{self.camera_ip}:{self.rtsp_port}/main.264"
            self.get_logger().info(f"Opening GStreamer pipeline for {rtsp_url}")
            
            # More robust pipeline with better error handling and TCP instead of UDP
            gst_str = (
                f"rtspsrc location={rtsp_url} latency=200 buffer-mode=auto protocols=tcp "
                f"! queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 ! rtph265depay "
                f"! h265parse ! avdec_h265 max-threads=2 ! videoconvert ! appsink max-buffers=2 drop=true"
            )
            
            # Create GStreamer-based capture with explicit pipeline
            self.rtsp_camera = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
            
            if not self.rtsp_camera.isOpened():
                self.get_logger().error("Failed to open GStreamer pipeline")
                # Try one more time with a simpler pipeline
                simple_gst_str = (
                    f"rtspsrc location={rtsp_url} latency=500 ! rtph265depay ! h265parse ! "
                    f"avdec_h265 ! videoconvert ! appsink"
                )
                self.rtsp_camera = cv2.VideoCapture(simple_gst_str, cv2.CAP_GSTREAMER)
                
                if not self.rtsp_camera.isOpened():
                    self.get_logger().error("Failed to open simpler GStreamer pipeline")
                    return False
            
            # Start the streaming thread if not already running
            if hasattr(self, 'rtsp_thread') and self.rtsp_thread.is_alive():
                self.get_logger().warn("RTSP thread already running")
            else:
                self.rtsp_thread = threading.Thread(target=self.rtsp_streaming)
                self.rtsp_thread.daemon = True
                self.rtsp_thread.start()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"GStreamer setup error: {str(e)}")
            return False

    def release_gstreamer_pipeline(self):
        """Properly release GStreamer pipeline to prevent memory leaks and segfaults"""
        try:
            if hasattr(self, 'rtsp_camera') and self.rtsp_camera is not None:
                # Set to NULL state explicitly before release
                if hasattr(self.rtsp_camera, 'pipeline') and self.rtsp_camera.pipeline is not None:
                    import gi
                    gi.require_version('Gst', '1.0')
                    from gi.repository import Gst
                    self.rtsp_camera.pipeline.set_state(Gst.State.NULL)
                    
                self.rtsp_camera.release()
                self.rtsp_camera = None
                
                # Force a garbage collection to help clean up GStreamer resources
                import gc
                gc.collect()
                
        except Exception as e:
            self.get_logger().error(f"Error releasing GStreamer pipeline: {str(e)}")

    def rtsp_streaming(self):
        consecutive_failures = 0
        max_consecutive_failures = 3  # Lower to reconnect sooner
        reconnect_delay = 3.0         # Longer delay for GStreamer to clean up
        
        while rclpy.ok() and not self.stop_threads:
            try:
                if self.rtsp_camera and self.rtsp_camera.isOpened():
                    ret, frame = self.rtsp_camera.read()
                    
                    if ret and frame is not None and frame.size > 0:
                        consecutive_failures = 0
                        
                        # Process and publish frame
                        img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                        img_msg.header.stamp = self.get_clock().now().to_msg()
                        img_msg.header.frame_id = "gimbal_camera_optical_frame"
                        self.image_pub.publish(img_msg)
                        self.publish_camera_info(img_msg.header)
                    else:
                        consecutive_failures += 1
                        self.get_logger().warning(f"Failed to read frame ({consecutive_failures}/{max_consecutive_failures})")
                else:
                    consecutive_failures += 1
                    self.get_logger().warning(f"GStreamer pipeline not ready ({consecutive_failures}/{max_consecutive_failures})")
                    
                # Handle reconnection
                if consecutive_failures >= max_consecutive_failures:
                    self.get_logger().warning("Reconnecting GStreamer pipeline...")
                    self.release_gstreamer_pipeline()  # Proper cleanup
                    time.sleep(reconnect_delay)  # Longer delay for cleanup
                    self.setup_rtsp()
                    consecutive_failures = 0
                    
                # Prevent busy-waiting
                time.sleep(0.01)
                
            except Exception as e:
                self.get_logger().error(f"Error in GStreamer streaming: {str(e)}")
                consecutive_failures += 1
                time.sleep(0.1)

    def udp_receiver(self):
        while rclpy.ok():
            try:
                if self.udp_socket:
                    data, addr = self.udp_socket.recvfrom(1024)
                    if data:
                        hex_data = binascii.hexlify(data).decode('utf-8')
                        # Split the data into messages
                        messages = self.split_messages(hex_data)
                        for message in messages:
                            self.process_response(message)
            except socket.timeout:
                pass
            except Exception as e:
                self.get_logger().error(f"UDP receiver error: {str(e)}")
            time.sleep(0.01)

    def split_messages(self, hex_data):
        """Split concatenated SIYI messages by finding all 5566 headers"""
        messages = []
        # Find all occurrences of the header
        header_positions = self.find_all_occurrences(hex_data, self.siyi_msg.HEADER)
        
        # Extract each message
        for i in range(len(header_positions)):
            start = header_positions[i]
            # If this is the last header, get all remaining data
            if i == len(header_positions) - 1:
                end = len(hex_data)
            else:
                end = header_positions[i + 1]
            
            message = hex_data[start:end]
            # Only add if it's a potentially complete message (at least minimum length)
            if len(message) >= self.siyi_msg.MINIMUM_DATA_LENGTH:
                messages.append(message)
        
        return messages

    def find_all_occurrences(self, string, substring):
        """Find all occurrences of a substring in a string"""
        positions = []
        start = 0
        while True:
            start = string.find(substring, start)
            if start == -1:
                break
            positions.append(start)
            start += len(substring)  # Move past this occurrence
        return positions

    def send_command(self, command_hex):
        try:
            if not self.udp_socket:
                self.get_logger().error("UDP socket not initialized")
                return False

            command_bytes = bytes.fromhex(command_hex)
            self.udp_socket.sendto(command_bytes, (self.camera_ip, self.command_port))
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to send command: {str(e)}")
            return False

    def process_response(self, response_hex):
        try:
            data, data_len, cmd_id, seq = self.siyi_msg.decode_msg(response_hex)
            if cmd_id == self.siyi_msg.ACQUIRE_FIRMWARE_VERSION:
                self.get_logger().info(f"Firmware version: {data}")
            elif cmd_id == self.siyi_msg.ACQUIRE_HARDWARE_ID:
                if data_len > 0:
                    self.get_logger().info(f"Gimbal info: {data}")
            elif cmd_id == self.siyi_msg.ACQUIRE_GIMBAL_ATTITUDE:
                if len(data) >= 12:
                    # Add this line to see the raw data
                    self.get_logger().info(f"Raw gimbal attitude hex: {data} (yaw: {data[0:4]}, pitch: {data[4:8]}, roll: {data[8:12]})")
                    
                    # Handle signed integers properly
                    yaw_raw = int(data[0:4], 16)
                    if yaw_raw > 0x7FFF:
                        yaw_raw -= 0x10000
                        
                    pitch_raw = int(data[4:8], 16)
                    if pitch_raw > 0x7FFF:
                        pitch_raw -= 0x10000
                        
                    roll_raw = int(data[8:12], 16)
                    if roll_raw > 0x7FFF:
                        roll_raw -= 0x10000
                    
                    # Convert to degrees
                    yaw = yaw_raw / 10.0
                    pitch = pitch_raw / 10.0
                    roll = roll_raw / 10.0
                    
                    # Add debug logging for values before they're published
                    self.get_logger().debug(f"Parsed gimbal angles: yaw={yaw:.1f}°, pitch={pitch:.1f}°, roll={roll:.1f}°")
                    
                    self.gimbal_angles['yaw'] = yaw
                    self.gimbal_angles['pitch'] = pitch
                    self.gimbal_angles['roll'] = roll
                    state_msg = Float32MultiArray()
                    state_msg.data = [yaw, pitch, roll]
                    self.gimbal_state_pub.publish(state_msg)
            elif cmd_id == self.siyi_msg.ACQUIRE_MAX_ZOOM:
                if data_len > 0:
                    max_zoom = int(data, 16)
                    self.get_logger().info(f"Maximum zoom level: {max_zoom}x")
            elif cmd_id == self.siyi_msg.FUNCTION_FEEDBACK_INFO:
                if data_len > 0:
                    self.get_logger().info(f"Function feedback: {data}")
            else:
                self.get_logger().debug(f"Received response for command {cmd_id}: {data}")
        except Exception as e:
            self.get_logger().error(f"Error processing response: {str(e)}")

    def update_gimbal_info(self):
        if self.connected:
            self.send_command(self.siyi_msg.gimbal_attitude_msg())

    def check_connection(self):
        if not self.connected:
            self.get_logger().info("Trying to reconnect to SIYI A8 Mini...")
            self.init_connections()
        else:
            if not self.send_command(self.siyi_msg.gimbal_info_msg()):
                self.connected = False
                self.get_logger().warning("Lost connection to SIYI A8 Mini")

    def gimbal_control_callback(self, msg):
        self.get_logger().info(f"Received gimbal control: {msg}")
        yaw_speed = int(msg.angular.z)
        pitch_speed = int(msg.angular.y)
        command = self.siyi_msg.gimbal_speed_msg(yaw_speed, pitch_speed)
        self.get_logger().info(f"Sending gimbal control command: {command}")
        self.send_command(command)

    def gimbal_angle_callback(self, msg):
        yaw_angle = float(msg.angular.z)
        pitch_angle = float(msg.angular.y)
        command = self.siyi_msg.gimbal_angles_msg(yaw_angle, pitch_angle)
        self.send_command(command)

    def zoom_control_callback(self, msg):
        control_type = int(msg.data[0])
        if control_type == 0:
            command = self.siyi_msg.zoom_halt_msg()
        elif control_type == 1:
            command = self.siyi_msg.zoom_in_msg()
        elif control_type == 2:
            command = self.siyi_msg.zoom_out_msg()
        elif control_type == 3:
            integer_part = int(msg.data[1])
            fractional_part = int(msg.data[2])
            command = self.siyi_msg.absolute_zoom_msg(integer_part, fractional_part)
        else:
            self.get_logger().warning(f"Unknown zoom control type: {control_type}")
            return
        self.send_command(command)

    def command_callback(self, msg):
        command_str = msg.data.lower()
        command = None
        if command_str == "center":
            command = self.siyi_msg.gimbal_center_msg()
        elif command_str == "autofocus":
            command = self.siyi_msg.autofocus_msg()
        elif command_str == "photo":
            command = self.siyi_msg.photo_msg()
        elif command_str == "record":
            command = self.siyi_msg.record_msg()
            self.is_recording = not self.is_recording
        elif command_str == "lock_mode":
            command = self.siyi_msg.lock_mode_msg()
            self.gimbal_mode = "lock"
        elif command_str == "follow_mode":
            command = self.siyi_msg.follow_mode_msg()
            self.gimbal_mode = "follow"
        elif command_str == "fpv_mode":
            command = self.siyi_msg.fpv_mode_msg()
            self.gimbal_mode = "fpv"
        elif command_str == "straight_down":
            command = self.siyi_msg.straight_down_msg()
        else:
            self.get_logger().warning(f"Unknown command: {command_str}")
            return
        self.send_command(command)

    def publish_camera_info(self, header):
        camera_info_msg = CameraInfo()
        camera_info_msg.header = header
        camera_info_msg.width = 1280
        camera_info_msg.height = 720

        # Camera matrix K [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        camera_info_msg.k = [
            743.761942, 0.0, 621.138957,
            0.0, 738.716957, 382.489783,
            0.0, 0.0, 1.0
        ]
        
        # Projection matrix P [fx, 0, cx, Tx, 0, fy, cy, Ty, 0, 0, 1, 0]
        camera_info_msg.p = [
            745.416443, 0.0, 629.900108, 0.0,
            0.0, 743.304504, 387.554254, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        # Rectification matrix R [1, 0, 0, 0, 1, 0, 0, 0, 1]
        camera_info_msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        
        # Distortion model and coefficients
        camera_info_msg.distortion_model = "plumb_bob"
        camera_info_msg.d = [-0.051467, 0.051758, 0.005913, 0.004882, 0.000000]

        self.camera_info_pub.publish(camera_info_msg)

    def cleanup(self):
        """Cleanup all resources"""
        self.stop_threads = True
        time.sleep(0.5)  # Give threads time to exit cleanly
        self.release_gstreamer_pipeline()
        if self.udp_socket:
            self.udp_socket.close()
        self.get_logger().info("SIYI A8 Mini camera node cleanup complete")

def main(args=None):
    rclpy.init(args=args)
    siyi_camera_node = SIYICameraNode()
    try:
        rclpy.spin(siyi_camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        siyi_camera_node.cleanup()
        siyi_camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

