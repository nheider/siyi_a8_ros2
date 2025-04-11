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
            
        # Services (optional)
        # self.create_service(...)
        
        # Timers
        self.create_timer(1.0 / self.update_rate, self.update_gimbal_info)
        self.create_timer(5.0, self.check_connection)
        
        # Start connection to camera
        self.get_logger().info(f"Connecting to SIYI A8 Mini at {self.camera_ip}")
        self.init_connections()
        
    def init_connections(self):
        """
        Initialize connections to the camera
        """
        try:
            # Setup UDP socket for command and control
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.settimeout(1.0)  # 1 second timeout
            
            # Start UDP receiver thread
            self.udp_thread = threading.Thread(target=self.udp_receiver)
            self.udp_thread.daemon = True
            self.udp_thread.start()
            
            # Setup RTSP connection for video streaming
            self.setup_rtsp()
            
            # Request initial camera information
            self.send_command(self.siyi_msg.firmware_version_msg())
            self.send_command(self.siyi_msg.gimbal_info_msg())
            self.send_command(self.siyi_msg.maximum_zoom_msg())
            
            self.connected = True
            self.get_logger().info("Connected to SIYI A8 Mini camera")
            
        except Exception as e:
            self.get_logger().error(f"Failed to connect to camera: {str(e)}")
            self.connected = False
    
    def setup_rtsp(self):
        """
        Setup RTSP connection for video streaming
        """
        try:

            # RTSP URL for SIYI camera
            rtsp_url = f"rtsp://{self.camera_ip}:{self.rtsp_port}/main.264"
            gst_pipeline = f"rtspsrc location={rtsp_url} latency=100 ! decodebin ! videoconvert ! appsink"
            
            # Open video capture
	    self.rtsp_camera = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            if not self.rtsp_camera.isOpened():
                self.get_logger().error("Failed to open RTSP stream")
                return False
                
            # Start RTSP streaming thread
            self.rtsp_thread = threading.Thread(target=self.rtsp_streaming)
            self.rtsp_thread.daemon = True
            self.rtsp_thread.start()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"RTSP setup error: {str(e)}")
            return False
    
    def rtsp_streaming(self):
        """
        Thread function for RTSP streaming
        """
        while rclpy.ok():
            try:
                if self.rtsp_camera and self.rtsp_camera.isOpened():
                    ret, frame = self.rtsp_camera.read()
                    if ret:
                        # Convert frame to ROS message
                        img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                        img_msg.header.stamp = self.get_clock().now().to_msg()
                        img_msg.header.frame_id = "siyi_camera_optical_frame"
                        
                        # Publish image
                        self.image_pub.publish(img_msg)
                        
                        # Also publish camera info
                        self.publish_camera_info(img_msg.header)
                    else:
                        self.get_logger().warning("Failed to read frame from RTSP stream")
                        # Try to reconnect
                        self.rtsp_camera.release()
                        time.sleep(1.0)
                        self.setup_rtsp()
                else:
                    self.get_logger().warning("RTSP camera not opened, trying to reconnect")
                    self.setup_rtsp()
                    time.sleep(1.0)
                    
            except Exception as e:
                self.get_logger().error(f"RTSP streaming error: {str(e)}")
                # Try to reconnect
                if self.rtsp_camera:
                    self.rtsp_camera.release()
                time.sleep(1.0)
                self.setup_rtsp()
                
            # Sleep briefly to avoid high CPU usage
            time.sleep(0.01)
    
    def udp_receiver(self):
        """
        Thread function for UDP command responses
        """
        while rclpy.ok():
            try:
                if self.udp_socket:
                    data, addr = self.udp_socket.recvfrom(1024)
                    if data:
                        # Convert bytes to hex string
                        hex_data = binascii.hexlify(data).decode('utf-8')
                        # Process the message
                        self.process_response(hex_data)
            except socket.timeout:
                # Timeout is normal
                pass
            except Exception as e:
                self.get_logger().error(f"UDP receiver error: {str(e)}")
                
            # Sleep briefly to avoid high CPU usage
            time.sleep(0.01)
    
    def send_command(self, command_hex):
        """
        Send a command to the camera
        
        Args:
            command_hex: Hex string command to send
        """
        try:
            if not self.udp_socket:
                self.get_logger().error("UDP socket not initialized")
                return False
                
            # Convert hex string to bytes
            command_bytes = bytes.fromhex(command_hex)
            # Send command
            self.udp_socket.sendto(command_bytes, (self.camera_ip, self.command_port))
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {str(e)}")
            return False
    
    def process_response(self, response_hex):
        """
        Process response from the camera
        
        Args:
            response_hex: Hex string response from camera
        """
        try:
            # Decode the message
            data, data_len, cmd_id, seq = self.siyi_msg.decode_msg(response_hex)
            
            # Process based on command ID
            if cmd_id == ACQUIRE_FIRMWARE_VERSION:
                self.get_logger().info(f"Firmware version: {data}")
                
            elif cmd_id == ACQUIRE_GIMBAL_INFO:
                # Parse gimbal info
                if data_len > 0:
                    # Process gimbal info - format depends on specific model
                    self.get_logger().info(f"Gimbal info: {data}")
                    
            elif cmd_id == ACQUIRE_GIMBAL_ATTITUDE:
                # Parse gimbal attitude
                if len(data) >= 12:  # 6 bytes for 3 angles
                    # Extract yaw, pitch, roll
                    yaw = int(data[0:4], 16) / 10.0
                    pitch = int(data[4:8], 16) / 10.0
                    roll = int(data[8:12], 16) / 10.0
                    
                    self.gimbal_angles['yaw'] = yaw
                    self.gimbal_angles['pitch'] = pitch
                    self.gimbal_angles['roll'] = roll
                    
                    # Publish gimbal state
                    state_msg = Float32MultiArray()
                    state_msg.data = [yaw, pitch, roll]
                    self.gimbal_state_pub.publish(state_msg)
                    
            elif cmd_id == ACQUIRE_MAX_ZOOM:
                # Parse max zoom level
                if data_len > 0:
                    max_zoom = int(data, 16)
                    self.get_logger().info(f"Maximum zoom level: {max_zoom}x")
                    
            elif cmd_id == FUNCTION_FEEDBACK_INFO:
                # Parse function feedback
                if data_len > 0:
                    # Process function feedback - format depends on specific model
                    self.get_logger().info(f"Function feedback: {data}")
                    
            else:
                # Other command response
                self.get_logger().debug(f"Received response for command {cmd_id}: {data}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing response: {str(e)}")
    
    def update_gimbal_info(self):
        """
        Timer callback to request gimbal information
        """
        if self.connected:
            # Request gimbal attitude
            self.send_command(self.siyi_msg.gimbal_attitude_msg())
    
    def check_connection(self):
        """
        Timer callback to check and maintain connection
        """
        if not self.connected:
            self.get_logger().info("Trying to reconnect to SIYI A8 Mini...")
            self.init_connections()
        else:
            # Send a simple command to verify connection
            if not self.send_command(self.siyi_msg.gimbal_info_msg()):
                self.connected = False
                self.get_logger().warning("Lost connection to SIYI A8 Mini")
    
    def gimbal_control_callback(self, msg):
        """
        Callback for gimbal speed control
        
        Args:
            msg: Twist message for gimbal control
                angular.z = yaw speed (-100 to 100)
                angular.y = pitch speed (-100 to 100)
        """
        yaw_speed = int(msg.angular.z)
        pitch_speed = int(msg.angular.y)
        
        command = self.siyi_msg.gimbal_speed_msg(yaw_speed, pitch_speed)
        self.send_command(command)
    
    def gimbal_angle_callback(self, msg):
        """
        Callback for gimbal angle control
        
        Args:
            msg: Twist message for gimbal angles
                angular.z = yaw angle (-135 to 135 degrees)
                angular.y = pitch angle (-90 to 25 degrees)
        """
        yaw_angle = float(msg.angular.z)
        pitch_angle = float(msg.angular.y)
        
        command = self.siyi_msg.gimbal_angles_msg(yaw_angle, pitch_angle)
        self.send_command(command)
    
    def zoom_control_callback(self, msg):
        """
        Callback for zoom control
        
        Args:
            msg: Float32MultiArray
                data[0] = zoom control type (0=stop, 1=in, 2=out, 3=absolute)
                data[1] = zoom value (for absolute zoom, integer part)
                data[2] = zoom value (for absolute zoom, fractional part)
        """
        control_type = int(msg.data[0])
        
        if control_type == 0:  # Stop zooming
            command = self.siyi_msg.zoom_halt_msg()
        elif control_type == 1:  # Zoom in
            command = self.siyi_msg.zoom_in_msg()
        elif control_type == 2:  # Zoom out
            command = self.siyi_msg.zoom_out_msg()
        elif control_type == 3:  # Absolute zoom
            integer_part = int(msg.data[1])
            fractional_part = int(msg.data[2])
            command = self.siyi_msg.absolute_zoom_msg(integer_part, fractional_part)
        else:
            self.get_logger().warning(f"Unknown zoom control type: {control_type}")
            return
            
        self.send_command(command)
    
    def command_callback(self, msg):
        """
        Callback for generic commands
        
        Args:
            msg: String message with command name
                e.g. "center", "autofocus", "photo", "record", etc.
        """
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
        """
        Publish camera info message
        
        Args:
            header: Header to use for the message
        """
        camera_info_msg = CameraInfo()
        camera_info_msg.header = header
        camera_info_msg.width = self.image_width
        camera_info_msg.height = self.image_height
        
        # Set camera parameters (focal length, principal point, etc.)
        # These are approximate values - you may need to calibrate your camera
        # for accurate parameters
        fx = self.image_width * 0.85  # Approximate focal length
        fy = self.image_width * 0.85
        cx = self.image_width / 2.0   # Principal point
        cy = self.image_height / 2.0
        
        # Camera matrix (K)
        camera_info_msg.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]
        
        # Projection matrix (P)
        camera_info_msg.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        # Identity rotation matrix (R)
        camera_info_msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        
        # No distortion (use this as a starting point)
        camera_info_msg.distortion_model = "plumb_bob"
        camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.camera_info_pub.publish(camera_info_msg)
        
    def cleanup(self):
        """
        Clean up resources before node shutdown
        """
        if self.rtsp_camera and self.rtsp_camera.isOpened():
            self.rtsp_camera.release()
            
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
