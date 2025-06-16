import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import socket
import time

class GStreamerStreamer(Node):
    def __init__(self):
        super().__init__('gstreamer_streamer')
        
        # Parameters for FRP client connection
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 1234)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('bitrate', 500)
        
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        bitrate = self.get_parameter('bitrate').value
        
        self.subscription = self.create_subscription(
            Image,
            'siyi_a8/image_raw/compressed',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        
        # Check if port is already in use (optional safety check)
        if self._is_port_in_use(host, port):
            raise RuntimeError(f"Port {port} is already in use at {host}")
        
        # GStreamer pipeline for TCP server
        gst_str = (
            f'appsrc ! videoconvert ! video/x-raw,format=I420,width={width},height={height},framerate={fps}/1 '
            f'! x264enc tune=zerolatency bitrate={bitrate} speed-preset=ultrafast '
            f'! mpegtsmux ! tcpserversink host={host} port={port}'
        )
        
        self.get_logger().info(f"Starting TCP server on {host}:{port}")
        self.get_logger().info(f"Using GStreamer pipeline: {gst_str}")
        
        self.out = cv2.VideoWriter(
            gst_str,
            cv2.CAP_GSTREAMER,
            0,  # fourcc (not used)
            float(fps),
            (width, height)
        )
        
        if not self.out.isOpened():
            self.get_logger().error('Failed to open GStreamer pipeline')
            self.get_logger().error('Make sure GStreamer is properly installed')
            raise RuntimeError("Failed to open GStreamer pipeline")
        
        self.get_logger().info(f'Successfully started TCP server on {host}:{port}')
        self.get_logger().info('Video stream will be available when clients connect')
    
    def _is_port_in_use(self, host, port):
        """Check if port is already in use"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            result = sock.connect_ex((host, port))
            sock.close()
            return result == 0
        except Exception:
            return False
    
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Get target dimensions
            width = self.get_parameter('width').value
            height = self.get_parameter('height').value
            frame = cv2.resize(frame, (width, height))
            
            # Write frame to GStreamer pipeline
            if self.out.isOpened():
                self.out.write(frame)
            else:
                self.get_logger().error("GStreamer pipeline is not opened")
                
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")
    
    def __del__(self):
        if hasattr(self, 'out') and self.out.isOpened():
            self.out.release()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = GStreamerStreamer()
        node.get_logger().info("FRP video bridge node started")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Failed to create node: {e}")
        print("\nTroubleshooting tips:")
        print("1. Make sure frpc is running to expose the TCP server")
        print("2. Check your frpc configuration for the correct local port")
        print("3. Verify GStreamer plugins are installed: 'gst-inspect-1.0 | grep tcp'")
        print("4. Use custom port: --ros-args -p port:=YOUR_FRP_PORT")
        print("5. Check if port is already in use with: ss -tlnp | grep 1234")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
