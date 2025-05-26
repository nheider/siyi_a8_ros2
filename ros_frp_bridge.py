import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class GStreamerStreamer(Node):
    def __init__(self):
        super().__init__('gstreamer_streamer')
        self.subscription = self.create_subscription(
            Image,
            'siyi_a8/image_raw/compressed',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

        # GStreamer pipeline to send video over TCP
        gst_str = (
            'appsrc ! videoconvert ! video/x-raw,format=I420,width=640,height=480,framerate=30/1 '
            '! x264enc tune=zerolatency bitrate=500 speed-preset=ultrafast '
            '! mpegtsmux ! tcpclientsink host=127.0.0.1 port=1234'
        )
        self.out = cv2.VideoWriter(
            gst_str,
            cv2.CAP_GSTREAMER,
            0,  # fourcc (not used)
            30.0,
            (640, 480)
        )
        if not self.out.isOpened():
            self.get_logger().error('Failed to open GStreamer pipeline')
            raise RuntimeError("Failed to open GStreamer pipeline")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.resize(frame, (640, 480))  # Ensure expected size
            self.out.write(frame)
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GStreamerStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
