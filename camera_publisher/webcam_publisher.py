from camera_publisher.topicos import nome_do_topico

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2


class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')

        # Identity / output format
        self.declare_parameter('camera_name', 'vertical')        # e.g., 'vertical', 'horizontal'
        self.declare_parameter('use_compressed', True)           # False = raw Image, True = CompressedImage
        self.declare_parameter('video_source', '/dev/video0')    # V4L device path

        # Orientation
        self.declare_parameter('horizontal_flip', False)
        self.declare_parameter('vertical_flip', False)

        # Telemetry-tunable output size (square center-crop then resize).
        # Lower values reduce telemetry bandwidth significantly.
        self.declare_parameter('frame_width', 800)
        self.declare_parameter('frame_height', 800)

        # Telemetry-tunable publish rate (Hz). Lower => less bandwidth.
        self.declare_parameter('publish_rate', 10.0)

        # JPEG quality (only used when use_compressed=True). 1..100.
        self.declare_parameter('jpeg_quality', 80)

        camera_name = self.get_parameter('camera_name').value
        self.use_compressed = bool(self.get_parameter('use_compressed').value)
        video_source = self.get_parameter('video_source').value
        horizontal_flip = bool(self.get_parameter('horizontal_flip').value)
        vertical_flip = bool(self.get_parameter('vertical_flip').value)
        self.frame_width = int(self.get_parameter('frame_width').value)
        self.frame_height = int(self.get_parameter('frame_height').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        if publish_rate <= 0.0:
            self.get_logger().warn(
                f'Invalid publish_rate={publish_rate}; falling back to 10.0 Hz')
            publish_rate = 10.0
        timer_period = 1.0 / publish_rate

        if horizontal_flip and vertical_flip:
            self.flip_code = -1
        elif horizontal_flip:
            self.flip_code = 1
        elif vertical_flip:
            self.flip_code = 0
        else:
            self.flip_code = None

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        topic_name = nome_do_topico(camera_name, self.use_compressed)
        tipo = CompressedImage if self.use_compressed else Image
        self.publisher_ = self.create_publisher(tipo, topic_name, qos_profile)
        self.get_logger().info(
            f"Publicando em: {topic_name} "
            f"({'comprimido' if self.use_compressed else 'cru'})")

        self.get_logger().info(
            f"Camera config: size={self.frame_width}x{self.frame_height} "
            f"@ {publish_rate:.2f} Hz, jpeg_q={self.jpeg_quality}, source={video_source}")

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(video_source, cv2.CAP_V4L)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        h, w = frame.shape[:2]

        if w > h:
            start_x = (w - h) // 2
            frame_cropped = frame[:, start_x:start_x + h]
        else:
            start_y = (h - w) // 2
            frame_cropped = frame[start_y:start_y + w, :]

        frame_resized = cv2.resize(frame_cropped, (self.frame_width, self.frame_height))

        if self.flip_code is not None:
            frame_out = cv2.flip(frame_resized, self.flip_code)
        else:
            frame_out = frame_resized

        if self.use_compressed:
            ok, buf = cv2.imencode('.jpg', frame_out,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not ok:
                return
            msg = CompressedImage()
            msg.format = 'jpeg'
            msg.data = buf.tobytes()
        else:
            msg = self.bridge.cv2_to_imgmsg(frame_out, encoding='bgr8')

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    rclpy.spin(webcam_publisher)
    webcam_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
