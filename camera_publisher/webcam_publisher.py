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

        # Telemetry-tunable output size (center-crop to this aspect ratio,
        # then resize). Lower values reduce telemetry bandwidth significantly.
        self.declare_parameter('frame_width', 800)
        self.declare_parameter('frame_height', 800)

        # Telemetry-tunable publish rate (Hz). Lower => less bandwidth.
        self.declare_parameter('publish_rate', 10.0)

        # JPEG quality (only used when use_compressed=True). 1..100.
        self.declare_parameter('jpeg_quality', 80)

        # Manual exposure/gain, to trade noise for less motion blur (helps
        # rolling shutter smear during fast maneuvers, but does not remove
        # the geometric skew itself). exposure_us <= 0 or gain < 0 leaves
        # that control on auto.
        self.declare_parameter('auto_exposure', True)
        self.declare_parameter('exposure_us', -1)
        self.declare_parameter('gain', -1)

        camera_name = self.get_parameter('camera_name').value
        self.use_compressed = bool(self.get_parameter('use_compressed').value)
        video_source = self.get_parameter('video_source').value
        horizontal_flip = bool(self.get_parameter('horizontal_flip').value)
        vertical_flip = bool(self.get_parameter('vertical_flip').value)
        self.frame_width = int(self.get_parameter('frame_width').value)
        self.frame_height = int(self.get_parameter('frame_height').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.auto_exposure = bool(self.get_parameter('auto_exposure').value)
        self.exposure_us = int(self.get_parameter('exposure_us').value)
        self.gain = int(self.get_parameter('gain').value)

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
        # Ask the driver for the target resolution up front, same as
        # webcam_test.py/calibrate.py's --calibrar does -- when the camera
        # honours it, the crop below becomes a no-op and what reaches
        # detection.py in flight is pixel-for-pixel what was calibrated.
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self._configure_exposure()

    def _configure_exposure(self):
        # OpenCV's V4L2 backend maps CAP_PROP_AUTO_EXPOSURE to the V4L2
        # "exposure_auto" menu: 3 = auto, 1 = manual.
        if self.auto_exposure:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
            return

        if not self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1):
            self.get_logger().warn("Camera did not accept manual exposure mode")

        if self.exposure_us > 0:
            # CAP_PROP_EXPOSURE units are driver-dependent; most UVC drivers
            # take units of 100us.
            if not self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure_us / 100.0):
                self.get_logger().warn("Failed to set exposure_us")

        if self.gain >= 0:
            # Most UVC webcams expose "gain" rather than a direct ISO control.
            if not self.cap.set(cv2.CAP_PROP_GAIN, self.gain):
                self.get_logger().warn("Failed to set gain")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        h, w = frame.shape[:2]

        # Center-crop to the TARGET aspect ratio (not necessarily square)
        # before resizing. Cropping to a square and then resizing to a
        # non-square target (e.g. 640x480) stretched the image by a fixed
        # 4:3 factor on every frame -- turning real circles into ellipses
        # in flight while calibration tools (webcam_test.py, which reads the
        # camera directly with no crop) never saw that distortion. Cropping
        # to the target ratio first makes the resize a uniform scale, so
        # there's no warp regardless of the camera's native resolution.
        target_ratio = self.frame_width / self.frame_height
        src_ratio = w / h
        if src_ratio > target_ratio:
            new_w = int(round(h * target_ratio))
            start_x = (w - new_w) // 2
            frame_cropped = frame[:, start_x:start_x + new_w]
        else:
            new_h = int(round(w / target_ratio))
            start_y = (h - new_h) // 2
            frame_cropped = frame[start_y:start_y + new_h, :]

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
