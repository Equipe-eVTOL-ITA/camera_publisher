import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        
        # Declare parameters
        self.declare_parameter('camera_name', 'vertical')  # e.g., 'vertical', 'horizontal'
        self.declare_parameter('use_compressed', True)  # False = raw, True = compressed
        self.declare_parameter('video_source', '/dev/video0')  # Video device path
        self.declare_parameter('horizontal_flip', False)  # Flip horizontally
        self.declare_parameter('vertical_flip', False)  # Flip vertically
        
        # Get parameters
        camera_name = self.get_parameter('camera_name').value
        self.use_compressed = self.get_parameter('use_compressed').value
        video_source = self.get_parameter('video_source').value
        horizontal_flip = self.get_parameter('horizontal_flip').value
        vertical_flip = self.get_parameter('vertical_flip').value
        
        # Determine flip code
        if horizontal_flip and vertical_flip:
            self.flip_code = -1
        elif horizontal_flip:
            self.flip_code = 1
        elif vertical_flip:
            self.flip_code = 0
        else:
            self.flip_code = None
        video_source = self.get_parameter('video_source').value
        horizontal_flip = self.get_parameter('horizontal_flip').value
        vertical_flip = self.get_parameter('vertical_flip').value
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        
        # Create publisher based on parameters
        if self.use_compressed:
            topic_name = f'{camera_name}_camera/compressed'
            self.publisher_ = self.create_publisher(CompressedImage, topic_name, qos_profile)
            self.get_logger().info(f"Publishing COMPRESSED images to: {topic_name}")
        else:
            topic_name = f'{camera_name}_camera/raw'
            self.publisher_ = self.create_publisher(Image, topic_name, qos_profile)
            self.get_logger().info(f"Publishing RAW images to: {topic_name}")
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(video_source, cv2.CAP_V4L)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Get frame dimensions
            h, w = frame.shape[:2]
            
            # Crop to square (center crop)
            if w > h:
                # Landscape: crop width
                start_x = (w - h) // 2
                frame_cropped = frame[:, start_x:start_x + h]
            else:
                # Portrait: crop height
                start_y = (h - w) // 2
                frame_cropped = frame[start_y:start_y + w, :]
            
            # Resize to 800x800
            frame_resized = cv2.resize(frame_cropped, (800, 800))

            # Flip based on parameters
            if self.flip_code is not None:
                frame_flipped = cv2.flip(frame_resized, self.flip_code)
            else:
                frame_flipped = frame_resized
            
            # Convert based on parameter
            if self.use_compressed:
                msg = self.bridge.cv2_to_compressed_imgmsg(frame_flipped)
            else:
                msg = self.bridge.cv2_to_imgmsg(frame_flipped, encoding='bgr8')

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
