from camera_publisher.topicos import nome_do_topico

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Declare parameters
        self.declare_parameter('use_compressed', False)  # False = raw, True = compressed
        # O PAPEL da camera no drone, e nao o modelo do hardware. Trocar uma
        # Raspberry Pi Camera por uma webcam na mesma posicao nao pode mudar o
        # topico -- senao todo config que o consome quebra em silencio.
        self.declare_parameter('camera_name', 'vertical')

        # Get parameters
        self.use_compressed = self.get_parameter('use_compressed').value
        camera_name = self.get_parameter('camera_name').value
        
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
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('/dev/video3', cv2.CAP_V4L)

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

            # Convert based on parameter
            if self.use_compressed:
                msg = self.bridge.cv2_to_compressed_imgmsg(frame_resized)
            else:
                msg = self.bridge.cv2_to_imgmsg(frame_resized, encoding='bgr8')
            
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
