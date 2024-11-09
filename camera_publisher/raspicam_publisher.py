import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'camera/image/compressed', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            #frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            msg = self.bridge.cv2_to_compressed_imgmsg(frame) # Using raw img: (frame_rgb, encoding='rgb8')
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
