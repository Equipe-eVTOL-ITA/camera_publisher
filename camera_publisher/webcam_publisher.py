import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'camera/compressed', 10)
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
            
            msg = self.bridge.cv2_to_compressed_imgmsg(frame_resized)
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
