import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import depthai as dai
import cv2

class OakRGBPublisher(Node):
    def __init__(self):
        super().__init__('oak_rgb_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'camera/image/compressed', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish at 10 Hz
        self.bridge = CvBridge()

        # Initialize DepthAI pipeline
        self.pipeline = dai.Pipeline()

        # Define OAK-D RGB camera source
        self.cam_rgb = self.pipeline.createColorCamera()
        self.cam_rgb.setPreviewSize(640, 480)
        self.cam_rgb.setInterleaved(False)
        self.cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)

        # Link the RGB output to XLink
        self.xout_rgb = self.pipeline.createXLinkOut()
        self.xout_rgb.setStreamName("rgb")
        self.cam_rgb.preview.link(self.xout_rgb.input)

        # Start DepthAI device
        self.device = dai.Device(self.pipeline)
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

    def timer_callback(self):
        in_rgb = self.q_rgb.tryGet()  # Non-blocking call to get the latest RGB frame
        if in_rgb:
            frame = in_rgb.getCvFrame()  # Convert to OpenCV format

            # Convert the OpenCV frame to a compressed ROS 2 image message
            msg = self.bridge.cv2_to_compressed_imgmsg(frame)
            self.publisher_.publish(msg)
            self.get_logger().info('Published image frame')

def main(args=None):
    rclpy.init(args=args)
    oak_rgb_publisher = OakRGBPublisher()
    rclpy.spin(oak_rgb_publisher)
    oak_rgb_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
