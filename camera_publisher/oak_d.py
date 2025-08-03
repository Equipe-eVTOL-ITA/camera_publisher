import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import depthai as dai
import cv2

class OakRGBPublisher(Node):
    def __init__(self):
        super().__init__('oak_rgb_publisher')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('publish_raw', True),
                ('publish_compressed', False),
                ('width', 800),
                ('height', 800),
                ('fps', 20),
                ('auto_exposure_compensation', 4),
            ]
        )
        
        # Get parameters with proper type casting and default values
        width_param = self.get_parameter('width').value
        height_param = self.get_parameter('height').value
        fps_param = self.get_parameter('fps').value
        exp_param = self.get_parameter('auto_exposure_compensation').value
        
        self.publish_raw = bool(self.get_parameter('publish_raw').value)
        self.publish_compressed = bool(self.get_parameter('publish_compressed').value)
        self.width = int(width_param) if width_param is not None else 800
        self.height = int(height_param) if height_param is not None else 800
        self.fps = int(fps_param) if fps_param is not None else 20
        self.auto_exposure_compensation = int(exp_param) if exp_param is not None else 4
        
        # Create publishers based on parameters
        if self.publish_raw:
            self.raw_publisher = self.create_publisher(Image, 'camera/image/raw', 10)
        
        if self.publish_compressed:
            self.compressed_publisher = self.create_publisher(CompressedImage, 'camera/image/compressed', 10)
            
        # Timer frequency based on FPS parameter
        timer_period = 1.0 / float(self.fps)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()

        # Initialize DepthAI pipeline
        self.pipeline = dai.Pipeline()

        # Define OAK-D RGB camera source
        self.cam_rgb = self.pipeline.createColorCamera()
        self.cam_rgb.setPreviewSize(self.width, self.height)
        self.cam_rgb.setInterleaved(False)
        self.cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

        self.cam_rgb.initialControl.setAutoExposureEnable()
        self.cam_rgb.initialControl.setAutoExposureCompensation(self.auto_exposure_compensation)

        # Link the RGB output to XLink
        self.xout_rgb = self.pipeline.createXLinkOut()
        self.xout_rgb.setStreamName("rgb")
        self.cam_rgb.preview.link(self.xout_rgb.input)

        # Start DepthAI device
        self.device = dai.Device(self.pipeline)
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        
        self.get_logger().info(f"OAK-D Publisher initialized: {self.width}x{self.height} @ {self.fps} FPS")
        self.get_logger().info(f"Publishing: raw={self.publish_raw}, compressed={self.publish_compressed}")

    def timer_callback(self):
        in_rgb = self.q_rgb.tryGet()  # Non-blocking call to get the latest RGB frame
        if in_rgb:
            frame = in_rgb.getCvFrame()  # Convert to OpenCV format

            # Create timestamp
            timestamp = self.get_clock().now().to_msg()
            
            # Publish raw image if enabled
            if self.publish_raw and hasattr(self, 'raw_publisher'):
                raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                raw_msg.header.stamp = timestamp
                raw_msg.header.frame_id = "camera_link"
                self.raw_publisher.publish(raw_msg)
            
            # Publish compressed image if enabled
            if self.publish_compressed and hasattr(self, 'compressed_publisher'):
                compressed_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
                compressed_msg.header.stamp = timestamp
                compressed_msg.header.frame_id = "camera_link"
                self.compressed_publisher.publish(compressed_msg)
                

def main(args=None):
    rclpy.init(args=args)
    oak_rgb_publisher = OakRGBPublisher()
    rclpy.spin(oak_rgb_publisher)
    oak_rgb_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
