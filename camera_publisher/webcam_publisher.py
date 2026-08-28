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

        # Modo de captura pedido ao driver V4L. 0 = nao mexe, aceita o default.
        # Sem isto o no publicava no que quer que a camera tivesse escolhido no
        # boot, e camera_width/camera_height do config de visao viravam chute.
        self.declare_parameter('capture_width', 0)
        self.declare_parameter('capture_height', 0)

        # Telemetry-tunable output size (square center-crop then resize).
        # Lower values reduce telemetry bandwidth significantly.
        #
        # PRECISAM SER IGUAIS. O recorte e para QUADRADO, e quem consome a
        # imagem (CameraCalibration::fromHorizontalFov) descreve os intrinsecos
        # com um LADO UNICO, nao com largura e altura. Pedir 640x480 aqui
        # esticava o recorte 1,33x na horizontal e deixava o centro optico
        # calculado em 240 quando o real era 320 -- sem erro nenhum a jusante.
        # Ver a checagem logo abaixo.
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 640)

        # Telemetry-tunable publish rate (Hz). Lower => less bandwidth.
        self.declare_parameter('publish_rate', 10.0)

        # JPEG quality (only used when use_compressed=True). 1..100.
        self.declare_parameter('jpeg_quality', 80)

        camera_name = self.get_parameter('camera_name').value
        self.use_compressed = bool(self.get_parameter('use_compressed').value)
        video_source = self.get_parameter('video_source').value
        horizontal_flip = bool(self.get_parameter('horizontal_flip').value)
        vertical_flip = bool(self.get_parameter('vertical_flip').value)
        capture_width = int(self.get_parameter('capture_width').value)
        capture_height = int(self.get_parameter('capture_height').value)
        self.frame_width = int(self.get_parameter('frame_width').value)
        self.frame_height = int(self.get_parameter('frame_height').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        # Melhor falhar aqui, na subida, do que em voo: imagem esticada nao
        # aparece como falha, aparece como o drone alinhando ao lado da base.
        if self.frame_width != self.frame_height:
            raise ValueError(
                f'frame_width ({self.frame_width}) != frame_height '
                f'({self.frame_height}): o recorte e para QUADRADO e os '
                f'intrinsecos a jusante supoem um lado unico. Redimensionar '
                f'para retangulo estica a imagem e desloca o centro optico.')

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

        # O driver pode NEGAR o modo pedido e entregar outro sem avisar, entao
        # o que vale e ler de volta o que ficou. E esse numero que precisa
        # estar em camera_width/camera_height do config de visao.
        if capture_width > 0 and capture_height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, capture_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, capture_height)

        efetiva_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        efetiva_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        pedido = (f'{capture_width}x{capture_height}' if capture_width > 0
                  else 'default do driver')
        self.get_logger().info(
            f'Captura efetiva: {efetiva_w}x{efetiva_h} (pedido: {pedido})')

        if capture_width > 0 and (efetiva_w, efetiva_h) != (capture_width, capture_height):
            self.get_logger().warn(
                f'O driver NEGOU {capture_width}x{capture_height} e entregou '
                f'{efetiva_w}x{efetiva_h}. Atualize camera_width/camera_height '
                f'no config de visao para {efetiva_w}/{efetiva_h}, senao os '
                f'intrinsecos descrevem uma imagem que nao existe.')

        # Recorte menor que a saida e so ampliacao: nao cria detalhe, custa
        # banda e engana quem le o topico.
        lado_do_recorte = min(efetiva_w, efetiva_h)
        if lado_do_recorte > 0 and self.frame_width > lado_do_recorte:
            self.get_logger().warn(
                f'Publicando {self.frame_width}x{self.frame_width} a partir de '
                f'um recorte de {lado_do_recorte}x{lado_do_recorte}: e '
                f'ampliacao, nao detalhe. Para ganhar resolucao real, use um '
                f'modo de captura cuja MENOR dimensao seja >= {self.frame_width}.')

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
