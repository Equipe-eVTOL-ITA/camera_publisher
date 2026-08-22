"""
Recorta uma ROI da camera e publica um stream leve para o COMPUTADOR DE SOLO.

POR QUE ESTE NO EXISTE

Na fase 3 da CBR 2026 o reconhecimento de gestos NAO e embarcado: a imagem sai
do drone e atravessa a rede ate o `gesture_detector`. Medido na Jetson com o
OAK-D publicando o par retificado para o cuVSLAM:

    /oak/left/image_rect             mono8 640x400, 256 KB/quadro, 30 Hz
    /oak/left/image_rect/compressed   64 KB/quadro, 30 Hz  ->  1,9 MB/s

1,9 MB/s e ~15 Mbps de enlace so para a imagem. E o `processing_frequency` do
detector NAO resolve isso: ele filtra no lado de SOLO, depois de o quadro ja ter
cruzado a rede. Com o detector a 8 Hz e a camera a 30 Hz, ~73% da banda e gasta
com quadros que sao descartados na chegada.

Este no corta as duas coisas, NO DRONE, antes da rede:

    taxa    30 Hz -> publish_rate (o termo dominante)
    pixels  quadro inteiro -> ROI
    JPEG    qualidade do driver -> jpeg_quality

O QUE ELE NAO PODE FAZER

O `/oak/left/image_rect` alimenta o cuVSLAM. Este no apenas SE INSCREVE nele --
nunca republica, nunca recorta, nunca altera nada no caminho do SLAM. Recortar
um quadro que vai para o VSLAM invalidaria a calibracao (o cx/cy do
camera_info deixaria de casar com a imagem) e destruiria a escala metrica, que
e exatamente o defeito que ja custou caro aqui uma vez.

Efeito colateral bom: o `/compressed` do driver depthai e um *lazy publisher*
(comprime so quando alguem assina). Quando o solo passar a assinar o topico
deste no, ninguem mais assina o do driver e a Jetson para de comprimir JPEG a
30 Hz -- CPU que volta para o cuVSLAM.

CUIDADO COM A ROI DESCENTRALIZADA

O `gesture_detector` publica o centroide da mao NORMALIZADO pelo quadro que
recebe, e os PIDs da fase 3 miram em 0.5 -- o centro da imagem. Com uma ROI fora
do centro, "0.5" passa a significar "centro do RECORTE", que nao e mais a frente
do drone. Por isso `roi_x`/`roi_y` valem -1 (centrado) por padrao: descentralize
so sabendo que o alvo dos PIDs vai junto.
"""

from camera_publisher.topicos import nome_do_topico

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class RoiStream(Node):

    def __init__(self):
        super().__init__('roi_stream')

        # ===== Entrada =====
        # Topico CRU, nao o /compressed. Assinar o comprimido obrigaria a Jetson
        # a comprimir a 30 Hz (lazy publisher) so para este no descomprimir e
        # comprimir de novo -- duas conversoes inuteis por quadro.
        self.declare_parameter('input_topic', '/oak/left/image_rect')

        # ===== Saida =====
        # O nome sai de topicos.py, como o de todo publicador deste pacote.
        # 'gesto' e nao 'frontal' de proposito: este stream e RECORTADO e
        # com taxa reduzida, e nao pode ser confundido com a vista completa da
        # camera frontal por quem for escrever um config depois.
        self.declare_parameter('camera_name', 'gesto')
        self.declare_parameter('jpeg_quality', 50)

        # ===== Taxa =====
        # O maior ganho de banda esta aqui, nao no recorte. Casar com o
        # `processing_frequency` do detector: publicar mais rapido do que ele
        # processa e gastar enlace a troco de nada.
        self.declare_parameter('publish_rate', 8.0)

        # ===== ROI =====
        # -1 = centrado. Ver o aviso sobre os PIDs no cabecalho do arquivo.
        self.declare_parameter('roi_width', 400)
        self.declare_parameter('roi_height', 400)
        self.declare_parameter('roi_x', -1)
        self.declare_parameter('roi_y', -1)

        # Redimensionamento OPCIONAL depois do recorte (0 = desligado). Recortar
        # preserva os pixels da mao; reduzir a escala os joga fora. Use so se a
        # banda ainda apertar depois de ajustar taxa e ROI -- o MediaPipe perde
        # alcance de deteccao rapido quando a mao fica pequena.
        self.declare_parameter('output_width', 0)
        self.declare_parameter('output_height', 0)

        input_topic = self.get_parameter('input_topic').value
        camera_name = self.get_parameter('camera_name').value
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.roi_w = int(self.get_parameter('roi_width').value)
        self.roi_h = int(self.get_parameter('roi_height').value)
        self.roi_x = int(self.get_parameter('roi_x').value)
        self.roi_y = int(self.get_parameter('roi_y').value)
        self.out_w = int(self.get_parameter('output_width').value)
        self.out_h = int(self.get_parameter('output_height').value)

        rate = float(self.get_parameter('publish_rate').value)
        if rate <= 0.0:
            self.get_logger().warn(
                f'publish_rate={rate} invalido; publicando todo quadro que chegar')
        self._intervalo_ns = int(1e9 / rate) if rate > 0.0 else 0
        self._ultimo_ns = 0

        # BEST_EFFORT com profundidade 1: se este no atrasar, o certo e PERDER
        # quadro, nao enfileirar. Uma fila funda aqui vira latencia acumulada no
        # gesto -- o operador levanta a mao e o drone responde ao passado.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        topico = nome_do_topico(camera_name, True)
        self.pub = self.create_publisher(CompressedImage, topico, qos)
        self.sub = self.create_subscription(Image, input_topic, self._callback, qos)

        self.bridge = CvBridge()
        self._avisou_roi = False

        self.get_logger().info(
            f'{input_topic} -> {topico} | ROI {self.roi_w}x{self.roi_h} '
            f'@ ({"centro" if self.roi_x < 0 else self.roi_x},'
            f'{"centro" if self.roi_y < 0 else self.roi_y}) | '
            f'{rate:.1f} Hz | JPEG q{self.jpeg_quality}')

    def _callback(self, msg: Image) -> None:
        agora = self.get_clock().now().nanoseconds
        if self._intervalo_ns > 0 and (agora - self._ultimo_ns) < self._intervalo_ns:
            return
        self._ultimo_ns = agora

        try:
            # passthrough e nao 'bgr8': a camera esquerda do OAK e mono8, e
            # converter para BGR aqui so encheria o JPEG de croma vazia --
            # medido, +13% de bytes no fio a q50, sem informacao nenhuma a
            # mais.
            #
            # Isso DEPENDE da classe Detector (cv_nodes/detector) converter
            # cinza -> BGR na chegada, porque o cv_bridge nao o faz: ele ignora
            # desired_encoding='bgr8' num JPEG cinza e devolve um array 2-D,
            # que o MediaPipe recusa. A conversao mora la de proposito, para
            # nao pagar os 13% na rede. Se aquele trecho sumir, este stream
            # para de ser consumivel -- e sem erro nenhum aqui.
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().error(f'falha ao converter a imagem: {exc}')
            return

        recorte = self._recortar(frame)

        if self.out_w > 0 and self.out_h > 0:
            recorte = cv2.resize(recorte, (self.out_w, self.out_h))

        ok, buf = cv2.imencode(
            '.jpg', recorte, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return

        saida = CompressedImage()
        saida.format = 'jpeg'
        saida.data = buf.tobytes()
        # O header original, com o stamp da CAMERA. Nao carimbe a hora de agora:
        # o stamp e o unico jeito de medir, no bag, quanto tempo o quadro levou
        # do sensor ate o solo.
        saida.header = msg.header
        self.pub.publish(saida)

    def _recortar(self, frame):
        """Recorta a ROI, presa dentro do quadro."""
        h, w = frame.shape[:2]

        # Uma ROI maior que o quadro nao pode virar recorte vazio: numpy fatia
        # fora dos limites sem levantar erro, e o no publicaria imagem de zero
        # pixel para sempre, em silencio.
        rw = min(self.roi_w, w) if self.roi_w > 0 else w
        rh = min(self.roi_h, h) if self.roi_h > 0 else h

        pedido_x = (w - rw) // 2 if self.roi_x < 0 else self.roi_x
        pedido_y = (h - rh) // 2 if self.roi_y < 0 else self.roi_y
        x = max(0, min(pedido_x, w - rw))
        y = max(0, min(pedido_y, h - rh))

        ajustou = (rw, rh, x, y) != (self.roi_w, self.roi_h, pedido_x, pedido_y)
        if ajustou and not self._avisou_roi:
            self._avisou_roi = True
            self.get_logger().warn(
                f'ROI pedida ({self.roi_w}x{self.roi_h} em '
                f'{self.roi_x},{self.roi_y}) nao cabe no quadro {w}x{h}; '
                f'usando {rw}x{rh} em {x},{y}')

        return frame[y:y + rh, x:x + rw]


def main(args=None):
    rclpy.init(args=args)
    node = RoiStream()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
