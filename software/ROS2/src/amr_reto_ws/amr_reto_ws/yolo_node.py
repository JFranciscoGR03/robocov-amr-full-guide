import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloPersonDetector(Node):
    """
    Nodo ROS 2 que detecta personas en imágenes usando YOLOv8.

    Publica True en /person_detected si se encuentra al menos una persona
    con confianza >= 0.5.
    """

    def __init__(self):
        """Inicializa el nodo, modelo YOLO, suscripción de imagen y publicador de detección."""
        super().__init__('yolo_person_detector')
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.person_pub = self.create_publisher(Bool, '/person_detected', 10)
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")

        self.get_logger().info("Nodo detector de personas con YOLO inicializado.")

    def image_callback(self, msg):
        """
        Procesa la imagen entrante y ejecuta detección con YOLO.

        Si se detecta una persona con confianza >= 0.5, publica True en /person_detected.

        Args:
            msg (Image): Imagen ROS recibida del tópico /image_raw.
        """
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)[0]

        person_detected = False

        for r in results.boxes:
            class_id = int(r.cls[0])
            conf = float(r.conf[0])
            if conf >= 0.5 and class_id == 0:  # Clase 0 es persona
                person_detected = True
                break

        self.person_pub.publish(Bool(data=person_detected))


def main(args=None):
    """Función principal para inicializar y ejecutar el nodo YoloPersonDetector."""
    rclpy.init(args=args)
    node = YoloPersonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
