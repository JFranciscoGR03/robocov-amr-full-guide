import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
import numpy as np


class ObstacleChecker(Node):
    """
    Nodo ROS 2 que detecta obstáculos en el frente del robot usando LIDAR y YOLO.

    Publica True en /pause si hay un obstáculo cerca y una persona ha sido detectada.
    """

    def __init__(self):
        """Inicializa el nodo, suscripciones, publicador y parámetros por defecto."""
        super().__init__('obstacle_checker')

        self.treshold = 2.0  # Umbral de detección en metros
        self.prev_state = False
        self.person = False  # Estado actual de detección de persona

        # Suscripciones
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.yolo_sub = self.create_subscription(Bool, '/person_detected', self.yolo_callback, 10)

        # Publicador de señal de pausa
        self.pause_pub = self.create_publisher(Bool, '/pause', 10)

        self.get_logger().info('Detector de obstáculos iniciado con threshold de 2.0 m.')

    def scan_callback(self, msg: LaserScan):
        """
        Procesa los datos del LIDAR para detectar obstáculos al frente del robot.

        Si una persona ha sido detectada por YOLO y hay un obstáculo dentro del umbral,
        se publica True en /pause.

        Args:
            msg (LaserScan): Mensaje de escaneo láser recibido.
        """
        ranges = np.array(msg.ranges)
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min
        angles_deg = (np.rad2deg(angles) + 180) % 360 - 180

        mask = (angles_deg >= 170) & (angles_deg <= 190)
        front_ranges = ranges[mask]

        front_valid = front_ranges[np.isfinite(front_ranges)]
        if len(front_valid) == 0:
            self.publish_pause(False)
            return

        front_min = np.min(front_valid)
        obstacle_detected = front_min <= self.treshold and self.person

        self.publish_pause(obstacle_detected)

    def yolo_callback(self, msg: Bool):
        """
        Callback que actualiza si una persona ha sido detectada por YOLO.

        Args:
            msg (Bool): True si hay persona detectada, False en caso contrario.
        """
        self.person = msg.data

    def publish_pause(self, value: bool):
        """
        Publica el valor de pausa al tópico /pause.

        Args:
            value (bool): True si debe activarse pausa, False si se puede continuar.
        """
        msg = Bool()
        msg.data = bool(value)
        self.pause_pub.publish(msg)


def main(args=None):
    """Función principal que inicializa y ejecuta el nodo ObstacleChecker."""
    rclpy.init(args=args)
    node = ObstacleChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
