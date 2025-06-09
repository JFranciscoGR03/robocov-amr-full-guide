import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
import numpy as np

class ObstacleChecker(Node):
    def __init__(self):
        super().__init__('obstacle_checker')

        # Umbral por defecto en metros
        self.treshold = 1.50
        self.prev_state = False

        # Suscripciones y publicaciones
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.pause_pub = self.create_publisher(
            Bool,
            '/pause',
            10
        )

        self.get_logger().info('ObstacleChecker iniciado con threshold de 1.50 m.')

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min
        angles_deg = (np.rad2deg(angles) + 180) % 360 - 180

        mask = (angles_deg >= 170) & (angles_deg <= 190)
        front_ranges = ranges[mask]
        self.get_logger().info(f"Rango frontal capturado: {len(front_ranges)} muestras")

        front_valid = front_ranges[np.isfinite(front_ranges)]
        if len(front_valid) == 0:
            self.get_logger().warn("No hay lecturas válidas en el frente. Publicando False.")
            self.publish_pause(False)
            return

        front_min = np.min(front_valid)
        self.get_logger().info(f"Mínimo frontal: {front_min:.2f} m")

        obstacle_detected = front_min <= self.treshold
        self.get_logger().info(f"ObstacleDetected = {obstacle_detected}")

        self.publish_pause(obstacle_detected)

    def publish_pause(self, value: bool):
        msg = Bool()
        msg.data = bool(value)
        self.pause_pub.publish(msg)
        self.get_logger().info(f"Publicado /pause = {value}")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()