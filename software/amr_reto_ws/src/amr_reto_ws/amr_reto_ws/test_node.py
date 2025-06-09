import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ObstacleAngleLogger(Node):
    def __init__(self):
        super().__init__('obstacle_angle_logger')
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.treshold = 2.0  # metros

        self.get_logger().info("🚦 Nodo ObstacleAngleLogger inicializado.")

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min

        # Convertimos a grados y filtramos
        angles_deg = np.rad2deg(angles)
        mask = np.isfinite(ranges) & (ranges <= self.treshold)

        obstacle_angles = angles_deg[mask]
        rounded_angles = np.round(obstacle_angles).astype(int)

        if len(rounded_angles) > 0:
            unique_angles = np.unique(rounded_angles)
            angle_list = ', '.join([f"{a}°" for a in unique_angles])
            self.get_logger().info(f"🔍 Obstáculos a {self.treshold:.2f} m en: {angle_list}")
        else:
            self.get_logger().info("🟢 Sin obstáculos a 2 metros.")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAngleLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()