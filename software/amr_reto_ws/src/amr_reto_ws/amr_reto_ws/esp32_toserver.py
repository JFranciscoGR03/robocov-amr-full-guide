import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import serial
import math

class AmclSerialForwarder(Node):
    def __init__(self):
        super().__init__('amcl_serial_forwarder')
        self.serial_port = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)  
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

        # Serializar como CSV
        data_str = f"{x:.2f},{y:.2f},{yaw:.2f}\n"
        self.serial_port.write(data_str.encode('utf-8'))
        self.get_logger().info(f"Sent: {data_str.strip()}")

    def euler_from_quaternion(self, x, y, z, w):
        """Convierte de quaternion a euler yaw."""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return (0.0, 0.0, yaw)

def main(args=None):
    rclpy.init(args=args)
    node = AmclSerialForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()