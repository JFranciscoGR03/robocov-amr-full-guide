import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion


class LogicController(Node):
    """Nodo ROS 2 que publica una bandera para cambiar el modo del controlador según la posición del robot."""

    def __init__(self):
        """Inicializa el nodo, suscripciones y publicador de la bandera de navegación."""
        super().__init__('logic_controller')

        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.flag_pub = self.create_publisher(Bool, '/flag_navigation', 10)

        self.lane_flag = False
        self.get_logger().info("Nodo alternador entre controladores listo.")

    def pose_callback(self, msg):
        """
        Callback que evalúa la posición del robot y publica una bandera si entra en zonas predefinidas.

        Args:
            msg (PoseWithCovarianceStamped): Mensaje de posición del robot desde AMCL.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Evaluar posición para activar el modo seguidor de carril
        # Coordenadas utilizando el mapa del almacen de Glaxo
        new_flag = False
        if 7.5 < x < 23.1:
            if 2.6 < y < 3.84 or -0.5 < y < 0.73:
                new_flag = True

        # Publicar solo si hubo cambio
        if new_flag != self.lane_flag:
            self.lane_flag = new_flag
            self.flag_pub.publish(Bool(data=self.lane_flag))


def main(args=None):
    """Función principal para lanzar el nodo LogicController."""
    rclpy.init(args=args)
    node = LogicController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
