import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
import cv2
import numpy as np
import math
import tf_transformations


class NavigationNode(Node):
    """Nodo ROS 2 para navegación mediante seguimiento de trayectoria utilizando Pure Pursuit con PID lateral."""

    def __init__(self):
        """Inicializa el nodo, parámetros de control, suscripciones, publicadores y temporizador."""
        super().__init__('navigation_node')

        # Subscripciones
        self.sub_mode = self.create_subscription(Bool, '/self_driving', self.mode_callback, 10)
        self.sub_pause = self.create_subscription(Bool, '/pause', self.pause_callback, 10)
        self.path_sub = self.create_subscription(Path, '/waypoints', self.path_callback, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        # Publicadores
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.waiting_path_pub = self.create_publisher(Bool, '/waiting_path', 10)

        # Timer
        self.timer = self.create_timer(0.05, self.control_loop)

        # Control y trayectoria
        self.path = []
        self.lookahead_distance = 1.2
        self.last_index = 0
        self.waiting_path = True

        # PID
        self.kp_pid = 1.2
        self.ki_pid = 0.0
        self.kd_pid = 2.2
        self.error_prev_pid = 0.0
        self.integral = 0.0
        self.max_integral = 1.0

        # Pose del robot
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0

        # GENERAL
        self.auto = False
        self.pause = False

        self.get_logger().info("Nodo de navegación iniciado con modo PID")

    def mode_callback(self, msg: Bool):
        """
        Callback para activar o desactivar el modo automático.

        Args:
            msg (Bool): True si se activa la navegación automática.
        """
        self.auto = msg.data
        self.get_logger().info(f"[MODO] self_driving: {self.auto}")

    def pause_callback(self, msg: Bool):
        """
        Callback para activar o desactivar la pausa.

        Detiene al robot inmediatamente si se pausa mientras está en modo auto.

        Args:
            msg (Bool): True para pausar, False para reanudar.
        """
        if msg.data and self.pause and self.auto:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info("Se recibió True, mandando stop inmediato.")
        elif not msg.data and self.pause and self.auto:
            self.get_logger().info("Se desactivó la pausa.")

        self.pause = msg.data

    def pose_callback(self, msg):
        """
        Callback que actualiza la pose actual del robot desde AMCL.

        Args:
            msg (PoseWithCovarianceStamped): Mensaje con posición y orientación del robot.
        """
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose_theta = yaw

    def path_callback(self, msg):
        """
        Callback que recibe y almacena una nueva ruta (Path).

        Args:
            msg (Path): Secuencia de puntos objetivo para navegar.
        """
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(self.path) < 2:
            self.get_logger().warn('Path demasiado corto.')
            self.waiting_path = True
            return
        self.last_index = 0
        self.waiting_path = False
        self.get_logger().info(f'Path recibido con {len(self.path)} puntos.')

    def find_lookahead_point(self):
        """
        Encuentra el punto de seguimiento más adelante en la trayectoria.

        Returns:
            tuple: Coordenadas (x, y) del punto lookahead.
        """
        for i in range(self.last_index, len(self.path)):
            dx = self.path[i][0] - self.pose_x
            dy = self.path[i][1] - self.pose_y
            dist = math.hypot(dx, dy)
            if dist > self.lookahead_distance:
                self.last_index = i
                return self.path[i]
        return self.path[-1]

    def control_loop(self):
        """Controlador que aplica PID lateral para seguir la trayectoria."""
        if not self.auto or self.pause:
            return

        self.waiting_path_pub.publish(Bool(data=self.waiting_path))

        if self.waiting_path or len(self.path) < 2:
            self.cmd_pub.publish(Twist())
            return

        goal_dx = self.path[-1][0] - self.pose_x
        goal_dy = self.path[-1][1] - self.pose_y
        distance_to_goal = math.hypot(goal_dx, goal_dy)

        if distance_to_goal < 1.6 and self.last_index >= len(self.path) - 1:
            self.get_logger().info('Meta alcanzada. Deteniendo robot.')
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.waiting_path = True
            self.path = []
            self.last_index = 0
            self.error_prev_pid = 0.0
            self.integral = 0.0
            return

        lookahead = self.find_lookahead_point()
        dx = lookahead[0] - self.pose_x
        dy = lookahead[1] - self.pose_y

        # Transformar a marco del robot
        local_x = math.cos(-self.pose_theta) * dx - math.sin(-self.pose_theta) * dy
        local_y = math.sin(-self.pose_theta) * dx + math.cos(-self.pose_theta) * dy

        # PID sobre error lateral
        error_lat = local_y
        self.integral += error_lat * 0.05
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        derivative = (error_lat - self.error_prev_pid) / 0.05
        angular_z = (self.kp_pid * error_lat + self.ki_pid * self.integral + self.kd_pid * derivative)
        self.error_prev_pid = error_lat

        # Velocidad adaptativa
        speed = 0.5 * (1 - min(abs(error_lat) / 2.0, 1.0)) + 0.35
        speed = min(speed, 0.65)

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = -angular_z
        self.cmd_pub.publish(twist)


def main(args=None):
    """Función principal para lanzar el nodo NavigationNode."""
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
