import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import tf_transformations

class NavigationNode(Node):
    """Nodo de ROS 2 que combina seguimiento de carril y navegación por trayectoria con Pure Pursuit."""

    def __init__(self):
        """Inicializa suscripciones, publicadores, parámetros PID y variables de navegación."""
        super().__init__('hybrid_navigation_node')
        self.bridge = CvBridge()

        # Suscripciones
        self.sub_image = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.sub_mode = self.create_subscription(Bool, '/self_driving', self.mode_callback, 10)
        self.sub_pause = self.create_subscription(Bool, '/pause', self.pause_callback, 10)
        self.path_sub = self.create_subscription(Path, '/waypoints', self.path_callback, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.flag_sub = self.create_subscription(Bool, '/flag_navigation', self.flag_callback, 10)

        # Publicadores
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.waiting_path_pub = self.create_publisher(Bool, '/waiting_path', 10)

        # Timer
        self.timer = self.create_timer(0.05, self.control_loop)

        # LANE FOLLOWER NODE
        self.kp_lane = 0.01
        self.ki_lane = 0.0
        self.kd_lane = 0.002

        self.error_prev_lane = 0.0
        self.error_sum_lane = 0.0
        self.last_time_lane = None

        self.linear_speed = 0.50
        self.reference = 20.0

        # PURE PURSUIT HYBRID
        self.path = []
        self.lookahead_distance = 1.2
        self.last_index = 0
        self.waiting_path = True

        # PID
        self.kp_pid = 1.2
        self.ki_pid = 0.0
        self.kd_pid = 0.2
        self.error_prev_pid = 0.0
        self.integral = 0.0
        self.max_integral = 1.0

        # Pose
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0

        # Estados
        self.auto = False
        self.pause = False
        self.lane_flag = False

        self.get_logger().info("Nodo de navegación híbrida iniciado con modo PID/Seguidor de carril.")

    def flag_callback(self, msg: Bool):
        """Cambia entre modo de carril (True) y Pure Pursuit (False)."""
        self.lane_flag = msg.data
        self.pause = False

        if not self.lane_flag:
            # Reinicio de Pure Pursuit
            if len(self.path) >= 2:
                closest_idx = 0
                min_dist = float('inf')
                for i, (x, y) in enumerate(self.path):
                    dist = math.hypot(x - self.pose_x, y - self.pose_y)
                    if dist < min_dist:
                        min_dist = dist
                        closest_idx = i
                self.last_index = closest_idx

            self.error_prev_pid = 0.0
            self.integral = 0.0
        else:
            # Reinicio del seguidor de carril
            self.error_prev_lane = 0.0
            self.error_sum_lane = 0.0
            self.last_time_lane = self.get_clock().now()

    def mode_callback(self, msg: Bool):
        """Activa o desactiva el modo automático."""
        self.auto = msg.data

    def pause_callback(self, msg: Bool):
        """Activa o desactiva la pausa durante el seguimiento de carril."""
        if msg.data and not self.pause and self.lane_flag:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

        self.pause = msg.data

    def pose_callback(self, msg):
        """Actualiza la posición y orientación del robot desde AMCL."""
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose_theta = yaw

    def path_callback(self, msg):
        """Guarda una nueva ruta y reinicia el índice de seguimiento."""
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(self.path) < 2:
            self.waiting_path = True
            return
        self.last_index = 0
        self.waiting_path = False

    def image_callback(self, msg: Image):
        """Procesa imagen y ejecuta el control PID para seguir carril."""
        if not self.auto or not self.lane_flag:
            return
        if self.pause:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.error_prev_lane = 0.0
            self.error_sum_lane = 0.0
            self.last_time_lane = self.get_clock().now()
            return

        self.waiting_path_pub.publish(Bool(data=self.waiting_path))

        if self.waiting_path or len(self.path) < 2:
            self.cmd_pub.publish(Twist())
            return

        goal_dx = self.path[-1][0] - self.pose_x
        goal_dy = self.path[-1][1] - self.pose_y
        distance_to_goal = math.hypot(goal_dx, goal_dy)

        if distance_to_goal < 0.8:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.waiting_path = True
            self.error_prev_lane = 0.0
            self.error_sum_lane = 0.0
            self.last_time_lane = self.get_clock().now()
            self.path = []
            self.last_index = 0
            self.error_prev_pid = 0.0
            self.integral = 0.0
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = frame.shape

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([18, 120, 120])
        upper_yellow = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)

        roi_top = int(height * 0.7)
        roi = mask[roi_top:height, :]

        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centroids = []
        for cnt in contours:
            M = cv2.moments(cnt)
            if M['m00'] > 100:
                cx = int(M['m10'] / M['m00'])
                centroids.append(cx)

        if len(centroids) >= 2:
            centroids.sort()
            left = centroids[0]
            right = centroids[-1]
            lane_center = (left + right) // 2
            image_center = width // 2
            error = float(image_center - lane_center - self.reference)
        else:
            error = 0.0

        now = self.get_clock().now()
        if self.last_time_lane is None:
            self.last_time_lane = now
            return

        dt = (now - self.last_time_lane).nanoseconds / 1e9
        if dt == 0.0:
            return

        p = self.kp_lane * error
        self.error_sum_lane += error * dt
        i = self.ki_lane * self.error_sum_lane
        d = self.kd_lane * (error - self.error_prev_lane) / dt
        angular_z = p + i + d

        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = -angular_z
        self.cmd_pub.publish(twist)

        self.error_prev_lane = error
        self.last_time_lane = now

    def find_lookahead_point(self):
        """
        Busca el siguiente punto de seguimiento para Pure Pursuit.

        Returns:
            Tuple[float, float]: punto (x, y) más lejano según lookahead_distance.
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
        """Controlador Pure Pursuit para seguimiento de trayectoria con PID lateral."""
        if not self.auto or self.lane_flag:
            return

        self.waiting_path_pub.publish(Bool(data=self.waiting_path))

        if self.waiting_path or len(self.path) < 2:
            self.cmd_pub.publish(Twist())
            return

        goal_dx = self.path[-1][0] - self.pose_x
        goal_dy = self.path[-1][1] - self.pose_y
        distance_to_goal = math.hypot(goal_dx, goal_dy)

        if distance_to_goal < 0.8 and self.last_index >= len(self.path) - 1:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.waiting_path = True
            self.path = []
            self.last_index = 0
            self.error_prev_pid = 0.0
            self.integral = 0.0
            self.error_prev_lane = 0.0
            self.error_sum_lane = 0.0
            self.last_time_lane = self.get_clock().now()
            return

        lookahead = self.find_lookahead_point()
        dx = lookahead[0] - self.pose_x
        dy = lookahead[1] - self.pose_y

        local_x = math.cos(-self.pose_theta) * dx - math.sin(-self.pose_theta) * dy
        local_y = math.sin(-self.pose_theta) * dx + math.cos(-self.pose_theta) * dy

        error_lat = local_y
        self.integral += error_lat * 0.05
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        derivative = (error_lat - self.error_prev_pid) / 0.05
        angular_z = self.kp_pid * error_lat + self.ki_pid * self.integral + self.kd_pid * derivative
        self.error_prev_pid = error_lat

        speed = 0.5 * (1 - min(abs(error_lat) / 2.0, 1.0)) + 0.35
        speed = min(speed, 0.50)

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = -angular_z
        self.cmd_pub.publish(twist)


def main(args=None):
    """Función principal para lanzar el nodo de navegación híbrida."""
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
