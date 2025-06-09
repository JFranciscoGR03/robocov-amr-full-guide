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
    def __init__(self):
        super().__init__('navigation_node')
        self.bridge = CvBridge()

        # Subscripciones
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
        # PID parameters
        self.kp_lane = 0.01
        self.ki_lane = 0.0
        self.kd_lane = 0.002

        self.error_prev_lane = 0.0
        self.error_sum_lane = 0.0
        self.last_time_lane = None

        self.linear_speed = 0.50
        self.reference = 20.0

        # PURE PURSUIT HYBRID
        # Control y trayectoria
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
        self.max_integral = 1.0  # Para evitar acumulación

        # Pose del robot
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0

        # GENERAL
        self.auto = False
        self.pause = False
        self.lane_flag = False

        self.get_logger().info("Nodo Navigation iniciado con modo PID")

    def flag_callback(self, msg: Bool):
        self.lane_flag = msg.data
        self.pause = False  # Por si quedó en pausa antes
        self.get_logger().info(f"[MODO] controlador actualizado: {self.lane_flag}")

        if not self.lane_flag:
            # 💡 MODO PURE PURSUIT ACTIVADO

            if len(self.path) >= 2:
                # Buscar el índice más cercano a la pose actual
                closest_idx = 0
                min_dist = float('inf')
                for i, (x, y) in enumerate(self.path):
                    dist = math.hypot(x - self.pose_x, y - self.pose_y)
                    if dist < min_dist:
                        min_dist = dist
                        closest_idx = i

                self.last_index = closest_idx
                # self.waiting_path = False
                self.get_logger().info(f"[RESET] Pure Pursuit activado. last_index = {closest_idx}, dist = {min_dist:.2f}")

            # Reiniciar PID de trayectoria
            self.error_prev_pid = 0.0
            self.integral = 0.0

        else:
            # 💡 MODO LANE FOLLOWER ACTIVADO

            self.error_prev_lane = 0.0
            self.error_sum_lane = 0.0
            self.last_time_lane = self.get_clock().now()
            self.get_logger().info("[RESET] Lane Follower activado. PID reiniciado.")

    def mode_callback(self, msg: Bool):
        self.auto = msg.data
        self.get_logger().info(f"[MODO] self_driving: {self.auto}")

    def pause_callback(self, msg: Bool):
        if msg.data and not self.pause and self.lane_flag:
            # Se acaba de activar la pausa
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info("[PAUSA] Se recibió True, mandando stop inmediato.")
        elif not msg.data and self.pause and self.lane_flag:
            self.get_logger().info("[PAUSA] Se desactivó la pausa.")

        self.pause = msg.data

    def pose_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose_theta = yaw

    def path_callback(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(self.path) < 2:
            self.get_logger().warn('Path demasiado corto.')
            self.waiting_path = True
            return
        self.last_index = 0
        self.waiting_path = False
        self.get_logger().info(f'Path recibido con {len(self.path)} puntos.')

    def image_callback(self, msg: Image):
        if not self.auto:
            return

        if not self.lane_flag:
            return

        if self.pause:
            # Detención inmediata y reseteo del PID
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

            self.error_prev_lane = 0.0
            self.error_sum_lane = 0.0
            self.last_time_lane = self.get_clock().now()

            self.get_logger().info("[PAUSA] Ejecutando parada y reiniciando PID.")
            return
        
        self.waiting_path_pub.publish(Bool(data=self.waiting_path))

        # Revisa condiciones para no hacer nada
        if self.waiting_path or len(self.path) < 2:
            self.cmd_pub.publish(Twist())
            return

        # Revisa si ya llegaste a la meta
        goal_dx = self.path[-1][0] - self.pose_x
        goal_dy = self.path[-1][1] - self.pose_y
        distance_to_goal = math.hypot(goal_dx, goal_dy)

        if distance_to_goal < 0.8:
            self.get_logger().info('🛑 Meta alcanzada. Deteniendo robot.')
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

        # Procesamiento de imagen para seguimiento de carril
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = frame.shape

        # HSV + filtro amarillo
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([18, 120, 120])
        upper_yellow = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Erosión para limpiar ruido
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)

        # ROI inferior (30%)
        roi_top = int(height * 0.7)
        roi = mask[roi_top:height, :]

        # Contornos y centroides
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
            self.get_logger().warn("Menos de 2 líneas detectadas.")
            error = 0.0  # Se podría detener también si quieres más seguridad

        # PID
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

        # Publicar velocidades
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = -angular_z
        self.cmd_pub.publish(twist)

        # Guardar estado
        self.error_prev_lane = error
        self.last_time_lane = now

    def find_lookahead_point(self):
        for i in range(self.last_index, len(self.path)):
            dx = self.path[i][0] - self.pose_x
            dy = self.path[i][1] - self.pose_y
            dist = math.hypot(dx, dy)
            if dist > self.lookahead_distance:
                self.last_index = i
                return self.path[i]
        return self.path[-1]

    def control_loop(self):
        if not self.auto:
            return

        if self.lane_flag:
            return

        self.waiting_path_pub.publish(Bool(data=self.waiting_path))

        # Revisa condiciones para no hacer nada
        if self.waiting_path or len(self.path) < 2:
            self.cmd_pub.publish(Twist())
            return

        # Revisa si ya llegaste a la meta
        goal_dx = self.path[-1][0] - self.pose_x
        goal_dy = self.path[-1][1] - self.pose_y
        distance_to_goal = math.hypot(goal_dx, goal_dy)

        if distance_to_goal < 0.8 and self.last_index >= len(self.path) - 1:
            self.get_logger().info('🛑 Meta alcanzada. Deteniendo robot.')
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

        # Buscar punto objetivo
        lookahead = self.find_lookahead_point()
        dx = lookahead[0] - self.pose_x
        dy = lookahead[1] - self.pose_y

        # Transformar a marco del robot
        local_x = math.cos(-self.pose_theta) * dx - math.sin(-self.pose_theta) * dy
        local_y = math.sin(-self.pose_theta) * dx + math.cos(-self.pose_theta) * dy

        # PID sobre error lateral (local_y)
        error_lat = (local_y) * 1.0
        self.integral += error_lat * 0.05
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        derivative = (error_lat - self.error_prev_pid) / 0.05
        angular_z = (self.kp_pid * error_lat + self.ki_pid * self.integral + self.kd_pid * derivative)
        self.error_prev_pid = error_lat

        # Velocidad adaptativa con mínimo garantizado
        speed = 0.5 * (1 - min(abs(error_lat) / 2.0, 1.0)) + 0.35
        speed = min(speed, 0.50)

        # Publicar velocidades
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = -angular_z
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()