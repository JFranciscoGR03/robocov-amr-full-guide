import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path
import math
import tf_transformations
from std_msgs.msg import Bool

class HybridPursuitPID(Node):
    def __init__(self):
        super().__init__('hybrid_pursuit_pid')

        # Control y trayectoria
        self.path = []
        self.lookahead_distance = 1.2
        self.last_index = 0
        self.waiting_path = True

        # PID
        self.kp = 1.2
        self.ki = 0.0
        self.kd = 0.2
        self.prev_error = 0.0
        self.integral = 0.0
        self.max_integral = 1.0  # Para evitar acumulación

        # Pose del robot
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0

        # Subs y pubs
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.waiting_path_pub = self.create_publisher(Bool, '/waiting_path', 10)
        self.path_sub = self.create_subscription(Path, '/waypoints', self.path_callback, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.sub_mode = self.create_subscription(Bool, '/self_driving', self.mode_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('✅ Hybrid Pursuit PID listo para jalar.')

    def mode_callback(self, msg: Bool):
        self.auto = msg.data
        self.get_logger().info(f"[MODO] self_driving: {self.auto}")

    def pose_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose_theta = yaw

    def path_callback(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(self.path) < 2:
            self.get_logger().warn('❗️Path demasiado corto.')
            self.waiting_path = True
            return
        self.last_index = 0
        self.waiting_path = False
        self.get_logger().info(f'📍 Path recibido con {len(self.path)} puntos.')

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
        self.waiting_path_pub.publish(Bool(data=self.waiting_path))

        # Revisa condiciones para no hacer nada
        if self.waiting_path or len(self.path) < 2:
            self.cmd_vel_pub.publish(Twist())
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
            self.cmd_vel_pub.publish(twist)
            self.waiting_path = True
            self.path = []
            self.last_index = 0
            self.prev_error = 0.0
            self.integral = 0.0
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
        derivative = (error_lat - self.prev_error) / 0.05
        angular_z = (self.kp * error_lat + self.ki * self.integral + self.kd * derivative)
        self.prev_error = error_lat

        # Velocidad adaptativa con mínimo garantizado
        speed = 0.5 * (1 - min(abs(error_lat) / 2.0, 1.0)) + 0.35
        speed = min(speed, 0.50)

        # Publicar velocidades
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = -angular_z
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = HybridPursuitPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()