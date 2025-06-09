import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path
import math
from std_msgs.msg import Bool

class PathFollowerPID(Node):
    def __init__(self):
        super().__init__('path_follower_pid')

        self.path = []
        self.current_idx = 0
        self.change_threshold = 1.0
        self.linear_speed = 0.5
        self.waiting_path = True

        self.pose_x = 0.0
        self.pose_y = 0.0

        # PID params
        self.kp = 0.6
        self.ki = 0.0
        self.kd = 0.1

        self.prev_error = 0.0
        self.integral = 0.0

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.waiting_path_pub = self.create_publisher(Bool, '/waiting_path', 10)
        self.path_sub = self.create_subscription(Path, '/waypoints', self.path_callback, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.10, self.control_loop)

    def path_callback(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.waiting_path = False
        self.get_logger().info(f'Path recibido con {len(self.path)} puntos.')

    def pose_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y

    def control_loop(self):
        self.waiting_path_pub.publish(Bool(data=self.waiting_path))
        if self.waiting_path or len(self.path) < 2:
            self.cmd_vel_pub.publish(Twist())
            return

        if self.current_idx >= len(self.path) - 1:
            last_point = self.path[-1]
            if math.hypot(self.pose_x - last_point[0], self.pose_y - last_point[1]) < 0.8:
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info('🛑🛑Buen día. Meta alcanzada.')
                self.reset_pid()
                self.path = []
                self.current_idx = 0
                self.waiting_path = True
                self.get_logger().info("Nodo de planificación listo. Esperando objetivo...")
            return

        P0 = self.path[self.current_idx]
        P1 = self.path[self.current_idx + 1]
        T = (P1[0] - P0[0], P1[1] - P0[1])
        R = (self.pose_x - P0[0], self.pose_y - P0[1])
        norm_T = math.hypot(*T)
        if norm_T < 1e-6:
            return

        proj_scalar = (R[0] * T[0] + R[1] * T[1]) / (norm_T ** 2)
        proj = (proj_scalar * T[0], proj_scalar * T[1])

        if proj_scalar > self.change_threshold:
            self.current_idx += 1
            self.get_logger().info(f'➡️Avanzando al segmento {self.current_idx}')
            self.reset_pid()
            return

        error_vec = (R[0] - proj[0], R[1] - proj[1])
        error_lat = math.hypot(*error_vec)
        if T[0] * R[1] - T[1] * R[0] > 0:
            error_lat = -error_lat

        self.integral += error_lat * 0.1
        derivative = (error_lat - self.prev_error) / 0.1
        angular_z = self.kp * error_lat + self.ki * self.integral + self.kd * derivative
        angular_z = max(min(angular_z, 2.0), -2.0)
        self.prev_error = error_lat

        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = -angular_z
        self.cmd_vel_pub.publish(cmd)

    def reset_pid(self):
        self.prev_error = 0.0
        self.integral = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()