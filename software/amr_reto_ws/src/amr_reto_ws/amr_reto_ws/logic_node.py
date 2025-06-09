import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion  # <-- necesario

class LogicController(Node):
    def __init__(self):
        super().__init__('logic_controller')

        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.flag_pub = self.create_publisher(Bool, '/flag_navigation', 10)

        self.lane_flag = False

        self.get_logger().info("Logic controller listo.")

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Evaluar posición para bandera
        new_flag = False
        if 7.5 < x < 23.1:
            if 2.6 < y < 3.84 or -0.5 < y < 0.73:
                new_flag = True

        # Publicar solo si hay cambio
        if new_flag != self.lane_flag:
            self.lane_flag = new_flag
            self.flag_pub.publish(Bool(data=self.lane_flag))

def main(args=None):
    rclpy.init(args=args)
    node = LogicController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()