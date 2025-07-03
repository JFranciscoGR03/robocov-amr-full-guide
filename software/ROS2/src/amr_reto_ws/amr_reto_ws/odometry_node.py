import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import tf_transformations


class OdometryNode(Node):
    """Nodo ROS 2 para calcular y publicar odometría diferencial basada en velocidades de ruedas."""

    def __init__(self):
        """Inicializa el nodo, suscripciones, publicadores, temporizador y parámetros del robot."""
        super().__init__('odom')

        # Suscripciones
        self.sub_velL = self.create_subscription(Float32, 'velocityEncL', self.process_callback_velL, 10)
        self.sub_velR = self.create_subscription(Float32, 'velocityEncR', self.process_callback_velR, 10)
        self.initialpose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.reset_callback_from_initialpose, 10)

        # Publicadores
        self.distancePublisher = self.create_publisher(Float32, 'distance', 10)
        self.odomPublisher = self.create_publisher(Odometry, '/odom', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Nodo de odometría inicializado.')

        # Parámetros del robot
        self.radius = 0.08  # Radio de rueda en metros
        self.base = 0.60    # Distancia entre ruedas (ancho de eje)

        # Estado del robot
        self.old_distance = 0.0
        self.old_theta = 0.0
        self.old_x = 0.0
        self.old_y = 0.0
        self.last_velL = 0.0
        self.last_velR = 0.0

        # Posiciones acumuladas de ruedas
        self.wheel_left_pos = 0.0
        self.wheel_right_pos = 0.0

    def process_callback_velL(self, msg):
        """
        Callback para actualizar la última velocidad del encoder izquierdo.

        Args:
            msg (Float32): Velocidad en rad/s (valor negativo por convención).
        """
        self.last_velL = -msg.data

    def process_callback_velR(self, msg):
        """
        Callback para actualizar la última velocidad del encoder derecho.

        Args:
            msg (Float32): Velocidad en rad/s (valor negativo por convención).
        """
        self.last_velR = -msg.data

    def timer_callback(self):
        """Actualiza y publica la odometría, distancia recorrida y estados de las ruedas."""
        dt = self.timer_period

        # Cálculo de velocidades lineales y angulares
        linear_speed = (self.radius * self.last_velR + self.radius * self.last_velL) / 2.0
        angular_speed = -((self.radius * self.last_velR - self.radius * self.last_velL) / self.base)

        # Cálculo de distancia y orientación
        linear_speed_distance = abs(linear_speed)
        distance = self.old_distance + linear_speed_distance * dt
        theta = (self.old_theta + angular_speed * dt) % (2 * np.pi)

        # Cálculo de nueva posición
        x = self.old_x + linear_speed * np.cos(theta) * dt
        y = self.old_y + linear_speed * np.sin(theta) * dt
        q = tf_transformations.quaternion_from_euler(0, 0, theta)

        # Crear mensaje de odometría
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        odom_msg.twist.twist.linear.x = linear_speed
        odom_msg.twist.twist.angular.z = angular_speed

        # Covarianzas (fijas)
        odom_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]

        odom_msg.twist.covariance = odom_msg.pose.covariance
        self.odomPublisher.publish(odom_msg)

        # Publicar posición angular de ruedas
        self.wheel_left_pos += self.last_velL * dt
        self.wheel_right_pos += self.last_velR * dt

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['wheel_left_joint', 'wheel_right_joint']
        joint_state.position = [self.wheel_left_pos, self.wheel_right_pos]
        self.joint_state_pub.publish(joint_state)

        # Publicar distancia total recorrida
        self.distancePublisher.publish(Float32(data=distance))

        # Actualizar estado interno
        self.old_distance = distance
        self.old_theta = theta
        self.old_x = x
        self.old_y = y


def main(args=None):
    """Función principal para inicializar y ejecutar el nodo OdometryNode."""
    rclpy.init(args=args)
    m_p = OdometryNode()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
