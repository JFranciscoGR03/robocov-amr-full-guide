import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time


def map_axis(value, min_output, max_output):
    """Mapea un valor de joystick entre -1 y 1 a un rango simétrico excluyendo una zona muerta."""
    if abs(value) < 1e-3:
        return 0.0
    sign = 1 if value > 0 else -1
    scaled = abs(value) * (max_output - min_output) + min_output
    return sign * scaled

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joy_to_cmdvel')

        self.subscription_joy = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_mode = self.create_publisher(Bool, 'self_driving', 10)

        self.DEADZONE_LINEAR = 0.442
        self.MAX_LINEAR = 0.884

        self.DEADZONE_ANGULAR = 1.88
        self.MAX_ANGULAR = 3.76

        self.auto = False
        self.last_toggle_time = 0.0  # ⏱️ Última vez que se alternó
        self.DEBOUNCE_DELAY = 0.3    # ⏳ 300 ms

        self.get_logger().info('Nodo joy inicializado.')

    def joy_callback(self, msg):
        if len(msg.axes) < 3:
            self.get_logger().warn('No hay suficientes ejes en el mensaje /joy')
            return

        raw_linear = msg.axes[1]
        raw_angular = msg.axes[2]

        arriba_izquierdo = msg.buttons[6]
        abajo_izquierdo = msg.buttons[8]
        arriba_derecho = msg.buttons[7]
        abajo_derecho = msg.buttons[9]

        combo_pressed = (
            arriba_izquierdo == 1 and
            abajo_izquierdo == 1 and
            abajo_derecho == 1 and
            arriba_derecho == 1
        )

        now = time.time()
        if combo_pressed and (now - self.last_toggle_time) > self.DEBOUNCE_DELAY:
            self.auto = not self.auto
            self.last_toggle_time = now
            self.get_logger().info(f'Modo auto cambiado a: {self.auto}')

        linear_mapped = map_axis(raw_linear, self.DEADZONE_LINEAR, self.MAX_LINEAR)
        angular_mapped = map_axis(raw_angular, self.DEADZONE_ANGULAR, self.MAX_ANGULAR)

        if not self.auto:
            twist = Twist()
            twist.linear.x = linear_mapped
            twist.angular.z = -angular_mapped
            self.publisher.publish(twist)

        mode = Bool()
        mode.data = self.auto
        self.publisher_mode.publish(mode)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()