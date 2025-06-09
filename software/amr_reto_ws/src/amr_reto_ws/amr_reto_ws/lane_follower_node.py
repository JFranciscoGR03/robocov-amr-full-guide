import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')
        self.bridge = CvBridge()

        # Subscripciones
        self.sub_image = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.sub_mode = self.create_subscription(Bool, '/self_driving', self.mode_callback, 10)
        self.sub_pause = self.create_subscription(Bool, '/pause', self.pause_callback, 10)

        # Publicador de velocidades
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # PID parameters
        self.kp = 0.01
        self.ki = 0.0
        self.kd = 0.002

        self.error_prev = 0.0
        self.error_sum = 0.0
        self.last_time = self.get_clock().now()

        self.linear_speed = 0.50
        self.reference = 20.0

        self.auto = False
        self.pause = False

        self.get_logger().info("Lane follower with PID initialized.")

    def mode_callback(self, msg: Bool):
        self.auto = msg.data
        self.get_logger().info(f"[MODO] self_driving: {self.auto}")

    def pause_callback(self, msg: Bool):
        if msg.data and not self.pause:
            # Se acaba de activar la pausa
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info("[PAUSA] Se recibió True, mandando stop inmediato.")
        elif not msg.data and self.pause:
            self.get_logger().info("[PAUSA] Se desactivó la pausa.")

        self.pause = msg.data

    def image_callback(self, msg: Image):
        #if not self.auto:
            #return

        if self.pause:
            # Detención inmediata y reseteo del PID
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

            self.error_prev = 0.0
            self.error_sum = 0.0
            self.last_time = self.get_clock().now()

            self.get_logger().info("[PAUSA] Ejecutando parada y reiniciando PID.")
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
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt == 0.0:
            return

        p = self.kp * error
        self.error_sum += error * dt
        i = self.ki * self.error_sum
        d = self.kd * (error - self.error_prev) / dt
        angular_z = p + i + d

        # Publicar velocidades
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = -angular_z
        self.cmd_pub.publish(twist)

        # Guardar estado
        self.error_prev = error
        self.last_time = now

        # DEBUG
        self.get_logger().info(f"[PID] Error: {error:.2f}, Angular Z: {angular_z:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()