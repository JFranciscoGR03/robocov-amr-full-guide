import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
from std_msgs.msg import Header
import cv2
import numpy as np
from cv2 import aruco
import yaml
import os
import time
from geometry_msgs.msg import Quaternion
from builtin_interfaces.msg import Time
import rclpy.duration

class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')
        self.bridge = CvBridge()
        self.marker_size = 0.169  # metros
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()

        self.timeout = 60.0  # segundos

        # === Calibración ===
        calib_path = self.declare_parameter(
            'calibration_file',
            '/home/jumpers/.ros/camera_info/brio_100.yaml'
        ).get_parameter_value().string_value

        if not os.path.isfile(calib_path):
            self.get_logger().error(f"Archivo no encontrado: {calib_path}")
            exit(1)

        with open(calib_path, 'r') as file:
            calib_data = yaml.safe_load(file)

        try:
            k = calib_data['camera_matrix']['data']
            d = calib_data['distortion_coefficients']['data']
            self.camera_matrix = np.array(k, dtype=np.float32).reshape(3, 3)
            self.dist_coeffs = np.array(d, dtype=np.float32).reshape(1, -1)
            self.get_logger().info("Calibración cargada correctamente")
        except KeyError as e:
            self.get_logger().error(f"Error al leer el YAML: clave faltante {e}")
            exit(1)

        # === Poses fijas de los ArUcos ===
        self.aruco_poses = {
            33: (20.3, -0.17, 3.1416, 1),
            34: (20.3, -0.17, 3.1416, 1),
            35: (7.5, -0.17, 0.0, -1),
            36: (7.5, -0.17, 0.0, -1),
            37: (20.3, 2.9, 3.1416, 1),
            38: (20.3, 2.9, 3.1416, 1),
            39: (7.5, 2.9, 0.0, -1),
            40: (7.5, 2.9, 0.0, -1),
        }

        # === Agrupación por pareja ===
        self.pairs = [
            {39, 40},
            {37, 38},
            {35, 36},
            {33, 34}
        ]
        self.last_seen = {}  # pareja_id -> timestamp

        # === Topics ===
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.get_logger().info("Nodo activo: publicando al detectar primer ArUco de cada pareja")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in self.aruco_poses:
                    continue

                tvec = tvecs[i][0]
                dist = np.linalg.norm(tvec)

                for idx, pair in enumerate(self.pairs):
                    if marker_id in pair:
                        last_time = self.last_seen.get(idx, 0.0)
                        now = time.time()

                        if now - last_time > self.timeout:
                            base_x, y, theta, sign = self.aruco_poses[marker_id]
                            adjusted_x = base_x + (sign * dist)

                            pose_msg = PoseWithCovarianceStamped()
                            pose_msg.header = Header()
                            pose_msg.header.frame_id = "map"
                            pose_msg.header.stamp = self.get_clock().now().to_msg()

                            pose_msg.pose.pose.position.x = adjusted_x
                            pose_msg.pose.pose.position.y = y
                            pose_msg.pose.pose.position.z = 0.0

                            pose_msg.pose.pose.orientation = self.yaw_to_quaternion(theta)

                            pose_msg.pose.covariance[0] = 0.05
                            pose_msg.pose.covariance[7] = 0.05
                            pose_msg.pose.covariance[35] = 0.01

                            self.pose_pub.publish(pose_msg)
                            self.get_logger().info(f"Publicado pose del ArUco {marker_id} (ajustado x={adjusted_x:.2f}) de la pareja {idx}")
                            self.last_seen[idx] = now
                        break

    def yaw_to_quaternion(self, yaw):
        return Quaternion(
            x=0.0,
            y=0.0,
            z=np.sin(yaw / 2.0),
            w=np.cos(yaw / 2.0)
        )

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()