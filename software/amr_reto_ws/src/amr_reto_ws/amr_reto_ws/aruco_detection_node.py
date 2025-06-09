import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
from cv2 import aruco
import yaml
import os

class ArucoPublisher(Node):
    def __init__(self):
        super().__init__('aruco_publisher')
        self.bridge = CvBridge()
        self.threshold = 3.0  # metros

        # === ArUco ===
        self.marker_size = 0.169  # 16.9 cm
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()

        # === Calibración desde YAML ===
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

        # === Topics ===
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.aruco_pub = self.create_publisher(Int32, '/aruco_info', 10)

        self.get_logger().info("Nodo ArUco activo: publicando ID.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

            # Buscar el más cercano dentro del umbral
            closest_idx = None
            min_dist = float('inf')

            for i, marker_id in enumerate(ids.flatten()):
                tvec = tvecs[i][0]
                dist = np.linalg.norm(tvec)
                if dist <= self.threshold and dist < min_dist:
                    min_dist = dist
                    closest_idx = i

            if closest_idx is not None:
                marker_id = int(ids[closest_idx])
                msg_out = Int32()
                msg_out.data = marker_id
                self.aruco_pub.publish(msg_out)
                self.get_logger().info(f"Publicado /aruco_info = {marker_id} (a {min_dist:.2f} m)")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()