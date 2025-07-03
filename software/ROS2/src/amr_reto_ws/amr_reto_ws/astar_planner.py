import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header, Bool
import cv2
import numpy as np
import yaml
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from scipy.interpolate import splprep, splev
from collections import deque


class PathPlannerNode(Node):
    """Nodo de ROS 2 para planear una ruta usando A* y publicarla como Path."""

    def __init__(self):
        """Inicializa el nodo, carga el mapa, suscribe topics y configura el planificador."""
        super().__init__('path_planner_node')

        # Rutas
        self.yaml_path = '/home/jumpers/maps/mapa_aulas1_edited.yaml'
        self.pgm_path = '/home/jumpers/maps/mapa_aulas1_edited.pgm'

        # Leer mapa
        with open(self.yaml_path, 'r') as f:
            info = yaml.safe_load(f)

        self.resolution = info['resolution']
        self.origin = info['origin'][:2]
        self.negate = info['negate']

        img = cv2.imread(self.pgm_path, cv2.IMREAD_GRAYSCALE)
        if self.negate == 1:
            img = 255 - img

        # Procesamiento binario del mapa
        self.binary_map = np.where(img > 253, 1, 0)  # 1: libre, 0: obstáculo
        self.binary_map = np.flipud(self.binary_map)
        self.dilated_map = self.binary_map  # ya dilatada

        # Pose del robot
        self.current_pose = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)

        # Suscripciones
        self.create_subscription(PointStamped, '/clicked_point', self.goal_callback, 10)
        self.create_subscription(Bool, '/waiting_path', self.waiting_path_callback, 10)
        self.create_subscription(Bool, '/self_driving', self.mode_callback, 10)

        # Publicador de ruta
        self.publisher = self.create_publisher(Path, '/waypoints', 10)

        self.waiting_path = True
        self.auto = False
        self.get_logger().info("Nodo planeador de rutas inicializado. Esperando punto de goal.")

    def mode_callback(self, msg: Bool):
        """Callback para actualizar el modo de conducción automática."""
        self.auto = msg.data
        self.get_logger().info(f"[MODO] self_driving: {self.auto}")

    def amcl_pose_callback(self, msg):
        """Callback para actualizar la pose actual del robot desde AMCL."""
        self.current_pose = msg.pose.pose

    def waiting_path_callback(self, msg):
        """Callback para actualizar si se está esperando una nueva ruta."""
        self.waiting_path = msg.data

    def world_to_pixel(self, x, y):
        """
        Convierte coordenadas del mundo real a coordenadas de píxel del mapa.

        Args:
            x (float): coordenada X en metros.
            y (float): coordenada Y en metros.

        Returns:
            Tuple[int, int]: coordenadas (px, py) en el mapa.
        """
        px = int((x - self.origin[0]) / self.resolution)
        py = int((y - self.origin[1]) / self.resolution)
        return px, py

    def pixel_to_world(self, px, py):
        """
        Convierte coordenadas de píxel del mapa a coordenadas del mundo real.

        Args:
            px (int): coordenada X en píxeles.
            py (int): coordenada Y en píxeles.

        Returns:
            Tuple[float, float]: coordenadas (x, y) en metros.
        """
        x = px * self.resolution + self.origin[0]
        y = py * self.resolution + self.origin[1]
        return x, y

    def get_robot_position(self):
        """
        Retorna la posición actual del robot.

        Returns:
            Tuple[float, float] or None: posición (x, y) si está disponible, None si no.
        """
        if self.current_pose is None:
            self.get_logger().warn('Aún no hay pose de AMCL.')
            return None
        return self.current_pose.position.x, self.current_pose.position.y

    def is_occupied(self, px, py):
        """
        Verifica si una celda del mapa está ocupada.

        Args:
            px (int): coordenada X en píxeles.
            py (int): coordenada Y en píxeles.

        Returns:
            bool: True si está ocupada o fuera de límites, False si es libre.
        """
        if px < 0 or py < 0 or px >= self.dilated_map.shape[1] or py >= self.dilated_map.shape[0]:
            return True  # fuera de límites es obstáculo
        return self.dilated_map[py, px] == 0

    def find_nearest_free_pixel(self, start_px, start_py, max_radius=30):
        """
        Encuentra el píxel libre más cercano a un punto dado.

        Args:
            start_px (int): coordenada X inicial.
            start_py (int): coordenada Y inicial.
            max_radius (int): radio máximo de búsqueda.

        Returns:
            Tuple[int, int] or None: coordenadas libres encontradas o None si falla.
        """
        visited = set()
        queue = deque()
        queue.append((start_px, start_py))

        while queue:
            x, y = queue.popleft()
            if (x, y) in visited:
                continue
            visited.add((x, y))

            if not self.is_occupied(x, y):
                return (x, y)

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                if (nx, ny) not in visited and abs(nx - start_px) <= max_radius and abs(ny - start_py) <= max_radius:
                    queue.append((nx, ny))

        self.get_logger().warn("No se encontró píxel libre cercano.")
        return None

    def goal_callback(self, msg):
        """
        Callback que recibe un punto objetivo y planea una ruta desde la posición actual.

        Args:
            msg (PointStamped): punto en coordenadas del mundo.
        """
        if not self.auto or not self.waiting_path:
            return

        goal_world = (msg.point.x, msg.point.y)
        start_world = self.get_robot_position()
        if not start_world:
            return

        self.get_logger().info(f'Inicio: {start_world}, Meta: {goal_world}')

        start_px = self.world_to_pixel(*start_world)
        goal_px = self.world_to_pixel(*goal_world)

        # Si el inicio está en obstáculo, buscar el libre más cercano
        if self.is_occupied(*start_px):
            self.get_logger().warn("Inicio está sobre obstáculo, buscando el píxel libre más cercano...")
            new_start_px = self.find_nearest_free_pixel(*start_px)
            if not new_start_px:
                self.get_logger().error("No se encontró punto de inicio válido.")
                return
            start_px = new_start_px
            start_world = self.pixel_to_world(*start_px)
            self.get_logger().info(f"Nuevo inicio: {start_world}")

        # Si el goal está en obstáculo, abortar
        if self.is_occupied(*goal_px):
            self.get_logger().error("La meta está sobre un obstáculo. Aborta.")
            return

        grid = Grid(matrix=self.dilated_map.tolist())
        start_node = grid.node(*start_px)
        goal_node = grid.node(*goal_px)
        finder = AStarFinder()
        path, _ = finder.find_path(start_node, goal_node, grid)

        if len(path) < 4:
            self.get_logger().warn("Ruta muy corta o no encontrada.")
            return

        path_world = [self.pixel_to_world(x, y) for x, y in path]

        x_vals, y_vals = zip(*path_world)
        tck, _ = splprep([x_vals, y_vals], s=0.5)
        unew = np.linspace(0, 1.0, num=int(len(path_world) / 2))
        smooth_x, smooth_y = splev(unew, tck)

        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for x, y in zip(smooth_x, smooth_y):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.publisher.publish(path_msg)
        self.get_logger().info(f"Ruta publicada con {len(smooth_x)} puntos.")


def main(args=None):
    """Función principal para inicializar y ejecutar el nodo PathPlannerNode."""
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
