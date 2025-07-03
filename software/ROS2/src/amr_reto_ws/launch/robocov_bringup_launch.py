# IMPORTS REQUIRED TO SET THE PACKAGE ADDRESS (DIRECTORIES)
import os
from ament_index_python.packages import get_package_share_directory

# IMPORTS REQUIRED FOR Launching Nodes
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

# IMPORTS REQUIRED FOR EVENTS AND ACTIONS
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    """
    Genera una descripción de lanzamiento que inicializa todos los nodos del robot AMR.

    Esto incluye sensores, transformaciones estáticas, navegación, localización,
    seguimiento de carril, detección de personas y joystick.
    """
    # Nodo de cámara USB
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='brio100_camera',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480]
        }]
    )

    # URDF del robot
    urdf_default_path = os.path.join(
        get_package_share_directory('amr_reto_ws'),
        'urdf',
        'robocov.urdf')

    with open(urdf_default_path, 'r') as infp:
        robot_desc = infp.read()

    # Transformaciones estáticas
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                   '--frame-id', 'map', '--child-frame-id', 'odom']
    )

    static_transform_node_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0.0',
                   '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                   '--frame-id', 'world', '--child-frame-id', 'map']
    )

    # Publicador de estado del robot
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Archivo de configuración del filtro EKF
    config_path = os.path.join(
        get_package_share_directory('amr_reto_ws'),
        'config',
        'ekf.yaml'
    )

    # LIDAR (RPLIDAR)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_s3_launch.py'
            )
        )
    )

    # Servidor de mapa
    localization_map = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=['/home/jumpers/maps/map_params.yaml']
    )

    # AMCL (localización)
    montecarlo_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=['/home/jumpers/maps/amcl_params.yaml']
    )

    # Joystick
    gamepad_node = Node(
        name="joystick_node",
        package='amr_reto_ws',
        executable='joystick_node'
    )

    joystick_node = Node(
        name="joy_node",
        package='joy',
        executable='joy_node',
        parameters=[{'autorepeat_rate': 10.0}],
    )

    # Odometría del robot
    odometry_node = Node(
        name="odom",
        package='amr_reto_ws',
        executable='odometry_node'
    )

    # Filtro EKF
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_path]
    )

    # Nodo de pausa basado en obstáculos y YOLO
    pause_node = Node(
        package='amr_reto_ws',
        executable='pause_node',
        name='pause_node'
    )

    # Lifecycle Manager de Nav2
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # Planificador de caminos (A*)
    astar_planner = Node(
        package='amr_reto_ws',
        executable='astar_planner',
        name='astar_planner'
    )

    # Navegación por trayectoria (Pure Pursuit PID)
    navigation_node = Node(
        package='amr_reto_ws',
        executable='navigation_node',
        name='navigation_node'
    )

    # Nodo de navegación híbrida (Lane + Pure Pursuit PID)
    hybrid_navigation_node = Node(
        package='amr_reto_ws',
        executable='hybrid_navigation_node',
        name='hybrid_navigation_node'
    )

    # Nodo de detección de personas con YOLO
    yolo_person_node = Node(
        package='amr_reto_ws',
        executable='yolo_node',
        name='yolo_node'
    )

    # Otros nodos (comentados por ahora)
    aruco_node = Node(
        package='amr_reto_ws',
        executable='aruco_detection_node',
        name='aruco_detection_node'
    )

    lane_node = Node(
        package='amr_reto_ws',
        executable='lane_follower_node',
        name='lane_follower_node'
    )

    logic_node = Node(
        package='amr_reto_ws',
        executable='logic_node',
        name='logic_node'
    )

    # Agregar nodos a la descripción de lanzamiento
    l_d = LaunchDescription([
        camera_node,
        static_transform_node,
        static_transform_node_2,
        robot_state_pub_node,
        lidar_launch,
        localization_map,
        montecarlo_node,
        gamepad_node,
        joystick_node,
        odometry_node,
        ekf_filter_node,
        pause_node,
        lifecycle_manager_node,
        astar_planner,
        navigation_node,
        yolo_person_node,
        # hybrid_navigation_node,
        # logic_node,
        # aruco_node,
        # lane_node,
    ])

    return l_d
