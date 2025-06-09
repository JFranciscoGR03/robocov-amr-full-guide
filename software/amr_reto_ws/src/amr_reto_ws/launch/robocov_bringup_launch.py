#IMPORTS REQUIRED TO SET THE PACKAGE ADDRESS (DIRECTORIES)
import os
from ament_index_python.packages import get_package_share_directory

#IMPORTS REQUIRED FOR Launching Nodes
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

#IMPORTS REQUIRED FOR EVENTS AND ACTIONS
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LocalSubstitution


def generate_launch_description():

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='brio100_camera',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480]
        }]
    )

    urdf_default_path = os.path.join(
        get_package_share_directory('amr_reto_ws'),
        'urdf',
        'robocov.urdf')

    with open(urdf_default_path, 'r') as infp:
        robot_desc = infp.read()

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

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    config_path = os.path.join(
        get_package_share_directory('amr_reto_ws'),
        'config',
        'ekf.yaml'
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_s3_launch.py'
            )
        )
    )

    localization_map = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=['/home/jumpers/maps/map_params.yaml']
    )

    planification_map = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server_dilatado',
        output='screen',
        parameters=['/home/jumpers/maps/map_edited_params.yaml'],
        remappings=[('map', '/map_dilatado')]
    )

    montecarlo_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=['/home/jumpers/maps/amcl_params.yaml']
    )

    gamepad_node = Node(name="joystick_node",
                        package='amr_reto_ws',
                        executable='joystick_node'
                        )

    joystick_node = Node(name="joy_node",
                         package='joy',
                         executable='joy_node',
                         parameters=[{'autorepeat_rate': 10.0}],
                         )

    odometry_node = Node(name="odom",
                         package='amr_reto_ws',
                         executable='odometry_node'
                         )

    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_path]
    )

    aruco_node = Node(
        package='amr_reto_ws',
        executable='aruco_detection_node',
        name='aruco_detection_node',
    )

    lane_node = Node(
        package='amr_reto_ws',
        executable='lane_follower_node',
        name='lane_follower_node',
    )

    path_node = Node(
        package='amr_reto_ws',
        executable='path_follower_node',
        name='path_follower_node',
    )

    pursuit_node = Node(
        package='amr_reto_ws',
        executable='point_stabilization',
        name='point_stabilization',
    )

    pause_node = Node(
        package='amr_reto_ws',
        executable='pause_node',
        name='pause_node',
    )

    logic_node = Node(
        package='amr_reto_ws',
        executable='logic_node',
        name='logic_node',
    )

    aruco_loc_node = Node(
        package='amr_reto_ws',
        executable='aruco_localization_node',
        name='aruco_localization_node',
    )

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

    astar_planner = Node(
        package='amr_reto_ws',
        executable='astar_planner',
        name='astar_planner',
    )

    navigation_node = Node(
        package='amr_reto_ws',
        executable='navigation_node',
        name='navigation_node',
    )

    esp32_to_server = Node(
        package='amr_reto_ws',
        executable='esp32_toserver',
        name='esp32_toserver',
    )

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
        logic_node,
        astar_planner,
        navigation_node
        #esp32_to_server,
        #planification_map,
        #aruco_node,
        #lane_node,
        #path_node,
        #aruco_loc_node,
        #pursuit_node
    ])

    return l_d
