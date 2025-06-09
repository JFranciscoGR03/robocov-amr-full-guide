from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_reto_ws'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jumpers',
    maintainer_email='jumpers@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_node = amr_reto_ws.odometry_node:main',
            'joystick_node = amr_reto_ws.joystick_node:main',
            'path_follower_node = amr_reto_ws.path_follower_node:main',
            'aruco_detection_node = amr_reto_ws.aruco_detection_node:main',
            'lane_follower_node = amr_reto_ws.lane_follower_node:main',
            'pause_node = amr_reto_ws.pause_node:main',
            'test_node = amr_reto_ws.test_node:main',
            'logic_node = amr_reto_ws.logic_node:main',
            'aruco_localization_node = amr_reto_ws.aruco_localization_node:main',
            'astar_planner = amr_reto_ws.astar_planner:main',
            'point_stabilization = amr_reto_ws.point_stabilization:main',
            'navigation_node = amr_reto_ws.navigation_node:main',
            'esp32_toserver = amr_reto_ws.esp32_toserver:main'
        ],
    },
)
