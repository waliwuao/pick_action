"""Launch LiDAR driver + 3-target recognition + pick action server.

Usage:
  ros2 launch pick_action pick_action.launch.py port_name:=/dev/ttyUSB0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pick_share = get_package_share_directory('pick_action')

    port_name = LaunchConfiguration('port_name')

    # LiDAR driver
    driver_share = get_package_share_directory('ldlidar_stl_ros2')
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(driver_share, 'launch', 'stl27l.launch.py')
        ),
        launch_arguments={'port_name': port_name}.items(),
    )

    # Recognition (3 targets, from spear_locator config)
    locator_share = get_package_share_directory('spear_locator')
    recognition_config = os.path.join(
        locator_share, 'config', 'recognition.yaml'
    )

    recognition_node = Node(
        package='spear_locator',
        executable='spear_recognition_node',
        name='spear_recognition',
        output='screen',
        parameters=[
            recognition_config,
            {'expected_count': 3},
        ],
    )

    # Pick action server
    pick_config = os.path.join(pick_share, 'config', 'pick_action.yaml')
    pick_node = Node(
        package='pick_action',
        executable='pick_action_server_node',
        name='pick_action_server',
        output='screen',
        parameters=[pick_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyUSB0',
            description='STL-27L serial device',
        ),
        driver_launch,
        recognition_node,
        pick_node,
    ])
