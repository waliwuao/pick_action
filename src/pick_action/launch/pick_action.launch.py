"""Launch pick action pipeline: LiDAR driver + recognition + action server.

Usage:
  ros2 launch pick_action pick_action.launch.py port_name:=/dev/ttyUSB0   # real LiDAR
  ros2 launch pick_action pick_action.launch.py use_synthetic:=true       # no hardware
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pick_share = get_package_share_directory('pick_action')
    driver_share = get_package_share_directory('ldlidar_stl_ros2')

    port_name = LaunchConfiguration('port_name')
    use_synthetic = LaunchConfiguration('use_synthetic')

    recognition_config = os.path.join(pick_share, 'config', 'recognition.yaml')
    pick_config = os.path.join(pick_share, 'config', 'pick_action.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyUSB0',
            description='STL-27L serial device',
        ),
        DeclareLaunchArgument(
            'use_synthetic',
            default_value='false',
            description='Use synthetic scan (true) or real LiDAR (false)',
        ),
        DeclareLaunchArgument(
            'expected_count',
            default_value='3',
            description='Number of expected targets',
        ),

        # Real LiDAR driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(driver_share, 'launch', 'stl27l.launch.py')
            ),
            launch_arguments={'port_name': port_name}.items(),
            condition=UnlessCondition(use_synthetic),
        ),

        # Synthetic scan (development fallback)
        Node(
            package='pick_action',
            executable='synthetic_scan_node',
            name='synthetic_spear_scan',
            output='screen',
            condition=IfCondition(use_synthetic),
        ),

        # Multi-frame recognition (3 targets)
        Node(
            package='pick_action',
            executable='recognition_node',
            name='spear_recognition',
            output='screen',
            parameters=[recognition_config],
        ),

        # Pick action server
        Node(
            package='pick_action',
            executable='pick_action_server_node',
            name='pick_action_server',
            output='screen',
            parameters=[pick_config],
        ),
    ])
