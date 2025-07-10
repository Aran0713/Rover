#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('leo_joy_example')
    joy_params = os.path.join(pkg_share, 'config', 'joy_mapping.yaml')

    return LaunchDescription([

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'coalesce_interval': 0.02,
                'autorepeat_rate': 30.0
            }]
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params],
            remappings=[('cmd_vel', 'cmd_vel')]
        ),

    ])
