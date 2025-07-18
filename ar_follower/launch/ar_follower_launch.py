from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ar_follower',
            executable='ar_follower_node',
            name='ar_follower_node',
            output='screen'
        )
    ])
