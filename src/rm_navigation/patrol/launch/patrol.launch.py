import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='patrol',
            executable='patrol_node',
            name='patrol_node',
            output='screen',
        ),
    ])
