from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serpentine_patrol',
            executable='serpentine_patrol_node',
            name='serpentine_patrol_node',
            output='screen',
        ),
    ])

