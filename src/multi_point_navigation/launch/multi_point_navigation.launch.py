import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('navigation_mode', default_value='sequence', description='Navigation mode'),
        DeclareLaunchArgument('waypoints', default_value='[1.0, 2.0, -2.0, -3.0, 3.0, 3.0]', description='List of waypoints'),

        Node(
            package='multi_point_navigation',
            executable='multi_point_navigation_executable',
            name='multi_point_navigation_node',
            output='screen',
            parameters=[{'navigation_mode': LaunchConfiguration('navigation_mode'),
                         'waypoints': LaunchConfiguration('waypoints')}],
        ),
    ])
