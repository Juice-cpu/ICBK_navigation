import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare spin speed parameter and other optional parameters
    # Real robot default should use system time; override with use_sim_time:=true for simulation.
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')    
        
    # Launch your real_vel_transform node
    real_vel_transform_node = Node(
        package='real_vel_transform',
        executable='real_vel_transform_node',  # Ensure this matches your compiled executable
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time }
        ]
    )

    return LaunchDescription([real_vel_transform_node])
