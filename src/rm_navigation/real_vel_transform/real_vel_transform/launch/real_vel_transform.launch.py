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
    serial_port = LaunchConfiguration('serial_port', default='/dev/stm32_serial')
    baud_rate = LaunchConfiguration('baud_rate', default='115200')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ACM0',
        description='Serial port used to send controller velocity frames')

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for controller serial port')
         
    # Launch your real_vel_transform node
    real_vel_transform_node = Node(
        package='real_vel_transform',
        executable='real_vel_transform_node',  # Ensure this matches your compiled executable
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'serial_port': serial_port,
                'baud_rate': baud_rate,
            }
        ]
    )

    return LaunchDescription([serial_port_arg, baud_rate_arg, real_vel_transform_node])
