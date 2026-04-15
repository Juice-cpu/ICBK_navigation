import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("icbk_sentry_bt")

    behavior_tree_file_arg = DeclareLaunchArgument(
        "behavior_tree_file",
        default_value=os.path.join(pkg_share, "behavior_trees", "sentry_behavior.xml"),
        description="Path to the behavior tree XML file",
    )

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/stm32_serial",
        description="Serial port for referee receiver",
    )

    baud_rate_arg = DeclareLaunchArgument(
        "baud_rate",
        default_value="115200",
        description="Serial baud rate",
    )

    referee_poll_period_ms_arg = DeclareLaunchArgument(
        "referee_poll_period_ms",
        default_value="2",
        description="Referee polling period in ms",
    )
    bt_tick_hz_arg = DeclareLaunchArgument(
        "bt_tick_hz",
        default_value="100",
        description="Behavior tree tick frequency in Hz",
    )
    behavior_tree_main_node = Node(
        package="icbk_sentry_bt",
        executable="behavior_tree_main",
        name="behavior_tree_main_node",
        output="screen",
        parameters=[
            {
                "bt_xml_path": LaunchConfiguration("behavior_tree_file"),
                "serial_port": LaunchConfiguration("serial_port"),
                "baud_rate": LaunchConfiguration("baud_rate"),
                "referee_poll_period_ms": LaunchConfiguration("referee_poll_period_ms"),
                "bt_tick_hz": LaunchConfiguration("bt_tick_hz"),
            }
        ],
    )

    return LaunchDescription(
        [
            behavior_tree_file_arg,
            serial_port_arg,
            baud_rate_arg,
            referee_poll_period_ms_arg,
            bt_tick_hz_arg,
            behavior_tree_main_node,
        ]
    )
