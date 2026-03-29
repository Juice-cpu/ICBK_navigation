from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serpentine_patrol',
            executable='serpentine_patrol_open_loop_node',
            name='serpentine_patrol_open_loop_node',
            output='screen',
            parameters=[{
                'cmd_topic': '/cmd_vel',
                'global_frame_candidates': ['map', 'odom'],
                'base_frame_candidates': ['base_link', 'base_link_fake'],
                'loop_hz': 30.0,
                'goal_tolerance': 0.35,
                'slow_down_radius': 0.5,
                'waypoint_switch_distance': 0.70,
                'max_speed': 5.0,
                'cruise_speed': 4.8,
                'min_speed': 2.2,
                'kp_linear': 1.2,
                'use_constant_speed': True,
                'startup_delay_sec': 0.0,
                'tf_ready_required_cycles': 1,
            }],
        ),
    ])
