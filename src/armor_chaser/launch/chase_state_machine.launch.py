from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    update_distance_threshold = LaunchConfiguration(
        "update_distance_threshold", default="0.5"
    )
    lost_timeout = LaunchConfiguration("lost_timeout", default="1.0")
    lost_grace = LaunchConfiguration("lost_grace", default="0.4")
    replan_period = LaunchConfiguration("replan_period", default="0.3")
    goal_frame = LaunchConfiguration("goal_frame", default="map")
    goal_offset_x = LaunchConfiguration("goal_offset_x", default="0.0")
    goal_offset_y = LaunchConfiguration("goal_offset_y", default="0.0")
    follow_distance = LaunchConfiguration("follow_distance", default="0.5")
    prediction_horizon = LaunchConfiguration("prediction_horizon", default="0.2")
    smoothing_alpha = LaunchConfiguration("smoothing_alpha", default="0.7")
    attack_radius = LaunchConfiguration("attack_radius", default="1.5")
    attack_num_sectors = LaunchConfiguration("attack_num_sectors", default="12")
    attack_cost_threshold = LaunchConfiguration("attack_cost_threshold", default="50")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "update_distance_threshold",
                default_value="0.5",
                description="Distance threshold to refresh chase goal (meters).",
            ),
            DeclareLaunchArgument(
                "lost_timeout",
                default_value="1.0",
                description="Seconds to wait without target before entering LOST.",
            ),
            DeclareLaunchArgument(
                "lost_grace",
                default_value="0.4",
                description="Grace period after short target loss before stopping updates.",
            ),
            DeclareLaunchArgument(
                "replan_period",
                default_value="0.3",
                description="Force refresh chase goal after this period (seconds).",
            ),
            DeclareLaunchArgument(
                "goal_frame",
                default_value="map",
                description="Frame id for NavigateToPose goals.",
            ),
            DeclareLaunchArgument(
                "goal_offset_x",
                default_value="0.0",
                description="X offset applied to target pose when chasing.",
            ),
            DeclareLaunchArgument(
                "goal_offset_y",
                default_value="0.0",
                description="Y offset applied to target pose when chasing.",
            ),
            DeclareLaunchArgument(
                "follow_distance",
                default_value="0.5",
                description="Distance to stay behind target along its heading.",
            ),
            DeclareLaunchArgument(
                "prediction_horizon",
                default_value="0.2",
                description="Time horizon for velocity-based target position prediction.",
            ),
            DeclareLaunchArgument(
                "smoothing_alpha",
                default_value="0.7",
                description="Low-pass coefficient for predicted target position.",
            ),
            DeclareLaunchArgument(
                "attack_radius",
                default_value="1.5",
                description="Radius to sample attack points around target.",
            ),
            DeclareLaunchArgument(
                "attack_num_sectors",
                default_value="12",
                description="Number of samples on the attack ring.",
            ),
            DeclareLaunchArgument(
                "attack_cost_threshold",
                default_value="50",
                description="Max acceptable cost for an attack point (0-100).",
            ),
            Node(
                package="armor_chaser",
                executable="chase_state_machine_node",
                name="chase_state_machine",
                output="screen",
                parameters=[
                    {
                        "update_distance_threshold": update_distance_threshold,
                        "lost_timeout": lost_timeout,
                        "lost_grace": lost_grace,
                        "replan_period": replan_period,
                        "goal_frame": goal_frame,
                        "goal_offset_x": goal_offset_x,
                        "goal_offset_y": goal_offset_y,
                        "follow_distance": follow_distance,
                        "prediction_horizon": prediction_horizon,
                        "smoothing_alpha": smoothing_alpha,
                        "attack_radius": attack_radius,
                        "attack_num_sectors": attack_num_sectors,
                        "attack_cost_threshold": attack_cost_threshold,
                    }
                ],
            ),
        ]
    )
