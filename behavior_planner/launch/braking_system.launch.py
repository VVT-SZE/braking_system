from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    # Declare global safety_distance parameter
    safety_distance_arg = DeclareLaunchArgument(
        "safety_distance",
        default_value="10.0",
        description="Global safety distance parameter in meters"
    )

    # Config files for each node
    ctrl_config_file = join(get_package_share_directory("ctrl_long_emergency"), "config", "params.yaml")
    behavior_config_file = join(get_package_share_directory("behavior_planner"), "config", "params.yaml")

    # Nodes
    behavior_planner_node = Node(
        package="behavior_planner",
        executable="behavior_planner",
        name="behavior_planner",
        output="screen",
        parameters=[
            behavior_config_file,
            {"safety_distance": LaunchConfiguration("safety_distance")}
        ]
    )

    plan_long_emergency_node = Node(
        package="plan_long_emergency",
        executable="plan_long_emergency",
        name="plan_long_emergency",
        output="screen",
        parameters=[
            {"safety_distance": LaunchConfiguration("safety_distance")}
        ]
    )

    ctrl_long_emergency_node = Node(
        package="ctrl_long_emergency",
        executable="ctrl_long_emergency",
        name="ctrl_long_emergency",
        output="screen",
        parameters=[
            ctrl_config_file,
            {"safety_distance": LaunchConfiguration("safety_distance")}
        ]
    )

    return LaunchDescription([
        safety_distance_arg,
        behavior_planner_node,
        plan_long_emergency_node,
        ctrl_long_emergency_node,
    ])