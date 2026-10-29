import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("aeb_perception"), "config", "params.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="perception_node + adapter_node paraméterfájl",
            ),
            Node(
                package="aeb_perception",
                executable="perception_node",
                name="aeb_perception_node",
                parameters=[LaunchConfiguration("params_file")],
                output="screen",
            ),
            Node(
                package="aeb_perception",
                executable="adapter_node",
                name="aeb_adapter_node",
                parameters=[LaunchConfiguration("params_file")],
                output="screen",
            ),
            Node(
                package="aeb_perception",
                executable="detection_visualizer",
                name="aeb_detection_visualizer",
                parameters=[LaunchConfiguration("params_file")],
                output="screen",
            ),
        ]
    )
