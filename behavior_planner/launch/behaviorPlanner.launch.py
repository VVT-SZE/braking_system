from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument("config_file", default_value=join(get_package_share_directory("behavior_planner"), "config", "params.yaml"), description="Path to the configuration file.")

    behavior_planner_node = Node(package="behavior_planner", executable="behavior_planner", name="behavior_planner", output="screen", parameters=[LaunchConfiguration("config_file")])

    return LaunchDescription([config_file_arg, behavior_planner_node])
