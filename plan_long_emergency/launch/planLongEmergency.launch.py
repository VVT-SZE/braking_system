from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument("config_file", default_value=join(get_package_share_directory("plan_long_emergency"), "config", "params.yaml"), description="Path to the configuration file.")

    plan_long_emergency_node = Node(package="plan_long_emergency", executable="plan_long_emergency", name="plan_long_emergency", output="screen", parameters=[LaunchConfiguration("config_file")])

    return LaunchDescription([config_file_arg, plan_long_emergency_node])
