import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Planning Scene ROS API Tutorial executable
    remove_object = Node(
        name="remove_object",
        package="add_object",
        executable="remove_object",
    )

    return LaunchDescription([remove_object])
