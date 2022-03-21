import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    xarm_config = os.path.join(
        get_package_share_directory('xarm_network'),
        'config',
        'xarm_config.yaml'
    )

    register_node = Node(
        package = 'xarm_network',
        name = 'xarm_register',
        executable = 'xarm_register',
        parameters = [xarm_config]
    )

    return LaunchDescription([
        register_node
    ])