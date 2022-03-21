import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    object_name = LaunchConfiguration('object_name')
    mesh_file = LaunchConfiguration('mesh_file')
    origin_xyz_rpy = LaunchConfiguration(
        'origin_xyz_rpy', default='[0.0,0.0,0.0,0.0,0.0,0.0]')
    scaling = LaunchConfiguration('scaling', default='[1.0,1.0,1.0]')

    # Planning Scene ROS API Tutorial executable
    add_object = Node(
        name="add_object",
        package="xarm_object",
        executable="add_object",
        output='screen',
        parameters=[
            {
                'object_name': object_name,
                'mesh_file': mesh_file,
                'origin_xyz_rpy': origin_xyz_rpy,
                'scaling': scaling
            }
        ],
    )

    return LaunchDescription([add_object])
