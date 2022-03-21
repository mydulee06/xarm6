#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    report_type = LaunchConfiguration('report_type', default='normal')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    server_ip = LaunchConfiguration('server_ip', default='192.168.0.77')
    port = LaunchConfiguration('port', default='5000')
    object_name = LaunchConfiguration('object_name')
    mesh_file = LaunchConfiguration('mesh_file')
    origin_xyz_rpy = LaunchConfiguration(
        'origin_xyz_rpy', default='[0.0,0.0,0.0,0.0,0.0,0.0]')
    scaling = LaunchConfiguration('scaling', default='[1.0,1.0,1.0]')


    dof = 6
    xarm_moveit_realmove_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_xarm_moveit_realmove.launch.py'])),
        launch_arguments={
            'robot_ip': robot_ip,
            'report_type': report_type,
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': str(dof),
            'no_gui_ctrl': 'true',
        }.items(),
    )

    xarm_planner_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_planner'), 'launch', '_xarm_planner.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': str(dof),
            'ros2_control_plugin': 'xarm_control/XArmHW',
        }.items(),
    )

    xarm_camera_socket = Node(
        name='xarm_camera_socket',
        package='xarm_network',
        executable='xarm_camera_socket',
        output='screen',
        parameters=[
            {
                'server_ip': server_ip,
                'port': port
            }
        ],
    )
    
    return LaunchDescription([
        xarm_moveit_realmove_launch,
        xarm_planner_node_launch,
        xarm_camera_socket
    ])
