#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    dof = LaunchConfiguration('dof', default=7)
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='fake_components/GenericSystem')
    controllers_name = LaunchConfiguration('controllers_name', default='fake_controllers')
    moveit_controller_manager_key = LaunchConfiguration('moveit_controller_manager_key', default='moveit_fake_controller_manager')
    moveit_controller_manager_value = LaunchConfiguration('moveit_controller_manager_value', default='moveit_fake_controller_manager/MoveItFakeControllerManager')

    moveit_config_package_name = 'xarm_moveit_config'
    xarm_type = 'xarm{}'.format(dof.perform(context))

    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory(moveit_config_package_name), 'launch', 'lib', 'xarm_moveit_config_lib.py'))
    get_xarm_robot_description_parameters = getattr(mod, 'get_xarm_robot_description_parameters')
    robot_description_parameters = get_xarm_robot_description_parameters(
        xacro_urdf_file=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']),
        xacro_srdf_file=PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'srdf', 'xarm.srdf.xacro']),
        urdf_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns.perform(context).strip('/'),
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': dof,
            'ros2_control_plugin': ros2_control_plugin,
        },
        srdf_arguments={
            'prefix': prefix,
            'add_gripper': add_gripper,
            'dof': dof,
        },
        arguments={
            'context': context,
            'xarm_type': xarm_type,
        }
    )

    load_yaml = getattr(mod, 'load_yaml')
    controllers_yaml = load_yaml(moveit_config_package_name, 'config', xarm_type, '{}.yaml'.format(controllers_name.perform(context)))
    ompl_planning_yaml = load_yaml(moveit_config_package_name, 'config', xarm_type, 'ompl_planning.yaml')
    kinematics_yaml = robot_description_parameters['robot_description_kinematics']
    joint_limits_yaml = robot_description_parameters.get('robot_description_planning', None)
    add_prefix_to_moveit_params = getattr(mod, 'add_prefix_to_moveit_params')
    add_prefix_to_moveit_params(
        controllers_yaml=controllers_yaml, ompl_planning_yaml=ompl_planning_yaml, 
        kinematics_yaml=kinematics_yaml, joint_limits_yaml=joint_limits_yaml, 
        prefix=prefix.perform(context))

    # Planning Configuration
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Moveit controllers Configuration
    moveit_controllers = {
        moveit_controller_manager_key.perform(context): controllers_yaml,
        'moveit_controller_manager': moveit_controller_manager_value.perform(context),
    }

    # Trajectory Execution Configuration
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'trajectory_execution.execution_duration_monitoring': False
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        # "planning_scene_monitor_options": {
        #     "name": "planning_scene_monitor",
        #     "robot_description": "robot_description",
        #     "joint_state_topic": "/joint_states",
        #     "attached_collision_object_topic": "/move_group/planning_scene_monitor",
        #     "publish_planning_scene_topic": "/move_group/publish_planning_scene",
        #     "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
        #     "wait_for_initial_state_timeout": 10.0,
        # },
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description_parameters,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # rviz with moveit configuration
    # rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'config', xarm_type, 'planner.rviz' if no_gui_ctrl.perform(context) == 'true' else 'moveit.rviz'])
    rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'rviz', 'planner.rviz' if no_gui_ctrl.perform(context) == 'true' else 'moveit.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description_parameters,
            ompl_planning_pipeline_config,
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'link_base'],
    )

    return [
        rviz_node,
        static_tf,
        move_group_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
