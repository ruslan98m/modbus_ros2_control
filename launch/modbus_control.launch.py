# Copyright 2025 modbus_ros2_control contributors.
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('modbus_ros2_control')
    urdf_arg = LaunchConfiguration('urdf_file').perform(context)
    urdf_file = urdf_arg if os.path.isabs(urdf_arg) else os.path.join(pkg_share, 'description', 'urdf', urdf_arg)
    if not os.path.isfile(urdf_file):
        urdf_file = os.path.join(pkg_share, 'description', 'urdf', 'modbus_system.urdf.xacro')

    # Process xacro to get robot_description
    from subprocess import run, PIPE
    result = run(['xacro', urdf_file], capture_output=True, text=True, cwd=pkg_share)
    if result.returncode != 0:
        raise RuntimeError('xacro failed: ' + (result.stderr or result.stdout or ''))
    robot_description = result.stdout

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            parameters=[
                {'robot_description': robot_description},
                os.path.join(pkg_share, 'config', 'controllers.yaml'),
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_file',
            default_value='modbus_system.urdf.xacro',
            description='URDF/xacro filename or path (relative to package description/urdf/)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
