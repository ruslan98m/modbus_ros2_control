# Copyright 2025 modbus_ros2_control contributors.
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="modbus_tcp_test_server",
                executable="modbus_tcp_test_server_node",
                name="modbus_tcp_test_server",
                parameters=[{"port": 5502, "slave_id": 1}],
                output="screen",
            ),
        ]
    )
