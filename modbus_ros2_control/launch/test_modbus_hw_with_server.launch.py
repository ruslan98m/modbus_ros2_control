# Copyright 2025 modbus_ros2_control contributors.
# SPDX-License-Identifier: Apache-2.0

"""Launch modbus TCP test server + robot_state_publisher + controller_manager (modbus hw interface) for testing."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def build_robot_description():
    """Build robot_description for test: Modbus TCP 127.0.0.1:5502."""
    pkg_examples = get_package_share_directory("modbus_ros2_control_examples")
    device_config = os.path.join(pkg_examples, "config", "devices", "plc_device.yaml")
    if not os.path.isfile(device_config):
        raise FileNotFoundError(f"device config not found: {device_config}")
    return f"""<?xml version="1.0"?>
<robot name="modbus_robot">
  <link name="base_link"/>
  <link name="plc_1_link"/>
  <joint name="plc_1" type="continuous">
    <parent link="base_link"/>
    <child link="plc_1_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <ros2_control name="ModbusTCP" type="system">
    <hardware>
      <plugin>modbus_hw_interface/ModbusSystemInterface</plugin>
      <param name="connection_type">tcp</param>
      <param name="bus_name">tcp_bus</param>
      <param name="ip_address">127.0.0.1</param>
      <param name="port">5502</param>
      <param name="poll_rate_hz">50</param>
      <param name="thread_priority">50</param>
    </hardware>
    <joint name="plc_1">
      <param name="plugin">modbus_slave_plugins/GenericModbusSlave</param>
      <param name="slave_id">1</param>
      <param name="device_config">{device_config}</param>
    </joint>
  </ros2_control>
</robot>
"""


def generate_launch_description():
    robot_description = build_robot_description()
    pkg_examples = get_package_share_directory("modbus_ros2_control_examples")
    controllers_yaml = os.path.join(pkg_examples, "config", "controllers.yaml")

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml,
        ],
        output="screen",
    )

    spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            Node(
                package="modbus_tcp_test_server",
                executable="modbus_tcp_test_server_node",
                name="modbus_tcp_test_server",
                parameters=[{"port": 5502, "slave_id": 1}],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            controller_manager_node,
            # Spawn joint_state_broadcaster so state interfaces (modbus registers) are published to /dynamic_joint_states
            RegisterEventHandler(
                OnProcessStart(
                    target_action=controller_manager_node,
                    on_start=[
                        TimerAction(
                            period=2.0,
                            actions=[spawner_node],
                        ),
                    ],
                ),
            ),
        ]
    )
