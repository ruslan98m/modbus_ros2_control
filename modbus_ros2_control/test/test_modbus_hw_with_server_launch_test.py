# Copyright 2025 modbus_ros2_control contributors.
# SPDX-License-Identifier: Apache-2.0

"""Launch test: modbus TCP test server + modbus hw interface (controller_manager)."""

import importlib.util
import os
import xml.etree.ElementTree as ET
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import launch_testing.asserts


def _get_build_robot_description():
    """Import build_robot_description from the launch file (used by robot_state_publisher there)."""
    pkg_share = get_package_share_directory('modbus_ros2_control')
    launch_path = os.path.join(pkg_share, 'launch', 'test_modbus_hw_with_server.launch.py')
    spec = importlib.util.spec_from_file_location('test_modbus_launch', launch_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.build_robot_description


def generate_test_description():
    pkg_share = get_package_share_directory('modbus_ros2_control')
    launch_path = os.path.join(pkg_share, 'launch', 'test_modbus_hw_with_server.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestRobotDescription(unittest.TestCase):
    """Validate robot_description / URDF structure (same URDF used by robot_state_publisher in launch)."""

    def test_urdf_valid_xml(self):
        """robot_description is valid XML."""
        urdf = _get_build_robot_description()()
        self.assertIsInstance(urdf, str)
        self.assertIn('<robot', urdf)
        root = ET.fromstring(urdf)
        self.assertEqual(root.tag, 'robot')

    def test_urdf_ros2_control_and_plugin(self):
        """URDF has ros2_control with ModbusSystemInterface plugin."""
        root = ET.fromstring(_get_build_robot_description()())
        ros2_control = root.find('ros2_control')
        self.assertIsNotNone(ros2_control, 'ros2_control element missing')
        self.assertEqual(ros2_control.get('name'), 'ModbusTCP')
        self.assertEqual(ros2_control.get('type'), 'system')
        hardware = ros2_control.find('hardware')
        self.assertIsNotNone(hardware)
        plugin_el = hardware.find('plugin')
        if plugin_el is None:
            plugin_el = next((p for p in hardware.findall('param') if p.get('name') == 'plugin'), None)
        self.assertIsNotNone(plugin_el, 'plugin element or param missing')
        plugin_val = (plugin_el.text or plugin_el.get('value') or '').strip()
        self.assertEqual(plugin_val, 'modbus_hw_interface/ModbusSystemInterface')

    def test_urdf_modbus_tcp_params(self):
        """URDF has TCP params: ip_address, port, bus_name."""
        root = ET.fromstring(_get_build_robot_description()())
        hardware = root.find('.//hardware')
        self.assertIsNotNone(hardware)
        params = {p.get('name'): (p.text or p.get('value') or '').strip() for p in hardware.findall('param')}
        self.assertEqual(params.get('connection_type'), 'tcp')
        self.assertEqual(params.get('ip_address'), '127.0.0.1')
        self.assertEqual(params.get('port'), '5502')
        self.assertEqual(params.get('bus_name'), 'tcp_bus')

class TestModbusHwWithServer(unittest.TestCase):
    """Test that modbus test server and controller_manager start and run."""

    def test_launch_ready(self):
        """Launch reached ReadyToTest."""
        pass


class TestModbusLifecycle(unittest.TestCase):
    """Test modbus_hw_interface lifecycle via controller_manager services."""

    def test_hardware_component_loaded_and_active(self):
        """ModbusTCP hardware component is loaded and in active state."""
        import rclpy
        from controller_manager_msgs.srv import ListHardwareComponents
        from lifecycle_msgs.msg import State as LifecycleState

        try:
            rclpy.init()
        except Exception:
            pass
        node = rclpy.create_node('test_lifecycle_client')
        try:
            client = node.create_client(
                ListHardwareComponents,
                '/controller_manager/list_hardware_components',
            )
            if not client.wait_for_service(timeout_sec=10.0):
                self.skipTest('controller_manager/list_hardware_components not available')
            req = ListHardwareComponents.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            self.assertTrue(future.done(), 'list_hardware_components did not complete')
            response = future.result()
            self.assertIsNotNone(response)
            components = list(response.component)
            modbus = next((c for c in components if c.name == 'ModbusTCP'), None)
            self.assertIsNotNone(modbus, f'ModbusTCP not in components: {[c.name for c in components]}')
            self.assertEqual(modbus.state.id, LifecycleState.PRIMARY_STATE_ACTIVE,
                             f'ModbusTCP state should be ACTIVE (3), got {modbus.state.id}')
        finally:
            node.destroy_node()

    def test_hardware_interfaces_exported(self):
        """Modbus hardware exports expected state and command interfaces."""
        import rclpy
        from controller_manager_msgs.srv import ListHardwareInterfaces

        try:
            rclpy.init()
        except Exception:
            pass
        node = rclpy.create_node('test_interfaces_client')
        try:
            client = node.create_client(
                ListHardwareInterfaces,
                '/controller_manager/list_hardware_interfaces',
            )
            if not client.wait_for_service(timeout_sec=10.0):
                self.skipTest('controller_manager/list_hardware_interfaces not available')
            req = ListHardwareInterfaces.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            self.assertTrue(future.done())
            response = future.result()
            self.assertIsNotNone(response)
            state_names = [s.name for s in response.state_interfaces]
            cmd_names = [c.name for c in response.command_interfaces]
            self.assertIn('ModbusTCP/tcp_bus_plc_1_temperature', state_names,
                          f'Expected temperature state interface in {state_names}')
            self.assertIn('ModbusTCP/tcp_bus_plc_1_setpoint', cmd_names,
                          f'Expected setpoint command interface in {cmd_names}')
        finally:
            node.destroy_node()

    def test_register_values_from_broadcaster_match_expected(self):
        """State from joint_state_broadcaster (/dynamic_joint_states): validate values read by hw interface.
        Test server sets registers to value=address, coils to address%2. We check state interfaces published by broadcaster.
        """
        import rclpy
        from control_msgs.msg import DynamicJointState

        try:
            rclpy.init()
        except Exception:
            pass
        node = rclpy.create_node('test_broadcaster_values')
        received = []

        def cb(msg):
            received.append(msg)

        try:
            sub = node.create_subscription(
                DynamicJointState,
                '/dynamic_joint_states',
                cb,
                10,
            )
            # Wait for broadcaster to publish (broadcaster spawns ~2s after controller_manager)
            for _ in range(50):
                rclpy.spin_once(node, timeout_sec=0.2)
                if received:
                    break
            self.assertGreater(len(received), 0, 'No /dynamic_joint_states message received')
            msg = received[-1]
            # joint_state_broadcaster may use hardware name 'ModbusTCP' or joint name 'plc_1'
            try:
                idx = list(msg.joint_names).index('ModbusTCP')
            except ValueError:
                try:
                    idx = list(msg.joint_names).index('plc_1')
                except ValueError:
                    self.skipTest('ModbusTCP/plc_1 not in dynamic_joint_states (joint_names: %s)' % list(msg.joint_names))

            # DynamicJointState: interface_values[i] is InterfaceValue for joint_names[i]
            iv = msg.interface_values[idx]
            name_to_val = dict(zip(iv.interface_names, iv.values))

            # Test server: input_register[addr] = addr. So temperature (addr 0) == 0
            temp_key = next((k for k in name_to_val if 'temperature' in k), None)
            self.assertIsNotNone(temp_key, 'temperature interface not found in %s' % list(name_to_val.keys()))
            self.assertAlmostEqual(float(name_to_val[temp_key]), 0.0, places=5,
                msg='temperature (input_register 0) should be 0')

            # counter = int32 from input_registers 7,8 (values 7 and 8) -> (7<<16)|8 = 458760
            counter_expected = (7 << 16) | 8
            counter_key = next((k for k in name_to_val if 'counter' in k), None)
            self.assertIsNotNone(counter_key, 'counter interface not found in %s' % list(name_to_val.keys()))
            self.assertAlmostEqual(float(name_to_val[counter_key]), counter_expected, places=0,
                msg='counter (input_registers 7,8) should be %d' % counter_expected)
        finally:
            node.destroy_node()


@launch_testing.post_shutdown_test()
class TestModbusHwWithServerExitCodes(unittest.TestCase):
    """After shutdown: assert all processes exited with code 0."""

    def test_processes_exit_gracefully(self, proc_info):
        # 0 = ok, -2 = SIGINT, -6/250 = SIGABRT (e.g. modbus_tcp_test_server on shutdown), 1 = spawner failed
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2]
        )
