# modbus_ros2_control

[![CI](https://github.com/ruslan98m/modbus_ros2_control/actions/workflows/ci.yml/badge.svg)](https://github.com/ruslan98m/modbus_ros2_control/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

 Packages for Modbus hardware interface and TCP test server with [ros2_control](https://control.ros.org/).

## Packages

| Package | Description |
|---------|-------------|
| **modbus_ros2_control** | Meta-package (this repo root). Launch and integration tests; depends on the packages below. |
| [modbus_master](modbus_master) | Modbus RTU/TCP client: connection, batch read/write, register decode. No ros2_control; used by modbus_hw_interface. |
| [modbus_hw_interface](modbus_hw_interface) | Hardware interface plugin for ros2_control: bus in xacro, device configs via plugins, poll thread; uses modbus_master for all Modbus I/O. |
| [modbus_slave_plugins](modbus_slave_plugins) | Slave plugins (e.g. GenericModbusSlave) and shared config types for device YAML. |
| [modbus_tcp_test_server](modbus_tcp_test_server) | Modbus TCP test server for integration and manual testing: listens on a port and responds to read/write; used by launch tests and for debugging. |

## Build

From the **repository root**:

```bash
cd /path/to/modbus_ros2_control
rosdep install -y --from-paths . --ignore-src
colcon build
source install/setup.bash
```

Build only specific packages:

```bash
colcon build --packages-select modbus_ros2_control modbus_hw_interface modbus_tcp_test_server
```

## Test

```bash
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test
colcon test-result --verbose
```

- **modbus_tcp_test_server**: unit tests (server + client in process).
- **modbus_ros2_control**: launch tests (test server + robot_state_publisher + controller_manager with modbus_hw_interface).

## Run test server (manual)

```bash
ros2 launch modbus_tcp_test_server slave.launch.py
```

See [modbus_hw_interface](modbus_hw_interface/README.md) for hardware interface configuration and usage.

## CI

- **Build and test**: [.github/workflows/ci.yml](.github/workflows/ci.yml) — build and run tests on push/PR to `main` and `jazzy`.
- **Format**: [.github/workflows/ci-format.yml](.github/workflows/ci-format.yml) — clang-format and pre-commit on C/C++ changes.

## License

Apache-2.0.
