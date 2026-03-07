# modbus_tcp_test_server

Modbus TCP **test server** for integration and manual testing of [modbus_hw_interface](../modbus_hw_interface/README.md). Listens on a configurable port and responds to read/write of coils, discrete inputs, input registers, and holding registers (libmodbus mapping).

Part of the **modbus_ros2_control** repository.

## Build

From the **repository root** (see [README](../README.md)):

```bash
cd /path/to/modbus_ros2_control
rosdep install -y --from-paths . --ignore-src
colcon build --packages-select modbus_tcp_test_server
source install/setup.bash
```

## Run (manual testing)

```bash
ros2 run modbus_tcp_test_server modbus_tcp_test_server_node --ros-args -p port:=5502 -p slave_id:=1
```

Or with launch:

```bash
ros2 launch modbus_tcp_test_server slave.launch.py
```

## Tests

- **Unit tests** (this package): server runs in a thread, client in main thread (e.g. write/read holding register).

  ```bash
  colcon test --packages-select modbus_tcp_test_server
  colcon test-result --verbose
  ```

- **Integration tests** (meta-package [modbus_ros2_control](../README.md)): launch test with test server + robot_state_publisher + controller_manager and modbus_hw_interface.

  ```bash
  colcon test --packages-select modbus_ros2_control
  colcon test-result --verbose
  ```

## Parameters

| Parameter   | Type | Default | Description        |
|------------|------|---------|--------------------|
| `port`     | int  | 5502    | TCP port to listen |
| `slave_id` | int  | 1       | Modbus slave ID    |

## License

Apache-2.0.
