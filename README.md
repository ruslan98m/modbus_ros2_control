# modbus_ros2_control

[![CI](https://github.com/ruslan98m/modbus_ros2_control/actions/workflows/ci.yml/badge.svg)](https://github.com/ruslan98m/modbus_ros2_control/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

**Hardware interface** for Modbus RTU (RS485) and Modbus TCP (Ethernet) with [ros2_control](https://control.ros.org/), in the same plugin style as [ethercat_driver_ros2](https://github.com/ICube-Robotics/ethercat_driver_ros2). **Bus is configured in xacro**; **each device has its own YAML** (like EtherCAT `slave_config`). Registers are exposed as **state** or **command** interfaces. Polling runs in a **separate thread** with configurable realtime_tools (priority, affinity).

## Features

- **Bus in xacro**: Connection (TCP or RTU) and its parameters are set in the xacro/URDF, one bus per `<ros2_control>` block.
- **Register types**: Coils (0x), Discrete inputs (1x), Input registers (3x), Holding registers (4x).
- **Data types**: `bool`, `int16`, `uint16`, `int32`, `uint32`, `float32` (32-bit use two registers).
- **Polling in a separate thread**: Modbus registers are polled sequentially in a dedicated thread. `read()`/`write()` only copy from/to thread-safe buffers. The poll thread can be configured with **thread priority** (SCHED_FIFO 0–99) and **CPU affinity** via **realtime_tools** (params `thread_priority`, `cpu_affinity`, `poll_rate_hz` in xacro).

**Note:** [libmodbus](https://libmodbus.org/) supports only **RTU** and **TCP**. Modbus ASCII is not supported.

## Dependencies

- ROS 2 Jazzy
- [ros2_control](https://github.com/ros-controls/ros2_control)
- **libmodbus**: `sudo apt install libmodbus-dev`
- **yaml-cpp**: `sudo apt install libyaml-cpp-dev`
- **xacro**

## Build

```bash
cd /path/to/ros2_ws
rosdep install -y --from-paths src --ignore-src
colcon build --packages-select modbus_driver
source install/setup.bash
```

## Configuration

### 1. Bus in xacro

Define the bus in the xacro using the macros from `description/ros2_control/modbus_bus.ros2_control.xacro`:

**TCP bus** — params: `name`, `bus_name`, `ip_address`, `port`, and optionally:
- `poll_rate_hz` (default 50)
- `thread_priority` (SCHED_FIFO 0–99 with RT kernel, else max nice; default 50)
- `cpu_affinity` (e.g. `"2"` or `"1,2,3"`, empty = no affinity)
- `lock_memory` (true/false) — lock process memory to avoid page faults (realtime_tools)
- `preallocate_stack` (true/false) — preallocate 8 KiB stack in poll thread

```xml
<xacro:include filename="$(find modbus_driver)/description/ros2_control/modbus_bus.ros2_control.xacro"/>

<xacro:modbus_tcp_bus name="ModbusTCP" bus_name="tcp_bus" ip_address="192.168.1.10" port="502">
  <devices>
    <joint name="plc_1">
      <param name="slave_id">1</param>
      <param name="device_config">$(find modbus_driver)/config/devices/plc_device.yaml</param>
    </joint>
  </devices>
</xacro:modbus_tcp_bus>
```

**RTU (RS485) bus** — same optional poll/RT params as TCP (`poll_rate_hz`, `thread_priority`, `cpu_affinity`, `lock_memory`, `preallocate_stack`):

```xml
<xacro:modbus_rtu_bus name="ModbusRTU" bus_name="rs485_bus" serial_port="/dev/ttyUSB0" baud_rate="9600">
  <devices>
    <joint name="drive_1">
      <param name="slave_id">1</param>
      <param name="device_config">$(find modbus_driver)/config/devices/drive_device.yaml</param>
    </joint>
  </devices>
</xacro:modbus_rtu_bus>
```

Each `<joint>` is one Modbus device: `name` is the device name (for interface naming), `slave_id` is the Modbus slave ID, `device_config` (or `slave_config`) is the path to that device’s YAML.

### 2. Device YAML (one file per device)

Each device has its own YAML with a `registers` list (like EtherCAT device configs):

**Example** `config/devices/plc_device.yaml`:

```yaml
# Configuration file for Modbus device. Used by xacro via device_config / slave_config.

# Optional: written once at startup. Only coil and holding_register.
init_registers:
  - { type: "holding_register", address: 100, data_type: "uint16", value: 1 }
  - { type: "coil", address: 0, data_type: "bool", value: true }

registers:
  - { name: "temperature", type: "input_register", address: 0, interface: "state", data_type: "int16" }
  - { name: "setpoint", type: "holding_register", address: 0, interface: "command", data_type: "uint16" }
  - { name: "relay", type: "coil", address: 0, interface: "command", data_type: "bool" }
```

- **init_registers** (optional): list of registers written **once at startup** (before polling). Only `coil` and `holding_register` are writable. Each entry: `type`, `address`, `data_type`, `value`.
- **registers**: list of entries with `name`, `type`, `address`, `interface` (`state` or `command`), `data_type`.
  - **type** (register): `coil`/`c`, `discrete_input`/`di`, `input_register`/`ir`, `holding_register`/`hr`.
  - **data_type**: `bool`/`b`, `int16`/`i16`/`i8`, `uint16`/`u16`/`u8`, `int32`/`i32`, `uint32`/`u32`, `float32`/`f32`, `int64`/`i64`, `uint64`/`u64`, `float64`/`f64`. Case-insensitive.

### Interface names

Format: `<hardware_name>/<bus_name>_<device_name>_<register_name>`.

Examples: `ModbusTCP/tcp_bus_plc_1_temperature`, `ModbusRTU/rs485_bus_drive_1_speed`.

### Two buses (TCP + RS485)

Use two macros in one robot xacro, e.g. `description/urdf/modbus_two_buses.urdf.xacro`:

```xml
<xacro:modbus_tcp_bus name="ModbusTCP" ...>
  <devices><joint name="plc_1">...</joint></devices>
</xacro:modbus_tcp_bus>
<xacro:modbus_rtu_bus name="ModbusRTU" ...>
  <devices><joint name="drive_1">...</joint></devices>
</xacro:modbus_rtu_bus>
```

## Running

```bash
# Default: modbus_system.urdf.xacro (one TCP bus, one device)
ros2 launch modbus_driver modbus_control.launch.py

# Two buses (TCP + RTU)
ros2 launch modbus_driver modbus_control.launch.py urdf_file:=modbus_two_buses.urdf.xacro
```

List interfaces:

```bash
ros2 control list_hardware_interfaces
```

## Contributing

- **Code style**: C++ is formatted with [clang-format](https://clang.llvm.org/docs/ClangFormat.html); use the repo’s `.clang-format`. Run `clang-format-14 -i --style=file <files>` before committing.
- **CI**: Push and pull requests run [CI](.github/workflows/ci.yml) (build + test) and [Format](.github/workflows/ci-format.yml) (clang-format check).
- See [CONTRIBUTING.md](CONTRIBUTING.md) for pull request flow and pre-commit hooks.

## License

Apache-2.0.
