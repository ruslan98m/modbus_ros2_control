# modbus_hw_interface

**Hardware interface** for Modbus RTU (RS485) and Modbus TCP (Ethernet) with [ros2_control](https://control.ros.org/). **Bus is configured in xacro**; **each device has its own YAML** (path via `device_config` / `slave_config`). Registers are exposed as **state** or **command** interfaces. Polling runs in a **separate thread** with configurable realtime_tools (priority, affinity). Adjacent registers can be read/written in a single Modbus request when the device supports it; buffers are preallocated so the poll loop does not allocate memory.

## Features

- **Bus in xacro**: Connection (TCP or RTU) and parameters in xacro/URDF, one bus per `<ros2_control>` block.
- **Register types**: Coils (0x), Discrete inputs (1x), Input registers (3x), Holding registers (4x).
- **Data types**: `bool`, `int16`, `uint16`, `int32`, `uint32`, `float32`, `int64`, `uint64`, `float64` (32-bit use two registers, 64-bit use four).
- **Device YAML**: Optional flags per device: `read_multiple_registers`, `write_multiple_registers`, `read_multiple_coils`, `write_multiple_coils` (default `true`). When enabled, adjacent registers of the same type are read/written in one Modbus request.
- **Poll thread**: Registers are polled in a dedicated thread; `read()`/`write()` only copy from/to thread-safe buffers. Params: `thread_priority`, `cpu_affinity`, `poll_rate_hz`. No heap allocation in the poll loop.

**Note:** [libmodbus](https://libmodbus.org/) supports **RTU** and **TCP** only (no Modbus ASCII).

## Dependencies

- ROS 2 Jazzy
- [ros2_control](https://github.com/ros-controls/ros2_control)
- **libmodbus**: `sudo apt install libmodbus-dev`
- **yaml-cpp**: `sudo apt install libyaml-cpp-dev`
- **xacro**

## Build

From the **repository root** (see [README](../README.md)):

```bash
cd /path/to/modbus_ros2_control
rosdep install -y --from-paths . --ignore-src
colcon build --packages-select modbus_hw_interface
source install/setup.bash
```

## Configuration

### 1. Bus in xacro

Use the macros from `description/ros2_control/modbus_bus.ros2_control.xacro`:

**TCP bus** — params: `name`, `bus_name`, `ip_address`, `port`, and optionally:
- `poll_rate_hz` (default 50)
- `thread_priority` (e.g. 50; used with RT kernel)
- `cpu_affinity` (e.g. `"2"` or `"1,2,3"`, empty = no affinity)

```xml
<xacro:include filename="$(find modbus_hw_interface)/description/ros2_control/modbus_bus.ros2_control.xacro"/>

<xacro:modbus_tcp_bus name="ModbusTCP" bus_name="tcp_bus" ip_address="192.168.1.10" port="502">
  <devices>
    <joint name="plc_1">
      <param name="slave_id">1</param>
      <param name="device_config">$(find modbus_hw_interface)/config/devices/plc_device.yaml</param>
    </joint>
  </devices>
</xacro:modbus_tcp_bus>
```

**RTU (RS485) bus** — optional `poll_rate_hz`, `thread_priority`, `cpu_affinity` as for TCP:

```xml
<xacro:modbus_rtu_bus name="ModbusRTU" bus_name="rs485_bus" serial_port="/dev/ttyUSB0" baud_rate="9600">
  <devices>
    <joint name="drive_1">
      <param name="slave_id">1</param>
      <param name="device_config">$(find modbus_hw_interface)/config/devices/drive_device.yaml</param>
    </joint>
  </devices>
</xacro:modbus_rtu_bus>
```

Each `<joint>` is one Modbus device: `name` is the device name (for interface naming), `slave_id` is the Modbus slave ID, `device_config` (or `slave_config`) is the path to that device’s YAML.

### 2. Device YAML (one file per device)

Each device has its own YAML with a `registers` list.

**Optional device-level** (defaults: all `true`):
- `read_multiple_registers`, `write_multiple_registers` — use one request for adjacent registers (FC03/FC04/FC16).
- `read_multiple_coils`, `write_multiple_coils` — use one request for adjacent coils/bits (FC01/FC02/FC15).

**Example** `config/devices/plc_device.yaml`:

```yaml
# Optional: multi-register/coil support (default: true)
# read_multiple_registers: true
# write_multiple_registers: true
# read_multiple_coils: true
# write_multiple_coils: true

# Optional: written once at startup. Only coil and holding_register.
init_registers:
  - { type: "holding_register", address: 100, data_type: "uint16", value: 1 }
  - { type: "coil", address: 0, data_type: "bool", value: true }

registers:
  - { name: "temperature", type: "input_register", address: 0, interface: "state", data_type: "int16" }
  - { name: "setpoint", type: "holding_register", address: 0, interface: "command", data_type: "uint16" }
  - { name: "relay", type: "coil", address: 0, interface: "command", data_type: "bool" }
```

- **init_registers** (optional): written once at startup; only `coil` and `holding_register`. Fields: `type`, `address`, `data_type`, `value`.
- **registers**: `name`, `type`, `address`, `interface` (`state` or `command`), `data_type`.
  - **type**: `coil`/`c`, `discrete_input`/`di`, `input_register`/`ir`, `holding_register`/`hr`.
  - **data_type**: `bool`/`b`, `int16`/`i16`, `uint16`/`u16`, `int32`/`i32`, `uint32`/`u32`, `float32`/`f32`, `int64`/`i64`, `uint64`/`u64`, `float64`/`f64`. Case-insensitive.

### Interface names

Format: `<hardware_name>/<bus_name>_<device_name>_<register_name>`.

Examples: `ModbusTCP/tcp_bus_plc_1_temperature`, `ModbusRTU/rs485_bus_drive_1_speed`.

### Two buses (TCP + RS485)

Use both macros in one robot xacro (e.g. `description/urdf/modbus_two_buses.urdf.xacro`).

## Running

```bash
# Default: one TCP bus, one device
ros2 launch modbus_hw_interface modbus_control.launch.py

# Two buses (TCP + RTU)
ros2 launch modbus_hw_interface modbus_control.launch.py urdf_file:=modbus_two_buses.urdf.xacro
```

List interfaces:

```bash
ros2 control list_hardware_interfaces
```

## Contributing

- **Code style**: C++ formatted with [clang-format](https://clang.llvm.org/docs/ClangFormat.html); use the repo’s `.clang-format`. Run `clang-format-14 -i --style=file <files>` before committing.
- **CI**: Push and PRs run [CI](.github/workflows/ci.yml) (build + test) and [Format](.github/workflows/ci-format.yml) (clang-format check).
- See [CONTRIBUTING.md](../CONTRIBUTING.md) for pull request flow and pre-commit hooks.

## License

Apache-2.0.
