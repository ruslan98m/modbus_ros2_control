# modbus_slave_plugins

Plugin package: implementations of the Modbus device interface (e.g. **GenericModbusSlave**). Loaded via URDF/xacro and used by [modbus_hw_interface](https://github.com/ruslan98m/modbus_ros2_control) (by analogy with [ethercat_driver_ros2](https://github.com/ICube-Robotics/ethercat_driver_ros2) and `GenericEcSlave`).

The base interface and types (`ModbusSlaveInterface`, `ModbusDeviceConfig`, `BatchGroup`, etc.) live in the [modbus_slave_interface](../modbus_slave_interface) package; this package provides the plugin implementation and config loading.

## Package contents

- **GenericModbusSlave** — plugin that loads device configuration from YAML (`slave_config` / `device_config`). Use in URDF with `plugin="modbus_slave_plugins/GenericModbusSlave"`.
- **ModbusDeviceConfigLoader** — loads and parses device YAML (registers, init_registers, options). Used by GenericModbusSlave.
- **modbus_utils** — helpers for parsing type strings (`dataTypeFromString`, `registerTypeFromString`, `registerCountForDataType`, etc.) and for config validation.

## Dependencies

- **modbus_slave_interface** — base class and types.
- **modbus_hw_interface** depends on this package to load device plugins and get GenericModbusSlave.

## Usage in URDF / xacro

Specify the plugin and the path to the device YAML:

```xml
<joint name="plc_1">
  <param name="plugin">modbus_slave_plugins/GenericModbusSlave</param>
  <param name="slave_id">1</param>
  <param name="device_config">$(find modbus_hw_interface)/config/devices/plc_device.yaml</param>
</joint>
```

You can use `slave_config` instead of `device_config`; the format is the same. The YAML format is described in [modbus_hw_interface](../modbus_hw_interface/README.md).

## Build

Build together with the hardware interface:

```bash
colcon build --packages-select modbus_slave_interface modbus_slave_plugins modbus_hw_interface
source install/setup.bash
```

## License

Apache-2.0.
