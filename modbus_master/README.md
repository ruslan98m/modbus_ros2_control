# modbus_master

Modbus RTU/TCP client library: connection, batch read/write, register decode. No ros2_control dependency.

Used by [modbus_hw_interface](modbus_hw_interface) so that the hardware interface package contains no direct Modbus (libmodbus) code.

## API

- **TcpConnectionParams** — TCP bus (ip_address, port).
- **RtuConnectionParams** — RTU/serial bus (serial_port, baud_rate, parity, data_bits, stop_bits).
- **ModbusMaster** — `connect()`, `disconnect()`, `readStateBatched()`, `writeCommandBatched()`, `writeInitRegisters()`.
- **BatchGroup** / **BatchItem** — describe batches of registers for read/write; built by the hardware interface from device config.

## Dependencies

- **modbus_slave_plugins** — for `ModbusDeviceConfig`, `ModbusRegisterConfig`, `RegisterType`, `RegisterDataType`.
- **libmodbus** — transport (TCP/RTU).
- **rclcpp** — logging only.

## Build

From workspace:

```bash
colcon build --packages-select modbus_master
```
