# modbus_master

Modbus RTU/TCP client library: connection, batch read/write, register decode. No ros2_control dependency.

Used by [modbus_hw_interface](modbus_hw_interface) so that the hardware interface package contains no direct Modbus (libmodbus) code.

## API

- **TcpConnectionParams** — TCP bus (ip_address, port).
- **RtuConnectionParams** — RTU/serial bus (serial_port, baud_rate, parity, data_bits, stop_bits).
- **ModbusMaster** — `connect()`, `disconnect()`, `readStateBatched()`, `writeCommandBatched()`, `writeInitRegisters()`.
- **BatchGroup** / **BatchItem** — from [modbus_slave_interface](../modbus_slave_interface); describe batches of registers for read/write; built by the master from device plugins and mappings.

## Dependencies

- **modbus_slave_interface** — for `ModbusDeviceConfig`, `ModbusRegisterConfig`, `RegisterType`, `RegisterDataType`, `BatchGroup`, and `ModbusSlaveInterface`.
- **libmodbus** — transport (TCP/RTU).
- **rclcpp** — logging only.
- **realtime_tools** — poll thread priority, affinity, memory lock.

## Build

From workspace:

```bash
colcon build --packages-select modbus_master
```
