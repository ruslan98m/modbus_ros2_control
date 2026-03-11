# modbus_slave_plugins

Отдельный проект: базовый класс плагинов Modbus-устройств и реализация GenericModbusSlave. Подключаются в URDF и обрабатываются [modbus_hw_interface](https://github.com/your-org/modbus_ros2_control) автоматически (по аналогии с [ethercat_driver_ros2](https://github.com/ICube-Robotics/ethercat_driver_ros2) и `GenericEcSlave`).

## Содержимое пакета

- **ModbusSlaveInterface** — базовый класс плагинов (интерфейс в `modbus_hw_interface` namespace, заголовки в `include/modbus_slave_plugins/`).
- **Типы и загрузчик** — `ModbusDeviceConfig`, `ModbusRegisterConfig`, `ModbusTypes`, `ModbusDeviceConfigLoader`, `modbus_utils` (библиотека `modbus_slave_common`).
- **GenericModbusSlave** — плагин, загружающий конфигурацию устройства из YAML (`slave_config` / `device_config`).

## Зависимости

- `modbus_hw_interface` зависит от этого пакета (получает типы, интерфейс раба и загрузчик конфига).

## Использование в URDF / xacro

Устройство можно описать либо через путь к YAML, либо через плагин.

**Без плагина (как раньше):**

```xml
<joint name="plc_1">
  <param name="slave_id">1</param>
  <param name="device_config">$(find modbus_hw_interface)/config/devices/plc_device.yaml</param>
</joint>
```

**С плагином (автоматическая обработка в hardware interface):**

```xml
<joint name="plc_1">
  <param name="plugin">modbus_slave_plugins/GenericModbusSlave</param>
  <param name="slave_id">1</param>
  <param name="slave_config">$(find modbus_hw_interface)/config/devices/plc_device.yaml</param>
</joint>
```

Формат YAML для `slave_config` / `device_config` тот же, что и при использовании без плагина.

## Сборка

Соберите пакет вместе с `modbus_hw_interface`:

```bash
colcon build --packages-select modbus_hw_interface modbus_slave_plugins
source install/setup.bash
```
