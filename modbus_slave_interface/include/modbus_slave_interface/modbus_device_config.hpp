// Copyright 2025 modbus_slave_interface contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_device_config.hpp
 * @brief Modbus device configuration structures for slave interface.
 */

#ifndef MODBUS_SLAVE_INTERFACE__MODBUS_DEVICE_CONFIG_HPP_
#define MODBUS_SLAVE_INTERFACE__MODBUS_DEVICE_CONFIG_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "modbus_slave_interface/modbus_types.hpp"

namespace modbus_hw_interface {

struct ModbusRegisterConfig {
  RegisterType type{RegisterType::HoldingRegister};
  uint16_t address{0};
  bool is_command{false};
  std::string interface_name;
  RegisterDataType data_type{RegisterDataType::Uint16};
  uint16_t register_count{1};
  double factor{1.0};
  double offset{0.0};
};

/** Written once at startup (on_activate) before polling. Only coil and holding_register are writable. */
struct ModbusInitRegisterConfig {
  RegisterType type{RegisterType::HoldingRegister};
  int address{0};
  RegisterDataType data_type{RegisterDataType::Uint16};
  int register_count{1};
  double value{0};
};

struct ModbusDeviceConfig {
  std::string name;
  int slave_id{1};
  bool read_multiple_registers{true};
  bool write_multiple_registers{true};
  bool read_multiple_coils{true};
  bool write_multiple_coils{true};
  double response_timeout_sec{0.0};
  std::vector<ModbusInitRegisterConfig> init_registers;
  std::vector<ModbusRegisterConfig> registers;
};

}  // namespace modbus_hw_interface

#endif  // MODBUS_SLAVE_INTERFACE__MODBUS_DEVICE_CONFIG_HPP_
