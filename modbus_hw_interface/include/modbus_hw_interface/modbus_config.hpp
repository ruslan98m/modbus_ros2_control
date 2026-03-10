// Copyright 2025 modbus_hw_interface contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_config.hpp
 * @brief Modbus bus and device configuration structures (YAML / URDF).
 */

#ifndef MODBUS_HW_INTERFACE__MODBUS_CONFIG_HPP_
#define MODBUS_HW_INTERFACE__MODBUS_CONFIG_HPP_

#include <string>
#include <vector>

#include "modbus_hw_interface/modbus_types.hpp"

namespace modbus_hw_interface {

struct ModbusRegisterConfig {
  RegisterType type{RegisterType::HoldingRegister};
  uint16_t address{0};
  bool is_command{false}; // if true, expose on command interface (write).
  std::string interface_name; // interface name (exactly one per register).
  RegisterDataType data_type{RegisterDataType::Uint16};
  uint16_t register_count{1};  // 1 for 16-bit, 2 for 32-bit/float32, 4 for 64-bit
  double factor{1.0};    // optional: value_ros = raw * factor + offset when reading
  double offset{0.0};    // optional: when writing raw = (value_ros - offset) / factor
};

/** Written once at startup (on_activate) before polling. Only coil and holding_register are
 * writable. */
struct ModbusInitRegisterConfig {
  RegisterType type{RegisterType::HoldingRegister};
  int address{0};
  RegisterDataType data_type{RegisterDataType::Uint16};
  int register_count{1};
  double value{0};
};

struct ModbusDeviceConfig {
  std::string name;  // device/joint name from xacro (for interface naming)
  int slave_id{1};
  /** If true, use one Modbus request to read adjacent registers (FC03/FC04 multi). Default true. */
  bool read_multiple_registers{true};
  /** If true, use one Modbus request to write adjacent holding registers (FC16). Default true. */
  bool write_multiple_registers{true};
  /** If true, use one Modbus request to read adjacent coils/bits (FC01/FC02 multi). Default true.
   */
  bool read_multiple_coils{true};
  /** If true, use one Modbus request to write adjacent coils (FC15). Default true. */
  bool write_multiple_coils{true};
  /** Response timeout in seconds (0 = use libmodbus default). */
  double response_timeout_sec{0.0};
  std::vector<ModbusInitRegisterConfig> init_registers;  // written once at startup
  std::vector<ModbusRegisterConfig> registers;
};

/** One bus = one connection (either RTU or TCP, not both). */
struct ModbusBusConfig {
  std::string bus_name;  // for interface naming
  bool is_tcp{true};
  // TCP (when is_tcp == true)
  std::string ip_address{"127.0.0.1"};
  int port{502};
  // RTU (when is_tcp == false)
  std::string serial_port{"/dev/ttyUSB0"};
  int baud_rate{9600};
  char parity{'N'};  // N, E, O
  int data_bits{8};
  int stop_bits{1};
  std::vector<ModbusDeviceConfig> devices;
};

}  // namespace modbus_hw_interface

#endif  // MODBUS_HW_INTERFACE__MODBUS_CONFIG_HPP_
