// Copyright 2025 modbus_slave_plugins contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file generic_modbus_slave.hpp
 * @brief Generic Modbus slave plugin: parses device config from YAML (by analogy with GenericEcSlave).
 *        Use in URDF with plugin="modbus_slave_plugins/GenericModbusSlave" and param slave_config or device_config.
 */

#ifndef MODBUS_SLAVE_PLUGINS__GENERIC_MODBUS_SLAVE_HPP_
#define MODBUS_SLAVE_PLUGINS__GENERIC_MODBUS_SLAVE_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "modbus_slave_plugins/modbus_device_config.hpp"
#include "modbus_slave_plugins/modbus_slave_interface.hpp"
#include "yaml-cpp/yaml.h"

namespace modbus_slave_plugins {

/**
 * Generic Modbus slave plugin.
 * All config parsing is done inside the plugin (setup_from_config_file, setup_from_config),
 * like GenericEcSlave in ethercat_driver_ros2. Hardware interface only loads the plugin and calls setupSlave().
 */
class GenericModbusSlave : public modbus_hw_interface::ModbusSlaveInterface {
 public:
  GenericModbusSlave() = default;
  ~GenericModbusSlave() override = default;

  bool setupSlave(const std::string& name,
                  const std::unordered_map<std::string, std::string>& parameters,
                  modbus_hw_interface::ModbusDeviceConfig& out) override;

  /** Parse device config from already-loaded YAML node (used by setup_from_config_file). */
  bool setup_from_config(const YAML::Node& slave_config,
                         modbus_hw_interface::ModbusDeviceConfig& out);

  /** Load YAML file and parse device config (like GenericEcSlave::setup_from_config_file). */
  bool setup_from_config_file(const std::string& config_file,
                              modbus_hw_interface::ModbusDeviceConfig& out);

  void updateState(uint8_t device_index,
                   const std::vector<std::string>& state_names,
                   const double* state_values,
                   size_t count,
                   modbus_hw_interface::SetStateCallback set_state) override;

  void getCommand(uint8_t device_index,
                  const std::vector<std::string>& command_names,
                  modbus_hw_interface::GetCommandCallback get_command,
                  std::vector<double>& command_values_out) override;
};

}  // namespace modbus_slave_plugins

#endif  // MODBUS_SLAVE_PLUGINS__GENERIC_MODBUS_SLAVE_HPP_
