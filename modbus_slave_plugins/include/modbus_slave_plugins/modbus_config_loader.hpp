// Copyright 2025 modbus_slave_plugins contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_config_loader.hpp
 * @brief Load Modbus device configuration from YAML (provided by modbus_slave_plugins).
 */

#ifndef MODBUS_HW_INTERFACE__MODBUS_CONFIG_LOADER_HPP_
#define MODBUS_HW_INTERFACE__MODBUS_CONFIG_LOADER_HPP_

#include <yaml-cpp/yaml.h>

#include <string>

#include "modbus_slave_interface/modbus_device_config.hpp"
#include "rclcpp/logger.hpp"

namespace modbus_hw_interface {

class ModbusDeviceConfigLoader {
 public:
  explicit ModbusDeviceConfigLoader(rclcpp::Logger logger);
  /** Load device config from YAML file. */
  bool load(const std::string& path, ModbusDeviceConfig& out);
  /** Parse already-loaded YAML node (for use inside plugins). path used for error messages. */
  bool loadFromNode(const std::string& path, const YAML::Node& root, ModbusDeviceConfig& out);

 private:
  rclcpp::Logger logger_;
  bool loadOptions(const std::string& path, const YAML::Node& root, ModbusDeviceConfig& out);
  bool loadInitRegisters(const std::string& path, const YAML::Node& root, ModbusDeviceConfig& out);
  bool loadRegisters(const std::string& path, const YAML::Node& root, ModbusDeviceConfig& out);
  bool validateNoDuplicateRegisterNames(const std::string& path, const ModbusDeviceConfig& config);
};

}  // namespace modbus_hw_interface

#endif  // MODBUS_HW_INTERFACE__MODBUS_CONFIG_LOADER_HPP_
