// Copyright 2025 modbus_hw_interface contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_config_loader.hpp
 * @brief Load Modbus device configuration from YAML files (class-based loader).
 */

#ifndef MODBUS_HW_INTERFACE__MODBUS_CONFIG_LOADER_HPP_
#define MODBUS_HW_INTERFACE__MODBUS_CONFIG_LOADER_HPP_

#include <string>

#include "modbus_hw_interface/modbus_config.hpp"
#include "modbus_hw_interface/modbus_types.hpp"
#include "rclcpp/logger.hpp"

#include <yaml-cpp/yaml.h>

namespace modbus_hw_interface
{

/**
 * Loads Modbus device configuration from YAML.
 * Validates options, init_registers and registers; logs errors and returns false on failure.
 */
class ModbusDeviceConfigLoader
{
public:
  /** @param logger Logger used for validation and parse errors. */
  explicit ModbusDeviceConfigLoader(rclcpp::Logger logger);

  /**
   * Load device config from a YAML file.
   * @param path Path to the device YAML config file.
   * @param out Filled on success; registers and init_registers are cleared and repopulated.
   * @return true on success, false on parse error, validation error or missing required keys.
   */
  bool load(const std::string & path, ModbusDeviceConfig & out);

private:
  rclcpp::Logger logger_;
  bool loadOptions(const std::string & path, const YAML::Node & root, ModbusDeviceConfig & out);
  bool loadInitRegisters(const std::string & path, const YAML::Node & root, ModbusDeviceConfig & out);
  bool loadRegisters(const std::string & path, const YAML::Node & root, ModbusDeviceConfig & out);
  /** Check that register address ranges do not overlap within each type. */
  bool validateNoRegisterOverlap(const std::string & path, const ModbusDeviceConfig & config);
  /** Check that register names are unique. */
  bool validateNoDuplicateRegisterNames(const std::string & path, const ModbusDeviceConfig & config);
};

}  // namespace modbus_hw_interface

#endif  // MODBUS_HW_INTERFACE__MODBUS_CONFIG_LOADER_HPP_
