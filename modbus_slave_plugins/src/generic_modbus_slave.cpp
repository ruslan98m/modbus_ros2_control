// Copyright 2025 modbus_slave_plugins contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file generic_modbus_slave.cpp
 * @brief Generic Modbus slave plugin: config parsing inside plugin (by analogy with GenericEcSlave).
 */

#include "modbus_slave_plugins/generic_modbus_slave.hpp"

#include <string>
#include <unordered_map>

#include "modbus_slave_plugins/modbus_config_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace modbus_slave_plugins {

bool GenericModbusSlave::setupSlave(
    const std::string& name,
    const std::unordered_map<std::string, std::string>& parameters,
    modbus_hw_interface::ModbusDeviceConfig& out) {
  auto it_cfg = parameters.find("device_config");
  if (it_cfg == parameters.end()) {
    it_cfg = parameters.find("slave_config");
  }
  if (it_cfg == parameters.end() || it_cfg->second.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("GenericModbusSlave"),
                 "Modbus slave '%s': missing param 'device_config' or 'slave_config' (path to YAML)",
                 name.c_str());
    return false;
  }

  out.name = name;
  out.slave_id = 1;
  auto it_slave = parameters.find("slave_id");
  if (it_slave != parameters.end()) {
    try {
      out.slave_id = std::stoi(it_slave->second);
    } catch (...) {
      /* keep default 1 */
    }
  }

  if (!setup_from_config_file(it_cfg->second, out)) {
    RCLCPP_ERROR(rclcpp::get_logger("GenericModbusSlave"),
                 "Failed to load device config from '%s' for slave '%s'",
                 it_cfg->second.c_str(), name.c_str());
    return false;
  }
  out.name = name;  // ensure name from URDF overrides any in YAML
  return true;
}

bool GenericModbusSlave::setup_from_config(const YAML::Node& slave_config,
                                           modbus_hw_interface::ModbusDeviceConfig& out) {
  if (!slave_config || slave_config.IsNull()) {
    RCLCPP_ERROR(rclcpp::get_logger("GenericModbusSlave"),
                 "GenericModbusSlave: failed to load slave configuration: empty configuration");
    return false;
  }
  modbus_hw_interface::ModbusDeviceConfigLoader loader(
      rclcpp::get_logger("GenericModbusSlave"));
  return loader.loadFromNode("slave_config", slave_config, out);
}

bool GenericModbusSlave::setup_from_config_file(const std::string& config_file,
                                                modbus_hw_interface::ModbusDeviceConfig& out) {
  try {
    const YAML::Node root = YAML::LoadFile(config_file);
    return setup_from_config(root, out);
  } catch (const YAML::ParserException& ex) {
    RCLCPP_ERROR(rclcpp::get_logger("GenericModbusSlave"),
                 "GenericModbusSlave: failed to load config (YAML incorrect): %s", ex.what());
    return false;
  } catch (const YAML::BadFile& ex) {
    RCLCPP_ERROR(rclcpp::get_logger("GenericModbusSlave"),
                 "GenericModbusSlave: failed to load config (file incorrect or missing): %s",
                 ex.what());
    return false;
  }
}

void GenericModbusSlave::updateState(size_t /*device_index*/,
                                    const std::vector<std::string>& state_names,
                                    const double* state_values,
                                    size_t count,
                                    modbus_hw_interface::SetStateCallback set_state) {
  for (size_t i = 0; i < count && i < state_names.size(); ++i) {
    set_state(state_names[i], state_values[i]);
  }
}

void GenericModbusSlave::getCommand(size_t /*device_index*/,
                                   const std::vector<std::string>& command_names,
                                   modbus_hw_interface::GetCommandCallback get_command,
                                   std::vector<double>& command_values_out) {
  command_values_out.resize(command_names.size());
  for (size_t i = 0; i < command_names.size(); ++i) {
    command_values_out[i] = get_command(command_names[i]);
  }
}

}  // namespace modbus_slave_plugins

PLUGINLIB_EXPORT_CLASS(modbus_slave_plugins::GenericModbusSlave,
                       modbus_hw_interface::ModbusSlaveInterface)
