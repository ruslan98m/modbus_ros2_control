// Copyright 2025 modbus_hw_interface contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_config_loader.cpp
 * @brief Load Modbus device configuration from YAML.
 */

#include "modbus_hw_interface/modbus_config_loader.hpp"
#include "modbus_hw_interface/modbus_utils.hpp"

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"

namespace modbus_hw_interface
{

bool loadDeviceConfigFromFile(const std::string & path, ModbusDeviceConfig & out)
{
  try {
    YAML::Node root = YAML::LoadFile(path);
    out.registers.clear();
    out.init_registers.clear();
    if (root["read_multiple_registers"].IsDefined()) {
      out.read_multiple_registers = root["read_multiple_registers"].as<bool>(true);
    }
    if (root["write_multiple_registers"].IsDefined()) {
      out.write_multiple_registers = root["write_multiple_registers"].as<bool>(true);
    }
    if (root["read_multiple_coils"].IsDefined()) {
      out.read_multiple_coils = root["read_multiple_coils"].as<bool>(true);
    }
    if (root["write_multiple_coils"].IsDefined()) {
      out.write_multiple_coils = root["write_multiple_coils"].as<bool>(true);
    }

    YAML::Node init_list = root["init_registers"];
    if (init_list) {
      for (const auto & r : init_list) {
        ModbusInitRegisterConfig init;
        init.type = registerTypeFromString(r["type"].as<std::string>("holding_register"));
        init.address = r["address"].as<int>(0);
        init.data_type = dataTypeFromString(r["data_type"].as<std::string>("uint16"));
        init.register_count = registerCountForDataType(init.data_type);
        init.value = r["value"].as<double>(0.0);
        out.init_registers.push_back(init);
      }
    }

    YAML::Node regs = root["registers"];
    if (!regs) {
      RCLCPP_ERROR(rclcpp::get_logger("ModbusConfigLoader"),
        "Device config '%s': missing 'registers'", path.c_str());
      return false;
    }
    for (const auto & r : regs) {
      ModbusRegisterConfig reg;
      reg.name = r["name"].as<std::string>("");
      reg.type = registerTypeFromString(r["type"].as<std::string>("holding_register"));
      reg.address = r["address"].as<int>(0);
      reg.is_command = r["interface"].as<std::string>("state") == "command";
      reg.data_type = dataTypeFromString(r["data_type"].as<std::string>("uint16"));
      reg.register_count = registerCountForDataType(reg.data_type);
      out.registers.push_back(reg);
    }
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ModbusConfigLoader"),
      "Failed to load device config '%s': %s", path.c_str(), e.what());
    return false;
  }
}

}  // namespace modbus_hw_interface
