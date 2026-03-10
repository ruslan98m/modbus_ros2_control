// Copyright 2025 modbus_hw_interface contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_config_loader.cpp
 * @brief Load Modbus device configuration from YAML (class implementation).
 */

#include "modbus_hw_interface/modbus_config_loader.hpp"

#include <algorithm>
#include <unordered_set>
#include <vector>

#include "modbus_hw_interface/modbus_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace modbus_hw_interface {

ModbusDeviceConfigLoader::ModbusDeviceConfigLoader(rclcpp::Logger logger) : logger_(logger) {}

bool ModbusDeviceConfigLoader::load(const std::string& path, ModbusDeviceConfig& out) {
  try {
    const YAML::Node root = YAML::LoadFile(path);
    out.registers.clear();
    out.init_registers.clear();

    if (!loadOptions(path, root, out)) {
      return false;
    }
    if (!loadInitRegisters(path, root, out)) {
      return false;
    }
    if (!loadRegisters(path, root, out)) {
      return false;
    }
    if (!validateNoDuplicateRegisterNames(path, out)) {
      return false;
    }

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Failed to load device config '%s': %s", path.c_str(), e.what());
    return false;
  }
}

bool ModbusDeviceConfigLoader::loadOptions(const std::string& path, const YAML::Node& root,
                                           ModbusDeviceConfig& out) {
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
  if (root["response_timeout_sec"].IsDefined()) {
    out.response_timeout_sec = root["response_timeout_sec"].as<double>(0.0);
    if (out.response_timeout_sec < 0.0) {
      RCLCPP_ERROR(logger_, "Device config '%s': response_timeout_sec must be >= 0", path.c_str());
      return false;
    }
  }
  return true;
}

bool ModbusDeviceConfigLoader::loadInitRegisters(const std::string& path, const YAML::Node& root,
                                                 ModbusDeviceConfig& out) {
  const YAML::Node init_list = root["init_registers"];
  if (!init_list) {
    return true;
  }
  if (!init_list.IsSequence()) {
    RCLCPP_ERROR(logger_, "Device config '%s': init_registers must be a list", path.c_str());
    return false;
  }
  for (size_t i = 0; i < init_list.size(); ++i) {
    const auto& r = init_list[i];
    ModbusInitRegisterConfig init;
    init.type = registerTypeFromString(r["type"].as<std::string>("holding_register"));
    init.address = r["address"].as<int>(0);
    init.data_type = dataTypeFromString(r["data_type"].as<std::string>("uint16"));
    init.register_count = registerCountForDataType(init.data_type);
    init.value = r["value"].as<double>(0.0);

    if (init.address < 0) {
      RCLCPP_ERROR(logger_, "Device config '%s': init_registers[%zu]: address must be >= 0",
                   path.c_str(), i);
      return false;
    }
    if (init.type != RegisterType::Coil && init.type != RegisterType::HoldingRegister) {
      RCLCPP_ERROR(logger_,
                   "Device config '%s': init_registers[%zu]: type must be coil or holding_register",
                   path.c_str(), i);
      return false;
    }
    out.init_registers.push_back(init);
  }
  return true;
}

bool ModbusDeviceConfigLoader::loadRegisters(const std::string& path, const YAML::Node& root,
                                             ModbusDeviceConfig& out) {
  const YAML::Node regs = root["registers"];
  if (!regs) {
    RCLCPP_ERROR(logger_, "Device config '%s': missing 'registers'", path.c_str());
    return false;
  }
  if (!regs.IsSequence()) {
    RCLCPP_ERROR(logger_, "Device config '%s': registers must be a list", path.c_str());
    return false;
  }
  for (size_t i = 0; i < regs.size(); ++i) {
    const auto& r = regs[i];
    ModbusRegisterConfig reg;
    reg.type = registerTypeFromString(r["type"].as<std::string>("holding_register"));
    const int addr = r["address"].as<int>(0);
    if (addr < 0) {
      RCLCPP_ERROR(logger_, "Device config '%s': registers[%zu]: address must be >= 0",
                   path.c_str(), i);
      return false;
    }
    reg.address = static_cast<uint16_t>(addr);

    const bool has_command = r["command_interface"].IsDefined();
    const bool has_state = r["state_interface"].IsDefined();
    if(has_command && has_state) {
      RCLCPP_ERROR(logger_, "Device config '%s': registers[%zu]: cannot have both state_interface and command_interface (use only one)", path.c_str(), i);
      return false;
    }
    else if(!has_command && !has_state) {
      RCLCPP_ERROR(logger_, "Device config '%s': registers[%zu]: must have either state_interface or command_interface (interface name)", path.c_str(), i);
      return false;
    }
    reg.interface_name = has_command ? r["command_interface"].as<std::string>("") : r["state_interface"].as<std::string>("");
    reg.is_command = has_command;
    reg.data_type = dataTypeFromString(r["data_type"].as<std::string>("uint16"));
    reg.register_count = static_cast<uint16_t>(registerCountForDataType(reg.data_type));
    if (r["factor"].IsDefined()) {
      reg.factor = r["factor"].as<double>(1.0);
    }
    if (r["offset"].IsDefined()) {
      reg.offset = r["offset"].as<double>(0.0);
    }

    if (reg.type == RegisterType::InputRegister || reg.type == RegisterType::DiscreteInput) {
      if (has_command) {
        RCLCPP_ERROR(logger_,
                     "Device config '%s': registers[%zu]: input_register and discrete_input are "
                     "read-only, use state_interface only (not command_interface)",
                     path.c_str(), i);
        return false;
      }
    }
    out.registers.push_back(reg);
  }
  return true;
}


bool ModbusDeviceConfigLoader::validateNoDuplicateRegisterNames(const std::string& path,
                                                                const ModbusDeviceConfig& config) {
  std::unordered_set<std::string> seen_state;
  std::unordered_set<std::string> seen_command;
  for (const auto& r : config.registers) {
    if (r.is_command) {
      if (!seen_command.insert(r.interface_name).second) {
        RCLCPP_ERROR(logger_, "Device config '%s': duplicate command_interface name '%s'",
                     path.c_str(), r.interface_name.c_str());
        return false;
      }
    } else {
      if (!seen_state.insert(r.interface_name).second) {
        RCLCPP_ERROR(logger_, "Device config '%s': duplicate state_interface name '%s'",
                     path.c_str(), r.interface_name.c_str());
        return false;
      }
    }
  }
  return true;
}

}  // namespace modbus_hw_interface
