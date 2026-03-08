// Copyright 2025 modbus_hw_interface contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_config_loader.cpp
 * @brief Load Modbus device configuration from YAML (class implementation).
 */

#include "modbus_hw_interface/modbus_config_loader.hpp"
#include "modbus_hw_interface/modbus_utils.hpp"

#include <algorithm>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace modbus_hw_interface
{

ModbusDeviceConfigLoader::ModbusDeviceConfigLoader(rclcpp::Logger logger)
: logger_(logger)
{}

bool ModbusDeviceConfigLoader::load(const std::string & path, ModbusDeviceConfig & out)
{
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
    validateNoRegisterOverlap(path, out);

    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to load device config '%s': %s", path.c_str(), e.what());
    return false;
  }
}

bool ModbusDeviceConfigLoader::loadOptions(
  const std::string & path, const YAML::Node & root, ModbusDeviceConfig & out)
{
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

bool ModbusDeviceConfigLoader::loadInitRegisters(
  const std::string & path, const YAML::Node & root, ModbusDeviceConfig & out)
{
  const YAML::Node init_list = root["init_registers"];
  if (!init_list) {
    return true;
  }
  if (!init_list.IsSequence()) {
    RCLCPP_ERROR(logger_, "Device config '%s': init_registers must be a list", path.c_str());
    return false;
  }
  for (size_t i = 0; i < init_list.size(); ++i) {
    const auto & r = init_list[i];
    ModbusInitRegisterConfig init;
    init.type = registerTypeFromString(r["type"].as<std::string>("holding_register"));
    init.address = r["address"].as<int>(0);
    init.data_type = dataTypeFromString(r["data_type"].as<std::string>("uint16"));
    init.register_count = registerCountForDataType(init.data_type);
    init.value = r["value"].as<double>(0.0);

    if (init.address < 0) {
      RCLCPP_ERROR(logger_,
        "Device config '%s': init_registers[%zu]: address must be >= 0", path.c_str(), i);
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

bool ModbusDeviceConfigLoader::loadRegisters(
  const std::string & path, const YAML::Node & root, ModbusDeviceConfig & out)
{
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
    const auto & r = regs[i];
    ModbusRegisterConfig reg;
    reg.name = r["name"].as<std::string>("");
    reg.type = registerTypeFromString(r["type"].as<std::string>("holding_register"));
    reg.address = r["address"].as<int>(0);
    reg.is_command = r["interface"].as<std::string>("state") == "command";
    reg.data_type = dataTypeFromString(r["data_type"].as<std::string>("uint16"));
    reg.register_count = registerCountForDataType(reg.data_type);

    if (reg.name.empty()) {
      RCLCPP_ERROR(logger_,
        "Device config '%s': registers[%zu]: name must be non-empty", path.c_str(), i);
      return false;
    }
    if (reg.address < 0) {
      RCLCPP_ERROR(logger_,
        "Device config '%s': registers[%zu] '%s': address must be >= 0",
        path.c_str(), i, reg.name.c_str());
      return false;
    }
    out.registers.push_back(reg);
  }
  return true;
}

bool ModbusDeviceConfigLoader::validateNoRegisterOverlap(
  const std::string & path, const ModbusDeviceConfig & config)
{
  struct RegRange
  {
    int address;
    int register_count;
    std::string name;
    int end() const { return address + register_count; }
  };

  const auto & regs = config.registers;
  for (int t = 0; t < 4; ++t) {
    const auto type = static_cast<RegisterType>(t);
    std::vector<RegRange> by_type;
    by_type.reserve(regs.size());
    for (const auto & r : regs) {
      if (r.type == type) {
        by_type.push_back({r.address, r.register_count, r.name});
      }
    }
    std::sort(by_type.begin(), by_type.end(),
      [](const RegRange & a, const RegRange & b) { return a.address < b.address; });

    const auto it = std::adjacent_find(by_type.begin(), by_type.end(),
      [](const RegRange & a, const RegRange & b) { return a.end() > b.address; });
    if (it != by_type.end()) {
      const auto & a = *it;
      const auto & b = *(it + 1);
      RCLCPP_WARN(logger_,
        "Device config '%s': register overlap: '%s' [%d, %d) and '%s' [%d, %d)",
        path.c_str(), a.name.c_str(), a.address, a.end(), b.name.c_str(), b.address, b.end());
      return false;
    }
  }
  return true;
}

bool ModbusDeviceConfigLoader::validateNoDuplicateRegisterNames(
  const std::string & path, const ModbusDeviceConfig & config)
{
  std::unordered_set<std::string> seen;
  for (const auto & r : config.registers) {
    if (!seen.insert(r.name).second) {
      RCLCPP_ERROR(logger_,
        "Device config '%s': duplicate register name '%s'", path.c_str(), r.name.c_str());
      return false;
    }
  }
  return true;
}

}  // namespace modbus_hw_interface
