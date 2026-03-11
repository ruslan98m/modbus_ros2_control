// Copyright 2025 modbus_slave_plugins contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_utils.hpp
 * @brief Helper functions for parsing and converting Modbus type strings (provided by modbus_slave_plugins).
 */

#ifndef MODBUS_HW_INTERFACE__MODBUS_UTILS_HPP_
#define MODBUS_HW_INTERFACE__MODBUS_UTILS_HPP_

#include <string>
#include <vector>

#include "modbus_slave_plugins/modbus_types.hpp"
#include "rclcpp/logger.hpp"

namespace modbus_hw_interface {

RegisterDataType dataTypeFromString(const std::string& s);
RegisterType registerTypeFromString(const std::string& s);
std::string dataTypeToInterfaceString(RegisterDataType t);
int registerCountForDataType(RegisterDataType t);
void applyRealtimeThreadParams(rclcpp::Logger logger, int thread_priority,
                               const std::vector<int>& cpu_affinity_cores);

}  // namespace modbus_hw_interface

#endif  // MODBUS_HW_INTERFACE__MODBUS_UTILS_HPP_
