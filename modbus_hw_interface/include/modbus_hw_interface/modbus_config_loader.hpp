// Copyright 2025 modbus_hw_interface contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_config_loader.hpp
 * @brief Load Modbus device configuration from YAML files.
 */

#ifndef MODBUS_HW_INTERFACE__MODBUS_CONFIG_LOADER_HPP_
#define MODBUS_HW_INTERFACE__MODBUS_CONFIG_LOADER_HPP_

#include <string>

#include "modbus_hw_interface/modbus_config.hpp"

namespace modbus_hw_interface
{

/**
 * Load device configuration from a YAML file (registers, init_registers, read/write multiple flags).
 * @param path Path to the device YAML config file.
 * @param out Filled on success; registers and init_registers are cleared and repopulated.
 * @return true on success, false on parse error or missing required keys.
 */
bool loadDeviceConfigFromFile(const std::string & path, ModbusDeviceConfig & out);

}  // namespace modbus_hw_interface

#endif  // MODBUS_HW_INTERFACE__MODBUS_CONFIG_LOADER_HPP_
