// Copyright 2025 modbus_hw_interface contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_config.hpp
 * @brief Modbus bus configuration. Device config types come from modbus_slave_plugins.
 */

#ifndef MODBUS_HW_INTERFACE__MODBUS_CONFIG_HPP_
#define MODBUS_HW_INTERFACE__MODBUS_CONFIG_HPP_

#include <string>
#include <vector>

#include "modbus_master/connection_params.hpp"
#include "modbus_slave_plugins/modbus_device_config.hpp"

namespace modbus_hw_interface {

/** One bus = one connection (either RTU or TCP, not both). Uses connection params from modbus_master. */
struct ModbusBusConfig {
  std::string bus_name;
  bool is_tcp{true};
  modbus_master::TcpConnectionParams tcp_params;
  modbus_master::RtuConnectionParams rtu_params;
  std::vector<ModbusDeviceConfig> devices;
};

}  // namespace modbus_hw_interface

#endif  // MODBUS_HW_INTERFACE__MODBUS_CONFIG_HPP_
