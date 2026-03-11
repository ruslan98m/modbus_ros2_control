// Copyright 2025 modbus_slave_plugins contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_slave_interface.hpp
 * @brief Base class for Modbus slave plugins (device config providers).
 *        Provided by modbus_slave_plugins; used by ModbusSystemInterface to load
 *        device config from plugin when component has "plugin" param in URDF.
 */

#ifndef MODBUS_HW_INTERFACE__MODBUS_SLAVE_INTERFACE_HPP_
#define MODBUS_HW_INTERFACE__MODBUS_SLAVE_INTERFACE_HPP_

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "modbus_slave_plugins/modbus_device_config.hpp"

namespace modbus_hw_interface {

/** Callback to write one state value (hardware interface provides this in read()). */
using SetStateCallback = std::function<void(const std::string& name, double value)>;
/** Callback to read one command value (hardware interface provides this in write()). */
using GetCommandCallback = std::function<double(const std::string& name)>;

/**
 * Base class for Modbus slave plugins.
 * Plugins are loaded from xacro by name (e.g. modbus_slave_plugins/GenericModbusSlave).
 * All plugins have the same structure: they push data to state interfaces and read from
 * command interfaces through a loop over vectors, like EtherCAT slaves (processData).
 * Hardware interface iterates over plugins at read() and write() stages.
 */
class ModbusSlaveInterface {
 public:
  virtual ~ModbusSlaveInterface() = default;

  /**
   * Setup device config from URDF/component parameters.
   * @param name Component name (joint/gpio/sensor name from URDF).
   * @param parameters Params from URDF (e.g. slave_id, slave_config, device_config).
   * @param out Filled device config on success.
   * @return true on success, false on error.
   */
  virtual bool setupSlave(const std::string& name,
                          const std::unordered_map<std::string, std::string>& parameters,
                          ModbusDeviceConfig& out) = 0;

  /**
   * Update state interfaces from read values (called in read() for each device).
   * Plugin loops over state_names/state_values and calls set_state for each.
   */
  virtual void updateState(size_t device_index,
                          const std::vector<std::string>& state_names,
                          const double* state_values,
                          size_t count,
                          SetStateCallback set_state) = 0;

  /**
   * Fill command values from command interfaces (called in write() for each device).
   * Plugin loops over command_names, calls get_command for each, writes to command_values_out.
   */
  virtual void getCommand(size_t device_index,
                          const std::vector<std::string>& command_names,
                          GetCommandCallback get_command,
                          std::vector<double>& command_values_out) = 0;
};

}  // namespace modbus_hw_interface

#endif  // MODBUS_HW_INTERFACE__MODBUS_SLAVE_INTERFACE_HPP_
