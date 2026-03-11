// Copyright 2025 modbus_slave_interface contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_slave_interface.hpp
 * @brief Base class for Modbus slave plugins (device config + interface ownership).
 *        HW interface calls setInterfaces() once, then readState()/writeCommand() per cycle.
 */

#ifndef MODBUS_SLAVE_INTERFACE__MODBUS_SLAVE_INTERFACE_HPP_
#define MODBUS_SLAVE_INTERFACE__MODBUS_SLAVE_INTERFACE_HPP_

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modbus_slave_interface/batch_group.hpp"
#include "modbus_slave_interface/modbus_device_config.hpp"

namespace modbus_hw_interface {

/** (interface full name, global buffer index) for state or command. */
using InterfaceNameIndex = std::pair<std::string, size_t>;

/** Callback to write one state value (hardware interface provides this in read()). */
using SetStateCallback = std::function<void(const std::string& name, double value)>;
/** Callback to read one command value (hardware interface provides this in write()). */
using GetCommandCallback = std::function<double(const std::string& name)>;

/**
 * Base class for Modbus slave plugins.
 * After setupSlave(), HW interface calls setInterfaces() so the plugin owns its
 * state/command interface names and global indices. readState()/writeCommand() then
 * run inside the plugin (no per-interface logic in HW interface).
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
   * Set this device's interfaces (name, global index) for state and command (called once by HW
   * interface).
   */
  void setInterfaces(uint8_t device_index, std::vector<InterfaceNameIndex> state_interfaces,
                     std::vector<InterfaceNameIndex> command_interfaces);

  /**
   * Read from state_buffer at own indices and push to state interfaces via set_state.
   * Default implementation: copy slice, call updateState().
   */
  virtual void readState(const std::vector<double>& state_buffer, SetStateCallback set_state);

  /**
   * Read command interfaces via get_command and write into cmd_vals at own indices.
   * Default implementation: call getCommand(), scatter to cmd_vals.
   */
  virtual void writeCommand(GetCommandCallback get_command, std::vector<double>& cmd_vals);

  uint8_t deviceIndex() const {
    return device_index_;
  }
  size_t stateCount() const {
    return state_interfaces_.size();
  }
  size_t commandCount() const {
    return command_interfaces_.size();
  }
  const std::vector<InterfaceNameIndex>& stateInterfaces() const {
    return state_interfaces_;
  }
  const std::vector<InterfaceNameIndex>& commandInterfaces() const {
    return command_interfaces_;
  }
  const std::vector<std::string>& stateNames() const {
    return state_names_;
  }
  const std::vector<std::string>& commandNames() const {
    return command_names_;
  }

  /**
   * Build read (state) batch groups for this device.
   * Returns ready-to-use BatchGroup with buffer allocated.
   */
  std::vector<modbus_slave_interface::BatchGroup> buildReadBatchGroups(
      uint8_t device_index, uint8_t slave_id, const ModbusDeviceConfig& dev,
      const std::vector<std::pair<uint16_t, size_t>>& reg_index_to_global_index) const;

  /**
   * Build write (command) batch groups for this device. Skips read-only types.
   */
  std::vector<modbus_slave_interface::BatchGroup> buildWriteBatchGroups(
      uint8_t device_index, uint8_t slave_id, const ModbusDeviceConfig& dev,
      const std::vector<std::pair<uint16_t, size_t>>& reg_index_to_global_index) const;

  /**
   * Update state interfaces (used internally by readState). Override for custom mapping.
   */
  virtual void updateState(uint8_t device_index, const std::vector<std::string>& state_names,
                           const double* state_values, size_t count,
                           SetStateCallback set_state) = 0;

  /**
   * Fill command values from command interfaces (used internally by writeCommand).
   */
  virtual void getCommand(uint8_t device_index, const std::vector<std::string>& command_names,
                          GetCommandCallback get_command,
                          std::vector<double>& command_values_out) = 0;

 protected:
  uint8_t device_index_{0};
  std::vector<InterfaceNameIndex> state_interfaces_;
  std::vector<InterfaceNameIndex> command_interfaces_;
  std::vector<std::string> state_names_;    // cached .first for updateState
  std::vector<std::string> command_names_;  // cached .first for getCommand
  std::vector<double> state_vals_buffer_;
  std::vector<double> command_out_buffer_;
};

}  // namespace modbus_hw_interface

#endif  // MODBUS_SLAVE_INTERFACE__MODBUS_SLAVE_INTERFACE_HPP_
