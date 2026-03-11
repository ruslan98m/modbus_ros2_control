// Copyright 2025 modbus_slave_plugins contributors.
// SPDX-License-Identifier: Apache-2.0

#include "modbus_slave_plugins/modbus_slave_interface.hpp"

namespace modbus_hw_interface {

void ModbusSlaveInterface::setInterfaces(
    uint8_t device_index,
    std::vector<InterfaceNameIndex> state_interfaces,
    std::vector<InterfaceNameIndex> command_interfaces) {
  device_index_ = device_index;
  state_interfaces_ = std::move(state_interfaces);
  command_interfaces_ = std::move(command_interfaces);
  state_names_.clear();
  state_names_.reserve(state_interfaces_.size());
  for (const auto& p : state_interfaces_)
    state_names_.push_back(p.first);
  command_names_.clear();
  command_names_.reserve(command_interfaces_.size());
  for (const auto& p : command_interfaces_)
    command_names_.push_back(p.first);
  state_vals_buffer_.resize(state_interfaces_.size(), 0.0);
  command_out_buffer_.resize(command_interfaces_.size(), 0.0);
}

void ModbusSlaveInterface::readState(const std::vector<double>& state_buffer,
                                     SetStateCallback set_state) {
  if (state_interfaces_.empty())
    return;
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    size_t g = state_interfaces_[i].second;
    if (g < state_buffer.size())
      state_vals_buffer_[i] = state_buffer[g];
  }
  updateState(device_index_, state_names_, state_vals_buffer_.data(),
              state_vals_buffer_.size(), set_state);
}

void ModbusSlaveInterface::writeCommand(GetCommandCallback get_command,
                                        std::vector<double>& cmd_vals) {
  if (command_interfaces_.empty())
    return;
  getCommand(device_index_, command_names_, get_command, command_out_buffer_);
  for (size_t i = 0; i < command_interfaces_.size() && i < command_out_buffer_.size(); ++i) {
    size_t g = command_interfaces_[i].second;
    if (g < cmd_vals.size())
      cmd_vals[g] = command_out_buffer_[i];
  }
}

}  // namespace modbus_hw_interface
