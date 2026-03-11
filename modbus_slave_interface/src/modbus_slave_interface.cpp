// Copyright 2025 modbus_slave_interface contributors.
// SPDX-License-Identifier: Apache-2.0

#include "modbus_slave_interface/modbus_slave_interface.hpp"

#include <algorithm>
#include <cstdint>

#include "modbus_slave_interface/batch_group.hpp"
#include "modbus_slave_interface/modbus_device_config.hpp"
#include "modbus_slave_interface/modbus_types.hpp"

namespace modbus_hw_interface {

namespace {

using modbus_slave_interface::BatchGroup;
using modbus_slave_interface::BatchItem;

void buildReadGroupsImpl(uint8_t device_index, uint8_t slave_id, const ModbusDeviceConfig& dev,
                         const std::vector<std::pair<uint16_t, size_t>>& reg_index_to_global,
                         std::vector<BatchGroup>& out) {
  if (reg_index_to_global.empty())
    return;
  std::vector<std::pair<size_t, const ModbusRegisterConfig*>> items;
  items.reserve(reg_index_to_global.size());
  for (const auto& [reg_idx, global_idx] : reg_index_to_global) {
    if (reg_idx >= dev.registers.size())
      continue;
    items.push_back({global_idx, &dev.registers[reg_idx]});
  }
  std::sort(items.begin(), items.end(), [](const auto& a, const auto& b) {
    if (a.second->type != b.second->type)
      return static_cast<int>(a.second->type) < static_cast<int>(b.second->type);
    return a.second->address < b.second->address;
  });
  for (size_t i = 0; i < items.size();) {
    const RegisterType cur_type = items[i].second->type;
    int cur_addr = items[i].second->address;
    BatchGroup grp;
    grp.device_index = device_index;
    grp.slave_id = slave_id;
    grp.type = cur_type;
    grp.start_address = static_cast<uint16_t>(cur_addr);
    grp.use_batch = (cur_type == RegisterType::Coil || cur_type == RegisterType::DiscreteInput)
                        ? dev.read_multiple_coils
                        : dev.read_multiple_registers;
    while (i < items.size() && items[i].second->type == cur_type &&
           items[i].second->address == cur_addr) {
      const ModbusRegisterConfig* reg = items[i].second;
      grp.items.push_back(
          {reg->register_count, reg->data_type, reg, static_cast<uint16_t>(items[i].first)});
      grp.total_count += static_cast<uint16_t>(reg->register_count);
      cur_addr += reg->register_count;
      ++i;
    }
    if (grp.use_batch && grp.total_count > 0) {
      if (cur_type == RegisterType::Coil || cur_type == RegisterType::DiscreteInput)
        grp.buffer.resize(static_cast<size_t>(grp.total_count));
      else
        grp.buffer.resize(static_cast<size_t>(grp.total_count) * sizeof(uint16_t));
    }
    out.push_back(std::move(grp));
  }
}

void buildWriteGroupsImpl(uint8_t device_index, uint8_t slave_id, const ModbusDeviceConfig& dev,
                          const std::vector<std::pair<uint16_t, size_t>>& reg_index_to_global,
                          std::vector<BatchGroup>& out) {
  if (reg_index_to_global.empty())
    return;
  std::vector<std::pair<size_t, const ModbusRegisterConfig*>> items;
  items.reserve(reg_index_to_global.size());
  for (const auto& [reg_idx, global_idx] : reg_index_to_global) {
    if (reg_idx >= dev.registers.size())
      continue;
    const ModbusRegisterConfig* reg = &dev.registers[reg_idx];
    if (reg->type == RegisterType::DiscreteInput || reg->type == RegisterType::InputRegister)
      continue;
    items.push_back({global_idx, reg});
  }
  std::sort(items.begin(), items.end(), [](const auto& a, const auto& b) {
    if (a.second->type != b.second->type)
      return static_cast<int>(a.second->type) < static_cast<int>(b.second->type);
    return a.second->address < b.second->address;
  });
  for (size_t i = 0; i < items.size();) {
    const RegisterType cur_type = items[i].second->type;
    uint16_t cur_addr = items[i].second->address;
    BatchGroup grp;
    grp.device_index = device_index;
    grp.slave_id = slave_id;
    grp.type = cur_type;
    grp.start_address = cur_addr;
    grp.use_batch =
        (cur_type == RegisterType::Coil) ? dev.write_multiple_coils : dev.write_multiple_registers;
    while (i < items.size() && items[i].second->type == cur_type &&
           items[i].second->address == cur_addr) {
      const ModbusRegisterConfig* reg = items[i].second;
      grp.items.push_back(
          {reg->register_count, reg->data_type, reg, static_cast<uint16_t>(items[i].first)});
      grp.total_count += static_cast<uint16_t>(reg->register_count);
      cur_addr += reg->register_count;
      ++i;
    }
    if (grp.use_batch && grp.total_count > 0) {
      if (cur_type == RegisterType::HoldingRegister)
        grp.buffer.resize(static_cast<size_t>(grp.total_count) * sizeof(uint16_t));
      else if (cur_type == RegisterType::Coil)
        grp.buffer.resize(grp.items.size());
    }
    out.push_back(std::move(grp));
  }
}

}  // namespace

std::vector<modbus_slave_interface::BatchGroup> ModbusSlaveInterface::buildReadBatchGroups(
    uint8_t device_index, uint8_t slave_id, const ModbusDeviceConfig& dev,
    const std::vector<std::pair<uint16_t, size_t>>& reg_index_to_global_index) const {
  std::vector<modbus_slave_interface::BatchGroup> out;
  buildReadGroupsImpl(device_index, slave_id, dev, reg_index_to_global_index, out);
  return out;
}

std::vector<modbus_slave_interface::BatchGroup> ModbusSlaveInterface::buildWriteBatchGroups(
    uint8_t device_index, uint8_t slave_id, const ModbusDeviceConfig& dev,
    const std::vector<std::pair<uint16_t, size_t>>& reg_index_to_global_index) const {
  std::vector<modbus_slave_interface::BatchGroup> out;
  buildWriteGroupsImpl(device_index, slave_id, dev, reg_index_to_global_index, out);
  return out;
}

void ModbusSlaveInterface::setInterfaces(uint8_t device_index,
                                         std::vector<InterfaceNameIndex> state_interfaces,
                                         std::vector<InterfaceNameIndex> command_interfaces) {
  device_index_ = device_index;
  state_interfaces_ = std::move(state_interfaces);
  command_interfaces_ = std::move(command_interfaces);
  state_names_.clear();
  state_names_.reserve(state_interfaces_.size());
  for (const auto& p : state_interfaces_) state_names_.push_back(p.first);
  command_names_.clear();
  command_names_.reserve(command_interfaces_.size());
  for (const auto& p : command_interfaces_) command_names_.push_back(p.first);
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
  updateState(device_index_, state_names_, state_vals_buffer_.data(), state_vals_buffer_.size(),
              set_state);
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
