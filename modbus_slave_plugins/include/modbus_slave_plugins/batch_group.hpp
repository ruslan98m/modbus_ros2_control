// Copyright 2025 modbus_slave_plugins contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file batch_group.hpp
 * @brief Batch of Modbus registers for read/write. Built by device plugins; used by master and HW interface.
 */

#ifndef MODBUS_SLAVE_PLUGINS__BATCH_GROUP_HPP_
#define MODBUS_SLAVE_PLUGINS__BATCH_GROUP_HPP_

#include <cstdint>
#include <vector>

#include "modbus_slave_plugins/modbus_device_config.hpp"
#include "modbus_slave_plugins/modbus_types.hpp"

namespace modbus_slave_plugins {

/** One item in a batch: index into state/command vector and register config pointer. */
struct BatchItem {
  int register_count{1};
  modbus_hw_interface::RegisterDataType data_type{modbus_hw_interface::RegisterDataType::Uint16};
  const modbus_hw_interface::ModbusRegisterConfig* reg{nullptr};
  uint16_t index{0};
};

/** One batch of registers to read or write (same type, same slave, contiguous or single). */
struct BatchGroup {
  uint8_t device_index{0};
  uint8_t slave_id{0};
  modbus_hw_interface::RegisterType type{modbus_hw_interface::RegisterType::HoldingRegister};
  uint16_t start_address{0};
  uint16_t total_count{0};
  bool use_batch{true};
  std::vector<BatchItem> items;
  std::vector<uint8_t> buffer;
};

}  // namespace modbus_slave_plugins

#endif  // MODBUS_SLAVE_PLUGINS__BATCH_GROUP_HPP_
