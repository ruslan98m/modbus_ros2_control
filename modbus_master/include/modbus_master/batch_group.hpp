// Copyright 2025 modbus_master contributors.
// SPDX-License-Identifier: Apache-2.0

#ifndef MODBUS_MASTER__BATCH_GROUP_HPP_
#define MODBUS_MASTER__BATCH_GROUP_HPP_

#include <cstdint>
#include <vector>

#include "modbus_slave_plugins/modbus_device_config.hpp"
#include "modbus_slave_plugins/modbus_types.hpp"

namespace modbus_master {

using modbus_hw_interface::RegisterDataType;
using modbus_hw_interface::RegisterType;

/** One item in a batch: index into state/command vector and register config pointer. */
struct BatchItem {
  int register_count;
  RegisterDataType data_type;
  const modbus_hw_interface::ModbusRegisterConfig* reg;
  uint16_t index;
};

/** One batch of registers to read or write (same type, same slave, contiguous or single). */
struct BatchGroup {
  uint8_t device_index;
  uint8_t slave_id;
  RegisterType type;
  uint16_t start_address;
  uint16_t total_count;
  bool use_batch;
  std::vector<BatchItem> items;
  std::vector<uint8_t> buffer;
};

}  // namespace modbus_master

#endif  // MODBUS_MASTER__BATCH_GROUP_HPP_
