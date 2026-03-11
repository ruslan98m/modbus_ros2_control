// Copyright 2025 modbus_master contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_master_helpers.hpp
 * @brief Internal helpers for ModbusMaster: RT thread params, register decode/encode, timeout.
 */

#ifndef MODBUS_MASTER__MODBUS_MASTER_HELPERS_HPP_
#define MODBUS_MASTER__MODBUS_MASTER_HELPERS_HPP_

#include <modbus/modbus.h>

#include <cstddef>
#include <vector>

#include "modbus_slave_interface/modbus_device_config.hpp"
#include "modbus_slave_interface/modbus_types.hpp"
#include "rclcpp/logger.hpp"

namespace modbus_master {
namespace detail {

void applyRealtimeThreadParams(rclcpp::Logger logger, int thread_priority,
                               const std::vector<int>& cpu_affinity_cores);

double decodeRegistersFromBuffer(const uint16_t* tab, size_t offset, int register_count,
                                 modbus_hw_interface::RegisterDataType data_type);

double readRegisterValue(modbus_t* ctx, const modbus_hw_interface::ModbusRegisterConfig& reg);

bool writeRegisterValue(modbus_t* ctx, const modbus_hw_interface::ModbusRegisterConfig& reg,
                        double value);

void setResponseTimeout(modbus_t* ctx, const modbus_hw_interface::ModbusDeviceConfig& dev);

}  // namespace detail
}  // namespace modbus_master

#endif  // MODBUS_MASTER__MODBUS_MASTER_HELPERS_HPP_
