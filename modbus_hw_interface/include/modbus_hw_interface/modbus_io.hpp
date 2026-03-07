// Copyright 2025 modbus_hw_interface contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_io.hpp
 * @brief Low-level Modbus read/write of a single register or coil.
 */

#ifndef MODBUS_HW_INTERFACE__MODBUS_IO_HPP_
#define MODBUS_HW_INTERFACE__MODBUS_IO_HPP_

#include <cstddef>
#include <modbus/modbus.h>

#include "modbus_hw_interface/modbus_config.hpp"
#include "modbus_hw_interface/modbus_types.hpp"

namespace modbus_hw_interface
{

/**
 * Decode a value from a raw register buffer (for batch read).
 * @param tab Buffer of 16-bit registers (at least offset + register_count).
 * @param offset Start index in tab.
 * @param register_count 1, 2, or 4.
 * @param data_type How to interpret the value.
 * @return Decoded value as double.
 */
double decodeRegistersFromBuffer(
  const uint16_t * tab, size_t offset, int register_count, RegisterDataType data_type);

/**
 * Read one register or coil and return its value as double.
 * @param ctx Connected Modbus context (slave must already be set).
 * @param reg Register configuration (type, address, data_type, register_count).
 * @return Value as double, or 0.0 on error.
 */
double readRegisterValue(modbus_t * ctx, const ModbusRegisterConfig & reg);

/**
 * Write one register or coil from a double value.
 * @param ctx Connected Modbus context (slave must already be set).
 * @param reg Register configuration (type, address, data_type, register_count).
 * @param value Value to write (coils: non-zero = 1; registers: converted by data_type).
 * @return true on success, false for read-only types or Modbus failure.
 */
bool writeRegisterValue(modbus_t * ctx, const ModbusRegisterConfig & reg, double value);

}  // namespace modbus_hw_interface

#endif  // MODBUS_HW_INTERFACE__MODBUS_IO_HPP_
