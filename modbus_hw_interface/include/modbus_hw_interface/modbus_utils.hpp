// Copyright 2025 modbus_hw_interface contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_utils.hpp
 * @brief Helper functions for parsing and converting Modbus type strings (YAML/config).
 */

#ifndef MODBUS_HW_INTERFACE__MODBUS_UTILS_HPP_
#define MODBUS_HW_INTERFACE__MODBUS_UTILS_HPP_

#include <string>

#include "modbus_hw_interface/modbus_types.hpp"

namespace modbus_hw_interface
{

/**
 * Parse a string to RegisterDataType.
 * Supports full names (e.g. "uint16", "float32") and short aliases (e.g. "u16", "f32", "b").
 * Case-insensitive. i8/u8 map to Int16/Uint16 (one Modbus register).
 * @param s Input string from YAML or config.
 * @return Parsed type, or RegisterDataType::Uint16 if unknown.
 */
RegisterDataType dataTypeFromString(const std::string & s);

/**
 * Parse a string to RegisterType.
 * Supports full names ("coil", "holding_register") and short aliases ("c", "hr").
 * Case-insensitive.
 * @param s Input string from YAML or config.
 * @return Parsed type, or RegisterType::HoldingRegister if unknown.
 */
RegisterType registerTypeFromString(const std::string & s);

/**
 * Return the canonical string for a data type (for interface export / logging).
 * @param t Data type.
 * @return String like "int16", "float32", or "double" for unknown.
 */
std::string dataTypeToInterfaceString(RegisterDataType t);

/**
 * Number of 16-bit Modbus registers needed to store the given data type.
 * @param t Data type.
 * @return 1 for 16-bit/bool, 2 for 32-bit/float32, 4 for 64-bit/float64.
 */
int registerCountForDataType(RegisterDataType t);

}  // namespace modbus_hw_interface

#endif  // MODBUS_HW_INTERFACE__MODBUS_UTILS_HPP_
