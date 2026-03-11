// Copyright 2025 modbus_slave_interface contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_types.hpp
 * @brief Modbus register and data type enums for slave interface.
 */

#ifndef MODBUS_SLAVE_INTERFACE__MODBUS_TYPES_HPP_
#define MODBUS_SLAVE_INTERFACE__MODBUS_TYPES_HPP_

#include <cstddef>

namespace modbus_hw_interface {

/**
 * Modbus register area / function code type.
 * @see https://modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf
 */
enum class RegisterType {
  Coil,           /**< 0x, read/write bits */
  DiscreteInput,  /**< 1x, read-only bits */
  InputRegister,  /**< 3x, read-only 16-bit registers */
  HoldingRegister /**< 4x, read/write 16-bit registers */
};

/**
 * Data type of a value stored in one or more Modbus registers.
 * 32-bit types use 2 registers, 64-bit use 4 registers.
 */
enum class RegisterDataType { Bool, Int16, Uint16, Int32, Uint32, Float32, Int64, Uint64, Float64 };

/** Hash functor for RegisterDataType in std::unordered_map */
struct RegisterDataTypeHash {
  std::size_t operator()(RegisterDataType t) const noexcept {
    return static_cast<std::size_t>(t);
  }
};

}  // namespace modbus_hw_interface

#endif  // MODBUS_SLAVE_INTERFACE__MODBUS_TYPES_HPP_
