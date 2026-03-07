// Copyright 2025 modbus_hw_interface contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_io.cpp
 * @brief Low-level Modbus read/write of a single register or coil.
 */

#include "modbus_hw_interface/modbus_io.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>

namespace modbus_hw_interface
{

double decodeRegistersFromBuffer(
  const uint16_t * tab, size_t offset, int register_count, RegisterDataType data_type)
{
  if (register_count == 1) {
    if (data_type == RegisterDataType::Int16) {
      return static_cast<double>(static_cast<int16_t>(tab[offset]));
    }
    return static_cast<double>(tab[offset]);
  }
  if (register_count == 2) {
    const uint32_t u32 = (static_cast<uint32_t>(tab[offset]) << 16) | tab[offset + 1];
    if (data_type == RegisterDataType::Float32) {
      float f;
      memcpy(&f, &u32, 4);
      return static_cast<double>(f);
    }
    if (data_type == RegisterDataType::Int32) {
      return static_cast<double>(static_cast<int32_t>(u32));
    }
    return static_cast<double>(u32);
  }
  if (register_count == 4) {
    const uint64_t u64 =
      (static_cast<uint64_t>(tab[offset]) << 48) |
      (static_cast<uint64_t>(tab[offset + 1]) << 32) |
      (static_cast<uint64_t>(tab[offset + 2]) << 16) |
      tab[offset + 3];
    if (data_type == RegisterDataType::Float64) {
      double d;
      memcpy(&d, &u64, 8);
      return d;
    }
    if (data_type == RegisterDataType::Int64) {
      return static_cast<double>(static_cast<int64_t>(u64));
    }
    return static_cast<double>(u64);
  }
  return 0.0;
}

double readRegisterValue(modbus_t * ctx, const ModbusRegisterConfig & reg)
{
  switch (reg.type) {
    case RegisterType::Coil: {
      uint8_t tab[1];
      if (modbus_read_bits(ctx, reg.address, 1, tab) == 1) {
        return tab[0] ? 1.0 : 0.0;
      }
      break;
    }
    case RegisterType::DiscreteInput: {
      uint8_t tab[1];
      if (modbus_read_input_bits(ctx, reg.address, 1, tab) == 1) {
        return tab[0] ? 1.0 : 0.0;
      }
      break;
    }
    case RegisterType::InputRegister:
    case RegisterType::HoldingRegister: {
      uint16_t tab[4];
      int nb = reg.register_count;
      int ret = (reg.type == RegisterType::InputRegister)
        ? modbus_read_input_registers(ctx, reg.address, nb, tab)
        : modbus_read_registers(ctx, reg.address, nb, tab);
      if (ret != nb) break;
      if (nb == 1) {
        if (reg.data_type == RegisterDataType::Int16) {
          return static_cast<double>(static_cast<int16_t>(tab[0]));
        }
        return static_cast<double>(tab[0]);
      }
      if (nb == 2) {
        uint32_t u32 = (static_cast<uint32_t>(tab[0]) << 16) | tab[1];
        int32_t i32 = static_cast<int32_t>(u32);
        if (reg.data_type == RegisterDataType::Float32) {
          float f;
          memcpy(&f, &u32, 4);
          return static_cast<double>(f);
        }
        if (reg.data_type == RegisterDataType::Int32) return static_cast<double>(i32);
        return static_cast<double>(u32);
      }
      if (nb == 4) {
        uint64_t u64 = (static_cast<uint64_t>(tab[0]) << 48) | (static_cast<uint64_t>(tab[1]) << 32) |
          (static_cast<uint64_t>(tab[2]) << 16) | tab[3];
        int64_t i64 = static_cast<int64_t>(u64);
        if (reg.data_type == RegisterDataType::Float64) {
          double d;
          memcpy(&d, &u64, 8);
          return d;
        }
        if (reg.data_type == RegisterDataType::Int64) return static_cast<double>(i64);
        return static_cast<double>(u64);
      }
      break;
    }
    default:
      break;
  }
  return 0.0;
}

bool writeRegisterValue(modbus_t * ctx, const ModbusRegisterConfig & reg, double value)
{
  switch (reg.type) {
    case RegisterType::DiscreteInput:
    case RegisterType::InputRegister:
      return false;  // read-only
    case RegisterType::Coil: {
      int v = (std::fabs(value) > 0.5) ? 1 : 0;
      return modbus_write_bit(ctx, reg.address, v) == 1;
    }
    case RegisterType::HoldingRegister: {
      if (reg.register_count == 1) {
        uint16_t v = static_cast<uint16_t>(value);
        if (reg.data_type == RegisterDataType::Int16) {
          v = static_cast<uint16_t>(static_cast<int16_t>(value));
        }
        return modbus_write_register(ctx, reg.address, v) == 1;
      }
      if (reg.register_count == 2) {
        uint32_t u32;
        if (reg.data_type == RegisterDataType::Float32) {
          float f = static_cast<float>(value);
          memcpy(&u32, &f, 4);
        } else {
          u32 = static_cast<uint32_t>(value);
        }
        uint16_t tab[2] = {static_cast<uint16_t>(u32 >> 16), static_cast<uint16_t>(u32 & 0xFFFF)};
        return modbus_write_registers(ctx, reg.address, 2, tab) == 2;
      }
      if (reg.register_count == 4) {
        uint64_t u64;
        if (reg.data_type == RegisterDataType::Float64) {
          double d = value;
          memcpy(&u64, &d, 8);
        } else if (reg.data_type == RegisterDataType::Int64) {
          u64 = static_cast<uint64_t>(static_cast<int64_t>(value));
        } else {
          u64 = static_cast<uint64_t>(value);
        }
        uint16_t tab[4] = {
          static_cast<uint16_t>(u64 >> 48),
          static_cast<uint16_t>((u64 >> 32) & 0xFFFF),
          static_cast<uint16_t>((u64 >> 16) & 0xFFFF),
          static_cast<uint16_t>(u64 & 0xFFFF),
        };
        return modbus_write_registers(ctx, reg.address, 4, tab) == 4;
      }
      break;
    }
  }
  return false;
}

}  // namespace modbus_hw_interface
