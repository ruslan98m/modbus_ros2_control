// Copyright 2025 modbus_master contributors.
// SPDX-License-Identifier: Apache-2.0

#include "modbus_master/modbus_master.hpp"

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>

#include "modbus_slave_plugins/modbus_device_config.hpp"
#include "modbus_slave_plugins/modbus_types.hpp"
#include "rclcpp/rclcpp.hpp"
#include <modbus/modbus.h>

namespace modbus_master {

using modbus_hw_interface::ModbusDeviceConfig;
using modbus_hw_interface::ModbusInitRegisterConfig;
using modbus_hw_interface::ModbusRegisterConfig;
using modbus_hw_interface::RegisterDataType;
using modbus_hw_interface::RegisterType;

namespace {

double decodeRegistersFromBuffer(const uint16_t* tab, size_t offset, int register_count,
                                 RegisterDataType data_type) {
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
        (static_cast<uint64_t>(tab[offset + 2]) << 16) | tab[offset + 3];
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

double readRegisterValue(modbus_t* ctx, const ModbusRegisterConfig& reg) {
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
      if (ret != nb)
        break;
      if (nb == 1) {
        if (reg.data_type == RegisterDataType::Int16) {
          return static_cast<double>(static_cast<int16_t>(tab[0]));
        }
        return static_cast<double>(tab[0]);
      }
      if (nb == 2) {
        uint32_t u32 = (static_cast<uint32_t>(tab[0]) << 16) | tab[1];
        if (reg.data_type == RegisterDataType::Float32) {
          float f;
          memcpy(&f, &u32, 4);
          return static_cast<double>(f);
        }
        if (reg.data_type == RegisterDataType::Int32) {
          return static_cast<double>(static_cast<int32_t>(u32));
        }
        return static_cast<double>(u32);
      }
      if (nb == 4) {
        uint64_t u64 = (static_cast<uint64_t>(tab[0]) << 48) |
                       (static_cast<uint64_t>(tab[1]) << 32) |
                       (static_cast<uint64_t>(tab[2]) << 16) | tab[3];
        if (reg.data_type == RegisterDataType::Float64) {
          double d;
          memcpy(&d, &u64, 8);
          return d;
        }
        if (reg.data_type == RegisterDataType::Int64) {
          return static_cast<double>(static_cast<int64_t>(u64));
        }
        return static_cast<double>(u64);
      }
      break;
    }
    default:
      break;
  }
  return 0.0;
}

bool writeRegisterValue(modbus_t* ctx, const ModbusRegisterConfig& reg, double value) {
  switch (reg.type) {
    case RegisterType::DiscreteInput:
    case RegisterType::InputRegister:
      return false;
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
        uint16_t tab[2] = {static_cast<uint16_t>(u32 >> 16),
                           static_cast<uint16_t>(u32 & 0xFFFF)};
        return modbus_write_registers(ctx, reg.address, 2, tab) == 2;
      }
      if (reg.register_count == 4) {
        uint64_t u64;
        if (reg.data_type == RegisterDataType::Float64) {
          memcpy(&u64, &value, 8);
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

void setResponseTimeout(modbus_t* ctx, const ModbusDeviceConfig& dev) {
  if (dev.response_timeout_sec <= 0)
    return;
  double s = dev.response_timeout_sec;
  uint32_t to_sec = static_cast<uint32_t>(s);
  uint32_t to_usec = static_cast<uint32_t>((s - to_sec) * 1e6);
  modbus_set_response_timeout(ctx, to_sec, to_usec);
}

}  // namespace

bool ModbusMaster::connect(const TcpConnectionParams& params) {
  if (ctx_) {
    return true;
  }
  modbus_t* raw = modbus_new_tcp(params.ip_address.c_str(), params.port);
  if (!raw) {
    RCLCPP_ERROR(rclcpp::get_logger("modbus_master"), "Failed to create Modbus TCP context");
    return false;
  }
  if (modbus_connect(raw) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("modbus_master"), "Failed to connect TCP: %s",
                 modbus_strerror(errno));
    modbus_free(raw);
    return false;
  }
  ctx_.reset(raw);
  return true;
}

bool ModbusMaster::connect(const RtuConnectionParams& params) {
  if (ctx_) {
    return true;
  }
  modbus_t* raw = modbus_new_rtu(params.serial_port.c_str(), static_cast<int>(params.baud_rate),
                                 params.parity, params.data_bits, params.stop_bits);
  if (!raw) {
    RCLCPP_ERROR(rclcpp::get_logger("modbus_master"), "Failed to create Modbus RTU context");
    return false;
  }
  if (modbus_connect(raw) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("modbus_master"), "Failed to connect RTU: %s",
                 modbus_strerror(errno));
    modbus_free(raw);
    return false;
  }
  ctx_.reset(raw);
  return true;
}

void ModbusMaster::disconnect() {
  ctx_.reset();
}

void ModbusMaster::readStateBatched(
    std::vector<BatchGroup>& read_groups,
    const std::vector<ModbusDeviceConfig>& devices,
    std::vector<double>& state_vals) {
  if (!ctx_)
    return;
  modbus_t* ctx = ctx_.get();
  for (auto& grp : read_groups) {
    if (grp.device_index < devices.size()) {
      setResponseTimeout(ctx, devices[grp.device_index]);
    }
    if (modbus_set_slave(ctx, static_cast<int>(grp.slave_id)) < 0)
      continue;
    if (grp.use_batch && grp.total_count > 0 && !grp.buffer.empty()) {
      if (grp.type == RegisterType::Coil) {
        if (modbus_read_bits(ctx, grp.start_address, grp.total_count, grp.buffer.data()) ==
            static_cast<int>(grp.total_count)) {
          size_t off = 0;
          for (const auto& it : grp.items) {
            state_vals[it.index] = grp.buffer[off] ? 1.0 : 0.0;
            off += static_cast<size_t>(it.register_count);
          }
        }
      } else if (grp.type == RegisterType::DiscreteInput) {
        if (modbus_read_input_bits(ctx, grp.start_address, grp.total_count,
                                   grp.buffer.data()) == static_cast<int>(grp.total_count)) {
          size_t off = 0;
          for (const auto& it : grp.items) {
            state_vals[it.index] = grp.buffer[off] ? 1.0 : 0.0;
            off += static_cast<size_t>(it.register_count);
          }
        }
      } else {
        uint16_t* regs = reinterpret_cast<uint16_t*>(grp.buffer.data());
        int ret = (grp.type == RegisterType::InputRegister)
                      ? modbus_read_input_registers(ctx, grp.start_address, grp.total_count, regs)
                      : modbus_read_registers(ctx, grp.start_address, grp.total_count, regs);
        if (ret == static_cast<int>(grp.total_count)) {
          size_t off = 0;
          for (const auto& it : grp.items) {
            state_vals[it.index] =
                decodeRegistersFromBuffer(regs, off, it.register_count, it.data_type);
            off += static_cast<size_t>(it.register_count);
          }
        }
      }
    } else {
      for (const auto& it : grp.items) {
        state_vals[it.index] = readRegisterValue(ctx, *it.reg);
      }
    }
  }
}

void ModbusMaster::writeCommandBatched(
    std::vector<BatchGroup>& write_groups,
    const std::vector<ModbusDeviceConfig>& devices,
    const std::vector<double>& command_vals) {
  if (!ctx_)
    return;
  modbus_t* ctx = ctx_.get();
  for (auto& grp : write_groups) {
    if (grp.device_index < devices.size()) {
      setResponseTimeout(ctx, devices[grp.device_index]);
    }
    if (modbus_set_slave(ctx, static_cast<int>(grp.slave_id)) < 0)
      continue;
    if (grp.use_batch && grp.total_count > 0 && grp.type == RegisterType::HoldingRegister &&
        !grp.buffer.empty()) {
      uint16_t* regs = reinterpret_cast<uint16_t*>(grp.buffer.data());
      size_t off = 0;
      for (const auto& it : grp.items) {
        const double val = command_vals[it.index];
        const ModbusRegisterConfig* reg = it.reg;
        if (reg->register_count == 1) {
          regs[off] = (reg->data_type == RegisterDataType::Int16)
                          ? static_cast<uint16_t>(static_cast<int16_t>(val))
                          : static_cast<uint16_t>(val);
        } else if (reg->register_count == 2) {
          uint32_t u32;
          if (reg->data_type == RegisterDataType::Float32) {
            float f = static_cast<float>(val);
            memcpy(&u32, &f, 4);
          } else {
            u32 = static_cast<uint32_t>(val);
          }
          regs[off] = static_cast<uint16_t>(u32 >> 16);
          regs[off + 1] = static_cast<uint16_t>(u32 & 0xFFFF);
        } else if (reg->register_count == 4) {
          uint64_t u64;
          if (reg->data_type == RegisterDataType::Float64) {
            memcpy(&u64, &val, 8);
          } else if (reg->data_type == RegisterDataType::Int64) {
            u64 = static_cast<uint64_t>(static_cast<int64_t>(val));
          } else {
            u64 = static_cast<uint64_t>(val);
          }
          regs[off] = static_cast<uint16_t>(u64 >> 48);
          regs[off + 1] = static_cast<uint16_t>((u64 >> 32) & 0xFFFF);
          regs[off + 2] = static_cast<uint16_t>((u64 >> 16) & 0xFFFF);
          regs[off + 3] = static_cast<uint16_t>(u64 & 0xFFFF);
        }
        off += static_cast<size_t>(reg->register_count);
      }
      modbus_write_registers(ctx, grp.start_address, grp.total_count, regs);
    } else if (grp.use_batch && grp.type == RegisterType::Coil && !grp.buffer.empty()) {
      for (size_t g = 0; g < grp.buffer.size(); ++g) {
        grp.buffer[g] = (std::fabs(command_vals[grp.items[g].index]) > 0.5) ? 1 : 0;
      }
      modbus_write_bits(ctx, grp.start_address, static_cast<int>(grp.buffer.size()),
                        grp.buffer.data());
    } else {
      for (const auto& it : grp.items) {
        writeRegisterValue(ctx, *it.reg, command_vals[it.index]);
      }
    }
  }
}

void ModbusMaster::writeInitRegisters(
    const std::vector<ModbusDeviceConfig>& devices) {
  if (!ctx_)
    return;
  modbus_t* ctx = ctx_.get();
  const auto logger = rclcpp::get_logger("modbus_master");
  for (size_t di = 0; di < devices.size(); ++di) {
    const auto& dev = devices[di];
    if (dev.init_registers.empty())
      continue;
    setResponseTimeout(ctx, dev);
    if (modbus_set_slave(ctx, dev.slave_id) < 0) {
      RCLCPP_WARN(logger, "Init registers: failed to set slave_id %d", dev.slave_id);
      continue;
    }
    for (const auto& init : dev.init_registers) {
      ModbusRegisterConfig reg;
      reg.type = init.type;
      reg.address = static_cast<uint16_t>(init.address);
      reg.data_type = init.data_type;
      reg.register_count = static_cast<uint16_t>(init.register_count);
      if (writeRegisterValue(ctx, reg, init.value)) {
        RCLCPP_DEBUG(logger, "Init write dev '%s' addr %d = %g", dev.name.c_str(), init.address,
                     init.value);
      } else {
        RCLCPP_WARN(logger, "Init write failed dev '%s' type %d addr %d", dev.name.c_str(),
                    static_cast<int>(init.type), init.address);
      }
    }
  }
}

}  // namespace modbus_master
