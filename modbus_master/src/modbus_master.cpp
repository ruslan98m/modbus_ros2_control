// Copyright 2025 modbus_master contributors.
// SPDX-License-Identifier: Apache-2.0

#include "modbus_master/modbus_master.hpp"

#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>

#include "modbus_master/modbus_master_helpers.hpp"
#include "modbus_slave_interface/modbus_device_config.hpp"
#include "modbus_slave_interface/modbus_types.hpp"
#include "rclcpp/rclcpp.hpp"
#include <modbus/modbus.h>

namespace modbus_master {

using modbus_hw_interface::ModbusDeviceConfig;
using modbus_hw_interface::ModbusInitRegisterConfig;
using modbus_hw_interface::ModbusRegisterConfig;
using modbus_hw_interface::RegisterDataType;
using modbus_hw_interface::RegisterType;

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

bool ModbusMaster::initFromParams(const MasterParams& params) {
  params_ = params;
  return true;
}

bool ModbusMaster::connect() {
  if (ctx_)
    return true;
  if (params_.is_tcp) {
    return connect(params_.tcp_params);
  }
  return connect(params_.rtu_params);
}

void ModbusMaster::resetInitRegisters() {
  init_registers_done_.store(false);
}

void ModbusMaster::setPlugins(
    std::vector<std::shared_ptr<modbus_hw_interface::ModbusSlaveInterface>> plugins) {
  plugins_ = std::move(plugins);
}

void ModbusMaster::setRegisterMappings(
    std::vector<std::vector<std::pair<uint16_t, size_t>>> state_mappings,
    std::vector<std::vector<std::pair<uint16_t, size_t>>> command_mappings) {
  state_register_mappings_ = std::move(state_mappings);
  command_register_mappings_ = std::move(command_mappings);
}

void ModbusMaster::buildPollGroups() {
  read_batch_groups_.clear();
  write_batch_groups_.clear();
  if (!devices_ || plugins_.size() != devices_->size() ||
      plugins_.size() != state_register_mappings_.size() ||
      plugins_.size() != command_register_mappings_.size())
    return;
  for (size_t dev_idx = 0; dev_idx < plugins_.size(); ++dev_idx) {
    const auto& dev = (*devices_)[dev_idx];
    const uint8_t slave_id = static_cast<uint8_t>(dev.slave_id);
    const uint8_t device_index = static_cast<uint8_t>(dev_idx);
    auto read_groups = plugins_[dev_idx]->buildReadBatchGroups(
        device_index, slave_id, dev, state_register_mappings_[dev_idx]);
    for (auto& g : read_groups)
      read_batch_groups_.push_back(std::move(g));
    auto write_groups = plugins_[dev_idx]->buildWriteBatchGroups(
        device_index, slave_id, dev, command_register_mappings_[dev_idx]);
    for (auto& g : write_groups)
      write_batch_groups_.push_back(std::move(g));
  }
}

void ModbusMaster::pollDevices() {
  if (!ctx_ || !devices_)
    return;
  readStateBatched(read_batch_groups_, *devices_, state_poll_buffer_);
  state_buffer_.writeFromNonRT(state_poll_buffer_);
  std::vector<double> cmd = get_command_();
  if (cmd.size() == command_count_)
    writeCommandBatched(write_batch_groups_, *devices_, cmd);
}

void ModbusMaster::startPollLoop(
    size_t state_count,
    size_t command_count,
    const std::vector<ModbusDeviceConfig>& devices,
    std::function<std::vector<double>()> get_command) {
  if (poll_thread_.joinable())
    return;
  if (!connect()) {
    RCLCPP_ERROR(rclcpp::get_logger("modbus_master"), "startPollLoop: connect failed");
    return;
  }
  state_count_ = state_count;
  command_count_ = command_count;
  devices_ = &devices;
  get_command_ = std::move(get_command);
  state_poll_buffer_.resize(state_count_, 0.0);
  state_buffer_.initRT(std::vector<double>(state_count_, 0.0));

  buildPollGroups();

  poll_running_.store(true);
  poll_thread_ = std::thread(&ModbusMaster::pollThreadLoop, this);
}

void ModbusMaster::stopPollLoop() {
  poll_running_.store(false);
  if (poll_thread_.joinable()) {
    poll_thread_.join();
    poll_thread_ = std::thread();
  }
  devices_ = nullptr;
}

const std::vector<double>* ModbusMaster::readStateSnapshotForRT() const {
  return state_buffer_.readFromRT();
}

void ModbusMaster::pollThreadLoop() {
  const auto logger = rclcpp::get_logger("modbus_master");
  detail::applyRealtimeThreadParams(logger, params_.thread_priority, params_.cpu_affinity_cores);

  const bool use_poll_delay = (params_.poll_rate_hz > 0.0);
  const auto period = use_poll_delay
                          ? std::chrono::duration<double>(1.0 / params_.poll_rate_hz)
                          : std::chrono::duration<double>(0);

  if (!ctx_ || !devices_ || !get_command_)
    return;

  if (!init_registers_done_.exchange(true)) {
    writeInitRegisters(*devices_);
  }

  while (poll_running_.load(std::memory_order_relaxed)) {
    const auto iteration_start = std::chrono::steady_clock::now();

    pollDevices();

    if (use_poll_delay) {
      const auto elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - iteration_start);
      const auto remaining = period - elapsed;
      if (remaining.count() > 0) {
        std::this_thread::sleep_for(remaining);
      } else {
        RCLCPP_WARN(logger, "Poll delay: elapsed %g > period %g", elapsed.count(), period.count());
      }
    }
  }
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
      detail::setResponseTimeout(ctx, devices[grp.device_index]);
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
                detail::decodeRegistersFromBuffer(regs, off, it.register_count, it.data_type);
            off += static_cast<size_t>(it.register_count);
          }
        }
      }
    } else {
      for (const auto& it : grp.items) {
        state_vals[it.index] = detail::readRegisterValue(ctx, *it.reg);
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
      detail::setResponseTimeout(ctx, devices[grp.device_index]);
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
        detail::writeRegisterValue(ctx, *it.reg, command_vals[it.index]);
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
    detail::setResponseTimeout(ctx, dev);
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
      if (detail::writeRegisterValue(ctx, reg, init.value)) {
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
