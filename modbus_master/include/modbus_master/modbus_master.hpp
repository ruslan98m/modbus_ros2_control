// Copyright 2025 modbus_master contributors.
// SPDX-License-Identifier: Apache-2.0

#ifndef MODBUS_MASTER__MODBUS_MASTER_HPP_
#define MODBUS_MASTER__MODBUS_MASTER_HPP_

#include <memory>
#include <vector>

#include "modbus_master/batch_group.hpp"
#include "modbus_master/connection_params.hpp"
#include "modbus_slave_plugins/modbus_device_config.hpp"
#include <modbus/modbus.h>

namespace modbus_master {

struct ModbusContextDeleter {
  void operator()(modbus_t* p) const {
    if (p) {
      modbus_close(p);
      modbus_free(p);
    }
  }
};

/**
 * Modbus client: one connection (TCP or RTU), batch read/write, init registers.
 */
class ModbusMaster {
 public:
  ModbusMaster() = default;

  ModbusMaster(const ModbusMaster&) = delete;
  ModbusMaster& operator=(const ModbusMaster&) = delete;

  /** Connect via TCP. Returns true on success. */
  bool connect(const TcpConnectionParams& params);

  /** Connect via RTU (serial). Returns true on success. */
  bool connect(const RtuConnectionParams& params);

  /** Disconnect and free context. */
  void disconnect();

  bool isConnected() const { return ctx_ != nullptr; }

  /**
   * Read state registers/coils into state_vals using batch groups.
   * devices[] is used for response_timeout_sec per device.
   */
  void readStateBatched(std::vector<BatchGroup>& read_groups,
                        const std::vector<modbus_hw_interface::ModbusDeviceConfig>& devices,
                        std::vector<double>& state_vals);

  /** Write command registers/coils from command_vals using batch groups (write_groups buffers are filled). */
  void writeCommandBatched(std::vector<BatchGroup>& write_groups,
                           const std::vector<modbus_hw_interface::ModbusDeviceConfig>& devices,
                           const std::vector<double>& command_vals);

  /** Write init_registers for each device once at startup. */
  void writeInitRegisters(const std::vector<modbus_hw_interface::ModbusDeviceConfig>& devices);

 private:
  std::unique_ptr<modbus_t, ModbusContextDeleter> ctx_;
};

}  // namespace modbus_master

#endif  // MODBUS_MASTER__MODBUS_MASTER_HPP_
