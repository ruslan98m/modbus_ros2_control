// Copyright 2025 modbus_master contributors.
// SPDX-License-Identifier: Apache-2.0

#ifndef MODBUS_MASTER__MODBUS_MASTER_HPP_
#define MODBUS_MASTER__MODBUS_MASTER_HPP_

#include <modbus/modbus.h>

#include <atomic>
#include <functional>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

#include "modbus_master/master_params.hpp"
#include "modbus_slave_interface/batch_group.hpp"
#include "modbus_slave_interface/modbus_device_config.hpp"
#include "modbus_slave_interface/modbus_slave_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace modbus_master {

using BatchGroup = modbus_slave_interface::BatchGroup;

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
 * Owns poll thread and parses connection + poll params from a params map.
 */
class ModbusMaster {
 public:
  ModbusMaster() = default;

  ModbusMaster(const ModbusMaster&) = delete;
  ModbusMaster& operator=(const ModbusMaster&) = delete;

  /** Set connection and poll params (parsed by hardware interface). Call before
   * connect/startPollLoop. */
  bool initFromParams(const MasterParams& params);

  /** Connect using params from initFromParams. Returns true on success. */
  bool connect();

  /** Connect via TCP. Returns true on success. */
  bool connect(const TcpConnectionParams& params);

  /** Connect via RTU (serial). Returns true on success. */
  bool connect(const RtuConnectionParams& params);

  /** Disconnect and free context. */
  void disconnect();

  bool isConnected() const {
    return ctx_ != nullptr;
  }

  /** Reset so that next startPollLoop run will call writeInitRegisters once. */
  void resetInitRegisters();

  /** Set device plugins (one per device). Call before startPollLoop. */
  void setPlugins(std::vector<std::shared_ptr<modbus_hw_interface::ModbusSlaveInterface>> plugins);

  /**
   * Set per-device register mappings: (reg_index, global_index) for state and command.
   * Call before startPollLoop after setPlugins.
   */
  void setRegisterMappings(std::vector<std::vector<std::pair<uint16_t, size_t>>> state_mappings,
                           std::vector<std::vector<std::pair<uint16_t, size_t>>> command_mappings);

  /**
   * Start poll thread. Builds read/write groups from plugins and mappings, then polls.
   * State is written to internal RT buffer; get_command() provides command_vals for write.
   * Call readStateSnapshotForRT() from RT context to read the latest state.
   */
  void startPollLoop(size_t state_count, size_t command_count,
                     const std::vector<modbus_hw_interface::ModbusDeviceConfig>& devices,
                     std::function<std::vector<double>()> get_command);

  /**
   * Read latest state snapshot from poll thread (call from real-time context only).
   * Returns nullptr if not ready or size mismatch.
   */
  const std::vector<double>* readStateSnapshotForRT() const;

  /** Stop poll thread. */
  void stopPollLoop();

  /** True if poll thread is running. */
  bool isPollLoopRunning() const {
    return poll_thread_.joinable();
  }

  /** Write init_registers for each device once at startup. */
  void writeInitRegisters(const std::vector<modbus_hw_interface::ModbusDeviceConfig>& devices);

 private:
  void buildPollGroups();
  void pollDevices();
  void pollThreadLoop();
  void readStateBatched(std::vector<BatchGroup>& read_groups,
                        const std::vector<modbus_hw_interface::ModbusDeviceConfig>& devices,
                        std::vector<double>& state_vals);
  void writeCommandBatched(std::vector<BatchGroup>& write_groups,
                           const std::vector<modbus_hw_interface::ModbusDeviceConfig>& devices,
                           const std::vector<double>& command_vals);

  std::vector<std::shared_ptr<modbus_hw_interface::ModbusSlaveInterface>> plugins_;
  std::vector<std::vector<std::pair<uint16_t, size_t>>> state_register_mappings_;
  std::vector<std::vector<std::pair<uint16_t, size_t>>> command_register_mappings_;

  std::unique_ptr<modbus_t, ModbusContextDeleter> ctx_;
  MasterParams params_;

  std::thread poll_thread_;
  std::atomic<bool> poll_running_{false};
  std::atomic<bool> init_registers_done_{false};

  size_t state_count_{0};
  size_t command_count_{0};
  std::vector<BatchGroup> read_batch_groups_;
  std::vector<BatchGroup> write_batch_groups_;
  const std::vector<modbus_hw_interface::ModbusDeviceConfig>* devices_{nullptr};
  std::function<std::vector<double>()> get_command_;
  std::vector<double> state_poll_buffer_;
  mutable realtime_tools::RealtimeBuffer<std::vector<double>> state_buffer_;
};

}  // namespace modbus_master

#endif  // MODBUS_MASTER__MODBUS_MASTER_HPP_
