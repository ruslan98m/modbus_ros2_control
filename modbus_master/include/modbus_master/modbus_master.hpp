// Copyright 2025 modbus_master contributors.
// SPDX-License-Identifier: Apache-2.0

#ifndef MODBUS_MASTER__MODBUS_MASTER_HPP_
#define MODBUS_MASTER__MODBUS_MASTER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

#include "modbus_slave_plugins/batch_group.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "modbus_master/connection_params.hpp"
#include "modbus_master/master_params.hpp"
#include "modbus_slave_plugins/modbus_device_config.hpp"
#include <modbus/modbus.h>

namespace modbus_master {

using BatchGroup = modbus_slave_plugins::BatchGroup;

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

  /** Parse connection and poll params from hardware_parameters map. Call before connect/startPollLoop. */
  bool initFromParams(const std::unordered_map<std::string, std::string>& params);

  /** Connect using params from initFromParams. Returns true on success. */
  bool connect();

  /** Connect via TCP. Returns true on success. */
  bool connect(const TcpConnectionParams& params);

  /** Connect via RTU (serial). Returns true on success. */
  bool connect(const RtuConnectionParams& params);

  /** Disconnect and free context. */
  void disconnect();

  bool isConnected() const { return ctx_ != nullptr; }

  /** Reset so that next startPollLoop run will call writeInitRegisters once. */
  void resetInitRegisters();

  /**
   * Start poll thread. Uses read_groups, write_groups, devices (must outlive the loop).
   * State is written to internal RT buffer; get_command() provides command_vals for write.
   * Call readStateSnapshotForRT() from RT context to read the latest state.
   */
  void startPollLoop(
      size_t state_count,
      size_t command_count,
      std::vector<BatchGroup>& read_groups,
      std::vector<BatchGroup>& write_groups,
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
  bool isPollLoopRunning() const { return poll_thread_.joinable(); }

  /**
   * Read state registers/coils into state_vals using batch groups.
   * devices[] is used for response_timeout_sec per device.
   */
  void readStateBatched(std::vector<BatchGroup>& read_groups,
                        const std::vector<modbus_hw_interface::ModbusDeviceConfig>& devices,
                        std::vector<double>& state_vals);

  /** Write command registers/coils from command_vals using batch groups. */
  void writeCommandBatched(std::vector<BatchGroup>& write_groups,
                           const std::vector<modbus_hw_interface::ModbusDeviceConfig>& devices,
                           const std::vector<double>& command_vals);

  /** Write init_registers for each device once at startup. */
  void writeInitRegisters(const std::vector<modbus_hw_interface::ModbusDeviceConfig>& devices);

 private:
  void pollThreadLoop();

  std::unique_ptr<modbus_t, ModbusContextDeleter> ctx_;
  MasterParams params_;

  std::thread poll_thread_;
  std::atomic<bool> poll_running_{false};
  std::atomic<bool> init_registers_done_{false};

  size_t state_count_{0};
  size_t command_count_{0};
  std::vector<BatchGroup>* read_groups_{nullptr};
  std::vector<BatchGroup>* write_groups_{nullptr};
  const std::vector<modbus_hw_interface::ModbusDeviceConfig>* devices_{nullptr};
  std::function<std::vector<double>()> get_command_;
  std::vector<double> state_poll_buffer_;
  mutable realtime_tools::RealtimeBuffer<std::vector<double>> state_buffer_;
};

}  // namespace modbus_master

#endif  // MODBUS_MASTER__MODBUS_MASTER_HPP_
