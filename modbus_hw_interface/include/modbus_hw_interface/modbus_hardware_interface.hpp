// Copyright 2025 modbus_hw_interface contributors.
// SPDX-License-Identifier: Apache-2.0

#ifndef MODBUS_HW_INTERFACE__MODBUS_HARDWARE_INTERFACE_HPP_
#define MODBUS_HW_INTERFACE__MODBUS_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "modbus_hw_interface/modbus_config.hpp"
#include "modbus_hw_interface/modbus_types.hpp"

#include <modbus/modbus.h>

namespace modbus_hw_interface
{

class ModbusSystemInterface : public hardware_interface::SystemInterface
{
public:
  ModbusSystemInterface() = default;
  ~ModbusSystemInterface() override = default;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions() override;
  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_command_interface_descriptions() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool loadBusFromParams(const hardware_interface::HardwareInfo & info);
  modbus_t * getContext();
  void closeContext();
  void buildBatchGroups();
  void readStateBatched(modbus_t * ctx, std::vector<double> & state_vals);
  void writeCommandBatched(modbus_t * ctx, const std::vector<double> & command_vals);
  void pollThreadLoop();
  void startPollThread();
  void stopPollThread();

  ModbusBusConfig bus_config_;
  modbus_t * ctx_{nullptr};  // single connection per interface (one bus)
  std::string hardware_name_;

  // Poll in a separate thread (sequential: read all state, then write all command)
  std::thread poll_thread_;
  std::atomic<bool> poll_running_{false};
  std::atomic<bool> init_registers_done_{false};  // reset in on_activate, set after first init write
  double poll_rate_hz_{50.0};
  int thread_priority_{50};   // SCHED_FIFO 0-99 when RT kernel, else unused
  std::vector<int> cpu_affinity_cores_;  // empty = no affinity

  realtime_tools::RealtimeBuffer<std::vector<double>> state_buffer_;
  realtime_tools::RealtimeBuffer<std::vector<double>> command_buffer_;

  struct RegisterHandle
  {
    int slave_id;
    size_t device_index;
    size_t reg_index;
  };
  std::vector<std::pair<std::string, RegisterHandle>> state_handles_;
  std::vector<std::pair<std::string, RegisterHandle>> command_handles_;

  /** Precomputed read groups: one Modbus read per group, then decode into state_vals. */
  struct ReadBatchItem
  {
    size_t state_index;
    int register_count;
    RegisterDataType data_type;
    const ModbusRegisterConfig * reg;  // for single-read path
  };
  struct ReadBatchGroup
  {
    int slave_id;
    RegisterType type;
    int start_address;
    int total_count;
    bool use_batch;
    std::vector<ReadBatchItem> items;
    /** Preallocated for Coil/DiscreteInput (no alloc in poll loop). */
    std::vector<uint8_t> read_bits_buffer;
    /** Preallocated for InputRegister/HoldingRegister (no alloc in poll loop). */
    std::vector<uint16_t> read_reg_buffer;
  };
  std::vector<ReadBatchGroup> read_batch_groups_;

  /** Precomputed write groups: one Modbus write per group from command_vals. */
  struct WriteBatchItem
  {
    size_t command_index;
    int register_count;
    RegisterDataType data_type;
    const ModbusRegisterConfig * reg;
  };
  struct WriteBatchGroup
  {
    int slave_id;
    RegisterType type;
    int start_address;
    int total_count;
    bool use_batch;
    std::vector<WriteBatchItem> items;
    std::vector<uint16_t> write_reg_buffer;
    std::vector<uint8_t> write_bits_buffer;
  };
  std::vector<WriteBatchGroup> write_batch_groups_;

  /** Reused in poll loop to avoid allocations (size = state_handles_.size()). */
  std::vector<double> state_poll_buffer_;
};

}  // namespace modbus_hw_interface

#endif  // MODBUS_HW_INTERFACE__MODBUS_HARDWARE_INTERFACE_HPP_
