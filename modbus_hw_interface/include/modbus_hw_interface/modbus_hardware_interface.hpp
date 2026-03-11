// Copyright 2025 modbus_hw_interface contributors.
// SPDX-License-Identifier: Apache-2.0

#ifndef MODBUS_HW_INTERFACE__MODBUS_HARDWARE_INTERFACE_HPP_
#define MODBUS_HW_INTERFACE__MODBUS_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <unordered_map>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "modbus_hw_interface/modbus_config.hpp"
#include "modbus_master/batch_group.hpp"
#include "modbus_master/modbus_master.hpp"
#include "modbus_slave_plugins/modbus_slave_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace modbus_hw_interface {

class ModbusSystemInterface : public hardware_interface::SystemInterface {
 public:
  ModbusSystemInterface() = default;
  ~ModbusSystemInterface() override = default;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions() override;
  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_command_interface_descriptions() override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

 private:
  bool loadBusFromParams(const hardware_interface::HardwareInfo& info);
  /** Load Modbus device configs from components (joints/gpios/sensors); append to
   * bus_config_.devices. */
  bool loadDevicesFromComponents(const std::vector<hardware_interface::ComponentInfo>& components,
                                 const std::string& component_type);
  /** Ensure Modbus connection is open; returns true if connected. */
  bool ensureConnected();
  void closeContext();
  void buildBatchGroups();
  /** Pre-allocate state/command name and value buffers for RT-safe read()/write(). */
  void buildRtBuffers();
  void pollThreadLoop();
  void startPollThread();
  void stopPollThread();

  ModbusBusConfig bus_config_;
  std::unique_ptr<modbus_master::ModbusMaster> master_;
  std::string hardware_name_;

  // Poll in a separate thread (sequential: read all state, then write all command)
  std::thread poll_thread_;
  std::atomic<bool> poll_running_{false};
  std::atomic<bool> init_registers_done_{
      false};                            // reset in on_activate, set after first init write
  double poll_rate_hz_{0.0};             // 0 = no delay in poll loop (run as fast as possible)
  int thread_priority_{50};              // SCHED_FIFO 0-99 when RT kernel, else unused
  std::vector<int> cpu_affinity_cores_;  // empty = no affinity

  realtime_tools::RealtimeBuffer<std::vector<double>> state_buffer_;
  realtime_tools::RealtimeBuffer<std::vector<double>> command_buffer_;

  struct RegisterHandle {
    uint8_t slave_id;
    uint8_t device_index;
    uint16_t reg_index;
  };
  std::vector<std::pair<std::string, RegisterHandle>> state_handles_;
  std::vector<std::pair<std::string, RegisterHandle>> command_handles_;

  std::vector<modbus_master::BatchGroup> read_batch_groups_;
  std::vector<modbus_master::BatchGroup> write_batch_groups_;

  /** Reused in poll loop to avoid allocations (size = state_handles_.size()). */
  std::vector<double> state_poll_buffer_;

  /** Pre-allocated for read() RT path: state interface names and values per device. */
  std::vector<std::vector<std::string>> state_names_per_device_;
  std::vector<std::vector<double>> state_vals_per_device_;
  /** state_handles_[i] -> (device_index, local_index) for one-pass scatter in read(). */
  std::vector<std::pair<size_t, size_t>> state_to_device_local_;

  /** Pre-allocated for write() RT path. */
  std::vector<double> cmd_vals_;
  std::vector<std::vector<std::string>> command_names_per_device_;
  std::vector<std::vector<double>> command_out_per_device_;
  /** command_global_index_[d][j] = global index into cmd_vals_ for device d, local j. */
  std::vector<std::vector<size_t>> command_global_index_;

  /** Loader for Modbus slave plugins (e.g. modbus_slave_plugins/GenericModbusSlave). */
  std::unique_ptr<pluginlib::ClassLoader<ModbusSlaveInterface>> modbus_slave_loader_;

  /** Loaded plugin instances (one per device), like ec_modules_ in EthercatDriver. */
  std::vector<std::shared_ptr<ModbusSlaveInterface>> modbus_slaves_;
};

}  // namespace modbus_hw_interface

#endif  // MODBUS_HW_INTERFACE__MODBUS_HARDWARE_INTERFACE_HPP_
