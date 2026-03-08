// Copyright 2025 modbus_hw_interface contributors.
// SPDX-License-Identifier: Apache-2.0

#ifndef MODBUS_HW_INTERFACE__MODBUS_HARDWARE_INTERFACE_HPP_
#define MODBUS_HW_INTERFACE__MODBUS_HARDWARE_INTERFACE_HPP_

#include <modbus/modbus.h>

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
#include "modbus_hw_interface/modbus_config.hpp"
#include "modbus_hw_interface/modbus_types.hpp"
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
  modbus_t* getContext();
  void closeContext();
  void buildBatchGroups();
  void readStateBatched(modbus_t* ctx, std::vector<double>& state_vals);
  void writeCommandBatched(modbus_t* ctx, const std::vector<double>& command_vals);
  /** Set ctx response timeout from device config; no-op if device has response_timeout_sec <= 0. */
  void setContextResponseTimeout(modbus_t* ctx, size_t device_index);
  void pollThreadLoop();
  void startPollThread();
  void stopPollThread();

  ModbusBusConfig bus_config_;
  modbus_t* ctx_{nullptr};  // single connection per interface (one bus)
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
    int slave_id;
    size_t device_index;
    size_t reg_index;
  };
  std::vector<std::pair<std::string, RegisterHandle>> state_handles_;
  std::vector<std::pair<std::string, RegisterHandle>> command_handles_;

  struct BatchItem {
    int register_count;
    RegisterDataType data_type;
    const ModbusRegisterConfig* reg;
    size_t index;
  };

  struct BatchGroup {
    size_t device_index;
    int slave_id;
    RegisterType type;
    int start_address;
    int total_count;
    bool use_batch;
    std::vector<BatchItem> items;
    std::vector<uint8_t> bits_buffer;  // for coils/discrete inputs
    std::vector<uint16_t> reg_buffer;  // for holding/input registers
  };

  std::vector<BatchGroup> read_batch_groups_;
  std::vector<BatchGroup> write_batch_groups_;

  /** Reused in poll loop to avoid allocations (size = state_handles_.size()). */
  std::vector<double> state_poll_buffer_;
};

}  // namespace modbus_hw_interface

#endif  // MODBUS_HW_INTERFACE__MODBUS_HARDWARE_INTERFACE_HPP_
