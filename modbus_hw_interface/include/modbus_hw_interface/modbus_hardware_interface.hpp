// Copyright 2025 modbus_hw_interface contributors.
// SPDX-License-Identifier: Apache-2.0

#ifndef MODBUS_HW_INTERFACE__MODBUS_HARDWARE_INTERFACE_HPP_
#define MODBUS_HW_INTERFACE__MODBUS_HARDWARE_INTERFACE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "modbus_master/modbus_master.hpp"
#include "modbus_slave_interface/modbus_device_config.hpp"
#include "modbus_slave_interface/modbus_slave_interface.hpp"
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
  /** Load Modbus device configs from components (joints/gpios/sensors); append to devices_. */
  bool loadDevicesFromComponents(const std::vector<hardware_interface::ComponentInfo>& components,
                                 const std::string& component_type);
  /** Ensure Modbus connection is open; returns true if connected. */
  bool ensureConnected();
  void closeContext();
  /** Call setInterfaces() on each plugin so devices own their interface names/indices. */
  void assignDeviceInterfaces();
  /** Pass devices, plugins and register mappings to the master for polling. */
  void setupMasterForPolling();
  /** Start master poll loop (no-op if already running). */
  void startMasterPollLoop();

  std::string bus_name_;
  std::vector<ModbusDeviceConfig> devices_;
  std::unique_ptr<modbus_master::ModbusMaster> master_;
  std::string hardware_name_;

  realtime_tools::RealtimeBuffer<std::vector<double>> command_buffer_;

  struct RegisterHandle {
    uint8_t slave_id;
    uint8_t device_index;
    uint16_t reg_index;
  };
  std::vector<std::pair<std::string, RegisterHandle>> state_handles_;
  std::vector<std::pair<std::string, RegisterHandle>> command_handles_;

  /** Pre-allocated for write() RT path (size = command_handles_.size()). */
  std::vector<double> cmd_vals_;

  std::unique_ptr<pluginlib::ClassLoader<ModbusSlaveInterface>> modbus_slave_loader_;
  /** Loaded plugin instances (one per device); each owns its interfaces via setInterfaces(). */
  std::vector<std::shared_ptr<ModbusSlaveInterface>> modbus_slaves_;
};

}  // namespace modbus_hw_interface

#endif  // MODBUS_HW_INTERFACE__MODBUS_HARDWARE_INTERFACE_HPP_
