// Copyright 2025 modbus_ros2_control contributors.
// SPDX-License-Identifier: Apache-2.0

#ifndef MODBUS_ROS2_CONTROL__MODBUS_HARDWARE_INTERFACE_HPP_
#define MODBUS_ROS2_CONTROL__MODBUS_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <chrono>
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

#include <modbus/modbus.h>
#include <yaml-cpp/yaml.h>

namespace modbus_ros2_control
{

enum class RegisterType
{
  Coil,             // 0x, read/write
  DiscreteInput,   // 1x, read-only
  InputRegister,    // 3x, read-only
  HoldingRegister  // 4x, read/write
};

enum class RegisterDataType
{
  Bool,
  Int16,
  Uint16,
  Int32,
  Uint32,
  Float32,
  Int64,
  Uint64,
  Float64
};

struct ModbusRegisterConfig
{
  std::string name;
  RegisterType type{RegisterType::HoldingRegister};
  int address{0};
  bool is_command{false};  // true = command interface, false = state interface
  RegisterDataType data_type{RegisterDataType::Uint16};
  int register_count{1};  // 1 for 16-bit, 2 for 32-bit/float32, 4 for 64-bit
};

/** Written once at startup (on_activate) before polling. Only coil and holding_register are writable. */
struct ModbusInitRegisterConfig
{
  RegisterType type{RegisterType::HoldingRegister};
  int address{0};
  RegisterDataType data_type{RegisterDataType::Uint16};
  int register_count{1};
  double value{0};
};

struct ModbusDeviceConfig
{
  std::string name;   // device/joint name from xacro (for interface naming)
  int slave_id{1};
  std::vector<ModbusInitRegisterConfig> init_registers;  // written once at startup
  std::vector<ModbusRegisterConfig> registers;
};

// One bus = one connection (either RTU or TCP, not both)
struct ModbusBusConfig
{
  std::string bus_name;  // for interface naming
  bool is_tcp{true};
  // TCP (when is_tcp == true)
  std::string ip_address{"127.0.0.1"};
  int port{502};
  // RTU (when is_tcp == false)
  std::string serial_port{"/dev/ttyUSB0"};
  int baud_rate{9600};
  char parity{'N'};  // N, E, O
  int data_bits{8};
  int stop_bits{1};
  std::vector<ModbusDeviceConfig> devices;
};

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
  bool loadDeviceConfigFromFile(const std::string & path, ModbusDeviceConfig & out);
  modbus_t * getContext();
  void closeContext();
  double readRegisterValue(modbus_t * ctx, const ModbusRegisterConfig & reg);
  bool writeRegisterValue(modbus_t * ctx, const ModbusRegisterConfig & reg, double value);
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
  bool lock_memory_{false};
  bool preallocate_stack_{false};
  realtime_tools::RealtimeBuffer<std::vector<double>> state_buffer_;
  realtime_tools::RealtimeBuffer<std::vector<double>> command_buffer_;

  struct StateRegisterHandle
  {
    int slave_id;
    size_t device_index;
    size_t reg_index;
  };
  struct CommandRegisterHandle
  {
    int slave_id;
    size_t device_index;
    size_t reg_index;
  };
  std::vector<std::pair<std::string, StateRegisterHandle>> state_handles_;
  std::vector<std::pair<std::string, CommandRegisterHandle>> command_handles_;
};

}  // namespace modbus_ros2_control

#endif  // MODBUS_ROS2_CONTROL__MODBUS_HARDWARE_INTERFACE_HPP_
