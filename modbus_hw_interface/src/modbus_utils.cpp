// Copyright 2025 modbus_ros2_control contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_utils.cpp
 * @brief String/type conversion helpers and realtime thread helpers for Modbus.
 */

#include "modbus_hw_interface/modbus_utils.hpp"

#include <sys/resource.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_helpers.hpp"

namespace modbus_hw_interface {

namespace {

/** Data type: full and short names (i8/u8 map to 16-bit; Modbus registers are 16-bit) */
static const std::unordered_map<std::string, RegisterDataType> STR_TO_DATA_TYPE = {
    {"bool", RegisterDataType::Bool},       {"b", RegisterDataType::Bool},
    {"int16", RegisterDataType::Int16},     {"i16", RegisterDataType::Int16},
    {"i8", RegisterDataType::Int16},        {"uint16", RegisterDataType::Uint16},
    {"u16", RegisterDataType::Uint16},      {"u8", RegisterDataType::Uint16},
    {"int32", RegisterDataType::Int32},     {"i32", RegisterDataType::Int32},
    {"uint32", RegisterDataType::Uint32},   {"u32", RegisterDataType::Uint32},
    {"float32", RegisterDataType::Float32}, {"f32", RegisterDataType::Float32},
    {"int64", RegisterDataType::Int64},     {"i64", RegisterDataType::Int64},
    {"uint64", RegisterDataType::Uint64},   {"u64", RegisterDataType::Uint64},
    {"float64", RegisterDataType::Float64}, {"f64", RegisterDataType::Float64},
};

/** Register type: full and short names */
static const std::unordered_map<std::string, RegisterType> STR_TO_REG_TYPE = {
    {"coil", RegisterType::Coil},
    {"c", RegisterType::Coil},
    {"discrete_input", RegisterType::DiscreteInput},
    {"di", RegisterType::DiscreteInput},
    {"input_register", RegisterType::InputRegister},
    {"ir", RegisterType::InputRegister},
    {"holding_register", RegisterType::HoldingRegister},
    {"hr", RegisterType::HoldingRegister},
};

/** Canonical string per data type (for interface export) */
static const std::unordered_map<RegisterDataType, std::string, RegisterDataTypeHash>
    DATA_TYPE_TO_STR = {
        {RegisterDataType::Bool, "bool"},       {RegisterDataType::Int16, "int16"},
        {RegisterDataType::Uint16, "uint16"},   {RegisterDataType::Int32, "int32"},
        {RegisterDataType::Uint32, "uint32"},   {RegisterDataType::Float32, "float32"},
        {RegisterDataType::Int64, "int64"},     {RegisterDataType::Uint64, "uint64"},
        {RegisterDataType::Float64, "float64"},
};

/** Number of 16-bit registers per data type */
static const std::unordered_map<RegisterDataType, int, RegisterDataTypeHash>
    DATA_TYPE_TO_REG_COUNT = {
        {RegisterDataType::Bool, 1},  {RegisterDataType::Int16, 1},  {RegisterDataType::Uint16, 1},
        {RegisterDataType::Int32, 2}, {RegisterDataType::Uint32, 2}, {RegisterDataType::Float32, 2},
        {RegisterDataType::Int64, 4}, {RegisterDataType::Uint64, 4}, {RegisterDataType::Float64, 4},
};

/** @return Lowercase copy of @a s (for case-insensitive lookup) */
std::string toLower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

}  // namespace

RegisterDataType dataTypeFromString(const std::string& s) {
  auto it = STR_TO_DATA_TYPE.find(toLower(s));
  return (it != STR_TO_DATA_TYPE.end()) ? it->second : RegisterDataType::Uint16;
}

RegisterType registerTypeFromString(const std::string& s) {
  auto it = STR_TO_REG_TYPE.find(toLower(s));
  return (it != STR_TO_REG_TYPE.end()) ? it->second : RegisterType::HoldingRegister;
}

std::string dataTypeToInterfaceString(RegisterDataType t) {
  auto it = DATA_TYPE_TO_STR.find(t);
  return (it != DATA_TYPE_TO_STR.end()) ? it->second : "double";
}

int registerCountForDataType(RegisterDataType t) {
  auto it = DATA_TYPE_TO_REG_COUNT.find(t);
  return (it != DATA_TYPE_TO_REG_COUNT.end()) ? it->second : 1;
}

void applyRealtimeThreadParams(rclcpp::Logger logger, int thread_priority,
                               const std::vector<int>& cpu_affinity_cores) {
  const bool has_realtime = realtime_tools::has_realtime_kernel();
  try {
    if (has_realtime) {
      const auto lock_result = realtime_tools::lock_memory();
      if (!lock_result.first) {
        RCLCPP_WARN(logger, "Unable to lock poll thread memory: '%s'", lock_result.second.c_str());
      } else {
        RCLCPP_INFO(logger, "Poll thread: successfully locked memory");
      }
      if (!realtime_tools::configure_sched_fifo(thread_priority)) {
        RCLCPP_ERROR(
            logger,
            "Could not enable FIFO RT scheduling for poll thread: error <%d>(%s). "
            "See https://control.ros.org/doc/ros2_control/controller_manager/doc/userdoc.html for "
            "details.",
            errno, strerror(errno));
      } else {
        RCLCPP_INFO(logger, "Poll thread set to FIFO RT scheduling with priority %d",
                    thread_priority);
      }
    } else {
      constexpr int NON_RT_NICE = -20;
      RCLCPP_WARN(logger,
                  "No real-time kernel detected. Setting maximum nice priority for poll thread.");
      if (setpriority(PRIO_PROCESS, 0, NON_RT_NICE) != 0) {
        RCLCPP_WARN(logger, "Unable to set nice priority: '%s'. Continuing with default.",
                    strerror(errno));
      } else {
        RCLCPP_INFO(logger, "Poll thread set to nice %d (non-RT)", -NON_RT_NICE);
      }
    }

    if (!cpu_affinity_cores.empty()) {
      const auto affinity_result = realtime_tools::set_current_thread_affinity(cpu_affinity_cores);
      if (!affinity_result.first) {
        RCLCPP_WARN(logger, "Unable to set poll thread CPU affinity: '%s'",
                    affinity_result.second.c_str());
      } else {
        RCLCPP_INFO(logger, "Poll thread CPU affinity set (%zu core(s))",
                    cpu_affinity_cores.size());
      }
    }

    if (has_realtime) {
      constexpr size_t MAX_SAFE_STACK = 8 * 1024;  // 8 KiB
      uint8_t dummy[MAX_SAFE_STACK];
      std::memset(dummy, 0, sizeof(dummy));
      (void)dummy;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Exception applying realtime thread params: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(logger, "Unknown exception applying realtime thread params");
  }
}

}  // namespace modbus_hw_interface
