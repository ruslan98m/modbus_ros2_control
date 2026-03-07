// Copyright 2025 modbus_hw_interface contributors.
// SPDX-License-Identifier: Apache-2.0

#include "modbus_hw_interface/modbus_config_loader.hpp"
#include "modbus_hw_interface/modbus_hardware_interface.hpp"
#include "modbus_hw_interface/modbus_io.hpp"
#include "modbus_hw_interface/modbus_utils.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <sys/resource.h>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_helpers.hpp"

namespace modbus_hw_interface
{

bool ModbusSystemInterface::loadBusFromParams(const hardware_interface::HardwareInfo & info)
{
  const auto & p = info.hardware_parameters;
  auto get = [&p](const std::string & key, const std::string & def) {
    auto it = p.find(key);
    return (it != p.end()) ? it->second : def;
  };
  std::string type_str = get("connection_type", "tcp");
  if (type_str == "tcp") {
    bus_config_.is_tcp = true;
    bus_config_.ip_address = get("ip_address", "127.0.0.1");
    try {
      bus_config_.port = std::stoi(get("port", "502"));
    } catch (...) {
      bus_config_.port = 502;
    }
  } else if (type_str == "rtu") {
    bus_config_.is_tcp = false;
    bus_config_.serial_port = get("serial_port", "/dev/ttyUSB0");
    try {
      bus_config_.baud_rate = std::stoi(get("baud_rate", "9600"));
    } catch (...) {
      bus_config_.baud_rate = 9600;
    }
    std::string par = get("parity", "N");
    bus_config_.parity = par.empty() ? 'N' : par[0];
    try {
      bus_config_.data_bits = std::stoi(get("data_bits", "8"));
      bus_config_.stop_bits = std::stoi(get("stop_bits", "1"));
    } catch (...) {
      bus_config_.data_bits = 8;
      bus_config_.stop_bits = 1;
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"),
      "connection_type must be 'tcp' or 'rtu', got '%s'", type_str.c_str());
    return false;
  }
  bus_config_.bus_name = get("bus_name", bus_config_.is_tcp ? "tcp" : "rtu");

  try {
    poll_rate_hz_ = std::stod(get("poll_rate_hz", "50"));
  } catch (...) {
    poll_rate_hz_ = 50.0;
  }
  try {
    thread_priority_ = std::stoi(get("thread_priority", "50"));
  } catch (...) {
    thread_priority_ = 50;
  }
  std::string affinity_str = get("cpu_affinity", "");
  cpu_affinity_cores_.clear();
  if (!affinity_str.empty()) {
    std::istringstream ss(affinity_str);
    std::string part;
    while (std::getline(ss, part, ',')) {
      try {
        size_t pos;
        int c = std::stoi(part, &pos);
        if (c >= 0) cpu_affinity_cores_.push_back(c);
      } catch (...) {}
    }
  }

  return true;
}

modbus_t * ModbusSystemInterface::getContext()
{
  if (ctx_) return ctx_;
  const auto & c = bus_config_;
  if (c.is_tcp) {
    ctx_ = modbus_new_tcp(c.ip_address.c_str(), c.port);
  } else {
    ctx_ = modbus_new_rtu(
      c.serial_port.c_str(), c.baud_rate, c.parity,
      c.data_bits, c.stop_bits);
  }
  if (!ctx_) {
    RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"),
      "Failed to create Modbus context for bus '%s'", c.bus_name.c_str());
    return nullptr;
  }
  if (modbus_connect(ctx_) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"),
      "Failed to connect Modbus bus '%s': %s", c.bus_name.c_str(), modbus_strerror(errno));
    modbus_free(ctx_);
    ctx_ = nullptr;
    return nullptr;
  }
  return ctx_;
}

void ModbusSystemInterface::closeContext()
{
  if (ctx_) {
    modbus_close(ctx_);
    modbus_free(ctx_);
    ctx_ = nullptr;
  }
}

void ModbusSystemInterface::buildBatchGroups()
{
  read_batch_groups_.clear();
  write_batch_groups_.clear();

  auto build_read = [this]() {
    std::map<std::pair<int, size_t>, std::vector<size_t>> by_device;
    for (size_t i = 0; i < state_handles_.size(); ++i) {
      const auto & h = state_handles_[i].second;
      by_device[{h.slave_id, h.device_index}].push_back(i);
    }
    for (const auto & [key, indices] : by_device) {
      const int slave_id = key.first;
      const size_t dev_idx = key.second;
      const auto & dev = bus_config_.devices[dev_idx];
      std::vector<std::pair<size_t, const ModbusRegisterConfig *>> items;
      for (size_t idx : indices) {
        const auto & h = state_handles_[idx].second;
        items.push_back({idx, &dev.registers[h.reg_index]});
      }
      std::sort(items.begin(), items.end(), [](const auto & a, const auto & b) {
        if (a.second->type != b.second->type) {
          return static_cast<int>(a.second->type) < static_cast<int>(b.second->type);
        }
        return a.second->address < b.second->address;
      });
      for (size_t i = 0; i < items.size(); ) {
        const RegisterType cur_type = items[i].second->type;
        int cur_addr = items[i].second->address;
        ReadBatchGroup grp;
        grp.slave_id = slave_id;
        grp.type = cur_type;
        grp.start_address = cur_addr;
        grp.total_count = 0;
        grp.use_batch = (cur_type == RegisterType::Coil || cur_type == RegisterType::DiscreteInput)
          ? dev.read_multiple_coils
          : dev.read_multiple_registers;
        while (i < items.size() && items[i].second->type == cur_type && items[i].second->address == cur_addr) {
          const auto * reg = items[i].second;
          grp.items.push_back({items[i].first, reg->register_count, reg->data_type, reg});
          grp.total_count += reg->register_count;
          cur_addr += reg->register_count;
          ++i;
        }
        if (grp.use_batch && grp.total_count > 0) {
          if (cur_type == RegisterType::Coil || cur_type == RegisterType::DiscreteInput) {
            grp.read_bits_buffer.resize(static_cast<size_t>(grp.total_count));
          } else {
            grp.read_reg_buffer.resize(static_cast<size_t>(grp.total_count));
          }
        }
        read_batch_groups_.push_back(std::move(grp));
      }
    }
  };

  auto build_write = [this]() {
    std::map<std::pair<int, size_t>, std::vector<size_t>> by_device;
    for (size_t i = 0; i < command_handles_.size(); ++i) {
      const auto & h = command_handles_[i].second;
      by_device[{h.slave_id, h.device_index}].push_back(i);
    }
    for (const auto & [key, indices] : by_device) {
      const int slave_id = key.first;
      const size_t dev_idx = key.second;
      const auto & dev = bus_config_.devices[dev_idx];
      std::vector<std::pair<size_t, const ModbusRegisterConfig *>> items;
      for (size_t idx : indices) {
        const auto & h = command_handles_[idx].second;
        items.push_back({idx, &dev.registers[h.reg_index]});
      }
      std::sort(items.begin(), items.end(), [](const auto & a, const auto & b) {
        if (a.second->type != b.second->type) {
          return static_cast<int>(a.second->type) < static_cast<int>(b.second->type);
        }
        return a.second->address < b.second->address;
      });
      for (size_t i = 0; i < items.size(); ) {
        const RegisterType cur_type = items[i].second->type;
        int cur_addr = items[i].second->address;
        if (cur_type == RegisterType::DiscreteInput || cur_type == RegisterType::InputRegister) {
          ++i;
          continue;
        }
        WriteBatchGroup grp;
        grp.slave_id = slave_id;
        grp.type = cur_type;
        grp.start_address = cur_addr;
        grp.total_count = 0;
        grp.use_batch = (cur_type == RegisterType::Coil)
          ? dev.write_multiple_coils
          : dev.write_multiple_registers;
        while (i < items.size() && items[i].second->type == cur_type && items[i].second->address == cur_addr) {
          const auto * reg = items[i].second;
          grp.items.push_back({items[i].first, reg->register_count, reg->data_type, reg});
          grp.total_count += reg->register_count;
          cur_addr += reg->register_count;
          ++i;
        }
        if (grp.use_batch && grp.total_count > 0) {
          if (cur_type == RegisterType::HoldingRegister) {
            grp.write_reg_buffer.resize(static_cast<size_t>(grp.total_count));
          } else if (cur_type == RegisterType::Coil) {
            grp.write_bits_buffer.resize(grp.items.size());
          }
        }
        write_batch_groups_.push_back(std::move(grp));
      }
    }
  };

  build_read();
  build_write();

  state_poll_buffer_.resize(state_handles_.size(), 0.0);
}

void ModbusSystemInterface::readStateBatched(modbus_t * ctx, std::vector<double> & state_vals)
{
  if (state_vals.size() != state_handles_.size()) return;
  std::fill(state_vals.begin(), state_vals.end(), 0.0);
  for (auto & grp : read_batch_groups_) {
    if (modbus_set_slave(ctx, grp.slave_id) < 0) continue;
    if (grp.use_batch && grp.total_count > 0) {
      if (grp.type == RegisterType::Coil && !grp.read_bits_buffer.empty()) {
        if (modbus_read_bits(ctx, grp.start_address, grp.total_count, grp.read_bits_buffer.data()) == grp.total_count) {
          size_t off = 0;
          for (const auto & it : grp.items) {
            state_vals[it.state_index] = grp.read_bits_buffer[off] ? 1.0 : 0.0;
            off += static_cast<size_t>(it.register_count);
          }
        }
      } else if (grp.type == RegisterType::DiscreteInput && !grp.read_bits_buffer.empty()) {
        if (modbus_read_input_bits(ctx, grp.start_address, grp.total_count, grp.read_bits_buffer.data()) == grp.total_count) {
          size_t off = 0;
          for (const auto & it : grp.items) {
            state_vals[it.state_index] = grp.read_bits_buffer[off] ? 1.0 : 0.0;
            off += static_cast<size_t>(it.register_count);
          }
        }
      } else if (!grp.read_reg_buffer.empty()) {
        int ret = (grp.type == RegisterType::InputRegister)
          ? modbus_read_input_registers(ctx, grp.start_address, grp.total_count, grp.read_reg_buffer.data())
          : modbus_read_registers(ctx, grp.start_address, grp.total_count, grp.read_reg_buffer.data());
        if (ret == grp.total_count) {
          size_t off = 0;
          for (const auto & it : grp.items) {
            state_vals[it.state_index] = decodeRegistersFromBuffer(
              grp.read_reg_buffer.data(), off, it.register_count, it.data_type);
            off += static_cast<size_t>(it.register_count);
          }
        }
      }
    } else {
      for (const auto & it : grp.items) {
        state_vals[it.state_index] = readRegisterValue(ctx, *it.reg);
      }
    }
  }
}

void ModbusSystemInterface::writeCommandBatched(
  modbus_t * ctx, const std::vector<double> & command_vals)
{
  if (command_vals.size() != command_handles_.size()) return;
  for (auto & grp : write_batch_groups_) {
    if (modbus_set_slave(ctx, grp.slave_id) < 0) continue;
    if (grp.use_batch && grp.total_count > 0 && grp.type == RegisterType::HoldingRegister && !grp.write_reg_buffer.empty()) {
      size_t off = 0;
      for (const auto & it : grp.items) {
        const double val = command_vals[it.command_index];
        const ModbusRegisterConfig * reg = it.reg;
        if (reg->register_count == 1) {
          grp.write_reg_buffer[off] = (reg->data_type == RegisterDataType::Int16)
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
          grp.write_reg_buffer[off] = static_cast<uint16_t>(u32 >> 16);
          grp.write_reg_buffer[off + 1] = static_cast<uint16_t>(u32 & 0xFFFF);
        } else if (reg->register_count == 4) {
          uint64_t u64;
          if (reg->data_type == RegisterDataType::Float64) {
            memcpy(&u64, &val, 8);
          } else if (reg->data_type == RegisterDataType::Int64) {
            u64 = static_cast<uint64_t>(static_cast<int64_t>(val));
          } else {
            u64 = static_cast<uint64_t>(val);
          }
          grp.write_reg_buffer[off] = static_cast<uint16_t>(u64 >> 48);
          grp.write_reg_buffer[off + 1] = static_cast<uint16_t>((u64 >> 32) & 0xFFFF);
          grp.write_reg_buffer[off + 2] = static_cast<uint16_t>((u64 >> 16) & 0xFFFF);
          grp.write_reg_buffer[off + 3] = static_cast<uint16_t>(u64 & 0xFFFF);
        }
        off += static_cast<size_t>(reg->register_count);
      }
      modbus_write_registers(ctx, grp.start_address, grp.total_count, grp.write_reg_buffer.data());
    } else if (grp.use_batch && grp.type == RegisterType::Coil && !grp.write_bits_buffer.empty()) {
      for (size_t g = 0; g < grp.write_bits_buffer.size(); ++g) {
        grp.write_bits_buffer[g] = (std::fabs(command_vals[grp.items[g].command_index]) > 0.5) ? 1 : 0;
      }
      modbus_write_bits(ctx, grp.start_address, static_cast<int>(grp.write_bits_buffer.size()), grp.write_bits_buffer.data());
    } else {
      for (const auto & it : grp.items) {
        writeRegisterValue(ctx, *it.reg, command_vals[it.command_index]);
      }
    }
  }
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  const auto & info = params.hardware_info;
  hardware_name_ = info.name;
  bus_config_.devices.clear();

  if (!loadBusFromParams(info)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info.joints) {
    auto it_cfg = joint.parameters.find("device_config");
    if (it_cfg == joint.parameters.end()) {
      it_cfg = joint.parameters.find("slave_config");
    }
    if (it_cfg == joint.parameters.end() || it_cfg->second.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"),
        "Joint '%s': missing param 'device_config' or 'slave_config' (path to device YAML)", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    int slave_id = 1;
    auto it_slave = joint.parameters.find("slave_id");
    if (it_slave != joint.parameters.end()) {
      try {
        slave_id = std::stoi(it_slave->second);
      } catch (...) {}
    }
    ModbusDeviceConfig dev;
    dev.name = joint.name;
    dev.slave_id = slave_id;
    if (!loadDeviceConfigFromFile(it_cfg->second, dev)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    bus_config_.devices.push_back(dev);
  }

  state_handles_.clear();
  command_handles_.clear();
  poll_running_.store(false);

  for (size_t di = 0; di < bus_config_.devices.size(); ++di) {
    const auto & dev = bus_config_.devices[di];
    std::string dev_prefix = bus_config_.bus_name + "_" + dev.name;
    for (size_t ri = 0; ri < dev.registers.size(); ++ri) {
      const auto & reg = dev.registers[ri];
      std::string if_name = dev_prefix + "_" + reg.name;
      std::string full_name = hardware_name_ + "/" + if_name;
      if (reg.is_command) {
        command_handles_.push_back({full_name, {dev.slave_id, di, ri}});
      } else {
        state_handles_.push_back({full_name, {dev.slave_id, di, ri}});
      }
    }
  }

  state_buffer_.initRT(std::vector<double>(state_handles_.size(), 0.0));
  command_buffer_.initRT(std::vector<double>(command_handles_.size(), 0.0));

  buildBatchGroups();

  return hardware_interface::CallbackReturn::SUCCESS;
}

void ModbusSystemInterface::pollThreadLoop()
{
  const auto logger = rclcpp::get_logger("ModbusSystemInterface");
  const bool has_realtime = realtime_tools::has_realtime_kernel();
  try {
    if (has_realtime) {
      const auto lock_result = realtime_tools::lock_memory();
      if (!lock_result.first) {
        RCLCPP_WARN(logger, "Unable to lock poll thread memory: '%s'", lock_result.second.c_str());
      } else {
        RCLCPP_INFO(logger, "Poll thread: successfully locked memory");
      }
      if (!realtime_tools::configure_sched_fifo(thread_priority_)) {
        RCLCPP_ERROR(
          logger,
          "Could not enable FIFO RT scheduling for poll thread: error <%d>(%s). "
          "See https://control.ros.org/doc/ros2_control/controller_manager/doc/userdoc.html for details.",
          errno, strerror(errno));
      } else {
        RCLCPP_INFO(logger, "Poll thread set to FIFO RT scheduling with priority %d", thread_priority_);
      }
    } else {
      constexpr int NON_RT_NICE = -20;
      RCLCPP_WARN(logger, "No real-time kernel detected. Setting maximum nice priority for poll thread.");
      if (setpriority(PRIO_PROCESS, 0, NON_RT_NICE) != 0) {
        RCLCPP_WARN(logger, "Unable to set nice priority: '%s'. Continuing with default.", strerror(errno));
      } else {
        RCLCPP_INFO(logger, "Poll thread set to nice %d (non-RT)", -NON_RT_NICE);
      }
    }

    if (!cpu_affinity_cores_.empty()) {
      const auto affinity_result = realtime_tools::set_current_thread_affinity(cpu_affinity_cores_);
      if (!affinity_result.first) {
        RCLCPP_WARN(logger, "Unable to set poll thread CPU affinity: '%s'", affinity_result.second.c_str());
      } else {
        RCLCPP_INFO(logger, "Poll thread CPU affinity set (%zu core(s))", cpu_affinity_cores_.size());
      }
    }

    if (has_realtime) {
      // Preallocate stack to avoid stack overflow
      constexpr size_t MAX_SAFE_STACK = 8 * 1024;  // 8 KiB
      uint8_t dummy[MAX_SAFE_STACK];
      memset(dummy, 0, sizeof(dummy));
      (void)dummy;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Exception applying realtime thread params: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(logger, "Unknown exception applying realtime thread params");
  }

  const auto period = std::chrono::duration<double>(1.0 / poll_rate_hz_);
  modbus_t * ctx = getContext();
  if (!ctx) return;

  if (!init_registers_done_.exchange(true)) {
    const auto logger = rclcpp::get_logger("ModbusSystemInterface");
    for (const auto & dev : bus_config_.devices) {
      if (dev.init_registers.empty()) continue;
      if (modbus_set_slave(ctx, dev.slave_id) < 0) {
        RCLCPP_WARN(logger, "Init registers: failed to set slave_id %d", dev.slave_id);
        continue;
      }
      for (const auto & init : dev.init_registers) {
        ModbusRegisterConfig reg;
        reg.type = init.type;
        reg.address = init.address;
        reg.data_type = init.data_type;
        reg.register_count = init.register_count;
        if (modbus_hw_interface::writeRegisterValue(ctx, reg, init.value)) {
          RCLCPP_DEBUG(logger, "Init write dev '%s' addr %d = %g", dev.name.c_str(), init.address, init.value);
        } else {
          RCLCPP_WARN(logger, "Init write failed dev '%s' type %d addr %d", dev.name.c_str(),
            static_cast<int>(init.type), init.address);
        }
      }
    }
  }

  while (poll_running_.load(std::memory_order_relaxed)) {
    readStateBatched(ctx, state_poll_buffer_);
    state_buffer_.writeFromNonRT(state_poll_buffer_);

    const std::vector<double> * ptr = command_buffer_.readFromNonRT();
    if (ptr && ptr->size() == command_handles_.size()) {
      writeCommandBatched(ctx, *ptr);
    }
    std::this_thread::sleep_for(period);
  }
}

void ModbusSystemInterface::startPollThread()
{
  if (poll_thread_.joinable()) return;
  poll_running_.store(true);
  poll_thread_ = std::thread(&ModbusSystemInterface::pollThreadLoop, this);
}

void ModbusSystemInterface::stopPollThread()
{
  poll_running_.store(false);
  if (poll_thread_.joinable()) {
    poll_thread_.join();
    poll_thread_ = std::thread();
  }
}

std::vector<hardware_interface::InterfaceDescription>
ModbusSystemInterface::export_unlisted_state_interface_descriptions()
{
  std::vector<hardware_interface::InterfaceDescription> out;
  for (const auto & [full_name, h] : state_handles_) {
    const auto & reg = bus_config_.devices[h.device_index].registers[h.reg_index];
    const auto & dev = bus_config_.devices[h.device_index];
    std::string if_name = bus_config_.bus_name + "_" + dev.name + "_" + reg.name;
    hardware_interface::InterfaceInfo inf;
    inf.name = if_name;
    inf.data_type = "double";
    out.push_back(hardware_interface::InterfaceDescription(hardware_name_, inf));
  }
  return out;
}

std::vector<hardware_interface::InterfaceDescription>
ModbusSystemInterface::export_unlisted_command_interface_descriptions()
{
  std::vector<hardware_interface::InterfaceDescription> out;
  for (const auto & [full_name, h] : command_handles_) {
    const auto & reg = bus_config_.devices[h.device_index].registers[h.reg_index];
    const auto & dev = bus_config_.devices[h.device_index];
    std::string if_name = bus_config_.bus_name + "_" + dev.name + "_" + reg.name;
    hardware_interface::InterfaceInfo inf;
    inf.name = if_name;
    inf.data_type = "double";
    out.push_back(hardware_interface::InterfaceDescription(hardware_name_, inf));
  }
  return out;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!getContext()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  startPollThread();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stopPollThread();
  closeContext();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  init_registers_done_.store(false);
  if (!poll_thread_.joinable()) startPollThread();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stopPollThread();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stopPollThread();
  closeContext();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stopPollThread();
  closeContext();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ModbusSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::vector<double> * ptr = state_buffer_.readFromRT();
  if (!ptr || ptr->size() != state_handles_.size()) return hardware_interface::return_type::OK;
  try {
    for (size_t i = 0; i < state_handles_.size(); ++i) {
      set_state(state_handles_[i].first, (*ptr)[i]);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"), "set_state: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ModbusSystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::vector<double> cmd_vals;
  cmd_vals.reserve(command_handles_.size());
  try {
    for (const auto & [full_name, h] : command_handles_) {
      cmd_vals.push_back(get_command<double>(full_name));
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"), "get_command: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  command_buffer_.writeFromNonRT(cmd_vals);
  return hardware_interface::return_type::OK;
}

}  // namespace modbus_hw_interface

PLUGINLIB_EXPORT_CLASS(
  modbus_hw_interface::ModbusSystemInterface,
  hardware_interface::SystemInterface)
