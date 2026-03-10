// Copyright 2025 modbus_hw_interface contributors.
// SPDX-License-Identifier: Apache-2.0

#include "modbus_hw_interface/modbus_hardware_interface.hpp"

#include <sys/resource.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "modbus_hw_interface/modbus_config_loader.hpp"
#include "modbus_hw_interface/modbus_io.hpp"
#include "modbus_hw_interface/modbus_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_helpers.hpp"

namespace modbus_hw_interface {

bool ModbusSystemInterface::loadBusFromParams(const hardware_interface::HardwareInfo &info) {
  const auto &p = info.hardware_parameters;
  auto get = [&p](const std::string &key, const std::string &def) {
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
    poll_rate_hz_ = std::stod(get("poll_rate_hz", "0"));
  } catch (...) {
    poll_rate_hz_ = 0.0;
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
        if (c >= 0)
          cpu_affinity_cores_.push_back(c);
      } catch (...) {
      }
    }
  }

  return true;
}

modbus_t *ModbusSystemInterface::getContext() {
  if (ctx_)
    return ctx_;
  const auto &c = bus_config_;
  if (c.is_tcp) {
    ctx_ = modbus_new_tcp(c.ip_address.c_str(), c.port);
  } else {
    ctx_ = modbus_new_rtu(c.serial_port.c_str(), c.baud_rate, c.parity, c.data_bits, c.stop_bits);
  }
  if (!ctx_) {
    RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"),
                 "Failed to create Modbus context for bus '%s'", c.bus_name.c_str());
    return nullptr;
  }
  if (modbus_connect(ctx_) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"),
                 "Failed to connect Modbus bus '%s': %s", c.bus_name.c_str(),
                 modbus_strerror(errno));
    modbus_free(ctx_);
    ctx_ = nullptr;
    return nullptr;
  }
  return ctx_;
}

void ModbusSystemInterface::setContextResponseTimeout(modbus_t *ctx, size_t device_index) {
  if (ctx == nullptr || device_index >= bus_config_.devices.size())
    return;
  const auto &dev = bus_config_.devices[device_index];
  if (dev.response_timeout_sec <= 0)
    return;
  double s = dev.response_timeout_sec;
  uint32_t to_sec = static_cast<uint32_t>(s);
  uint32_t to_usec = static_cast<uint32_t>((s - to_sec) * 1e6);
  modbus_set_response_timeout(ctx, to_sec, to_usec);
}

void ModbusSystemInterface::closeContext() {
  if (ctx_) {
    modbus_close(ctx_);
    modbus_free(ctx_);
    ctx_ = nullptr;
  }
}

void ModbusSystemInterface::buildBatchGroups() {
  read_batch_groups_.clear();
  write_batch_groups_.clear();

  auto build_read = [this]() {
    std::map<std::pair<int, size_t>, std::vector<size_t>> by_device;
    for (size_t i = 0; i < state_handles_.size(); ++i) {
      const auto &h = state_handles_[i].second;
      by_device[{static_cast<int>(h.slave_id), static_cast<size_t>(h.device_index)}].push_back(i);
    }
    for (const auto &[key, indices] : by_device) {
      const int slave_id = key.first;
      const size_t dev_idx = key.second;
      const auto &dev = bus_config_.devices[dev_idx];
      std::vector<std::pair<size_t, const ModbusRegisterConfig *>> items;
      for (size_t idx : indices) {
        const auto &h = state_handles_[idx].second;
        items.push_back({idx, &dev.registers[h.reg_index]});
      }
      std::sort(items.begin(), items.end(), [](const auto &a, const auto &b) {
        if (a.second->type != b.second->type) {
          return static_cast<int>(a.second->type) < static_cast<int>(b.second->type);
        }
        return a.second->address < b.second->address;
      });
      for (size_t i = 0; i < items.size();) {
        const RegisterType cur_type = items[i].second->type;
        int cur_addr = items[i].second->address;
        BatchGroup grp;
        grp.device_index = static_cast<uint8_t>(dev_idx);
        grp.slave_id = static_cast<uint8_t>(slave_id);
        grp.type = cur_type;
        grp.start_address = static_cast<uint16_t>(cur_addr);
        grp.total_count = 0;
        grp.use_batch = (cur_type == RegisterType::Coil || cur_type == RegisterType::DiscreteInput)
                            ? dev.read_multiple_coils
                            : dev.read_multiple_registers;
        while (i < items.size() && items[i].second->type == cur_type &&
               items[i].second->address == cur_addr) {
          const auto *reg = items[i].second;
          grp.items.push_back({reg->register_count, reg->data_type, reg,
                               static_cast<uint16_t>(items[i].first)});
          grp.total_count += static_cast<uint16_t>(reg->register_count);
          cur_addr += reg->register_count;
          ++i;
        }
        if (grp.use_batch && grp.total_count > 0) {
          if (cur_type == RegisterType::Coil || cur_type == RegisterType::DiscreteInput) {
            grp.buffer.resize(static_cast<size_t>(grp.total_count));
          } else {
            grp.buffer.resize(static_cast<size_t>(grp.total_count) * sizeof(uint16_t));
          }
        }
        read_batch_groups_.push_back(std::move(grp));
      }
    }
  };

  auto build_write = [this]() {
    std::map<std::pair<int, size_t>, std::vector<size_t>> by_device;
    for (size_t i = 0; i < command_handles_.size(); ++i) {
      const auto &h = command_handles_[i].second;
      by_device[{static_cast<int>(h.slave_id), static_cast<size_t>(h.device_index)}].push_back(i);
    }
    for (const auto &[key, indices] : by_device) {
      const int slave_id = key.first;
      const size_t dev_idx = key.second;
      const auto &dev = bus_config_.devices[dev_idx];
      std::vector<std::pair<size_t, const ModbusRegisterConfig *>> items;
      for (size_t idx : indices) {
        const auto &h = command_handles_[idx].second;
        items.push_back({idx, &dev.registers[h.reg_index]});
      }
      std::sort(items.begin(), items.end(), [](const auto &a, const auto &b) {
        if (a.second->type != b.second->type) {
          return static_cast<int>(a.second->type) < static_cast<int>(b.second->type);
        }
        return a.second->address < b.second->address;
      });
      for (size_t i = 0; i < items.size();) {
        const RegisterType cur_type = items[i].second->type;
        uint16_t cur_addr = items[i].second->address;
        if (cur_type == RegisterType::DiscreteInput || cur_type == RegisterType::InputRegister) {
          ++i;
          continue;
        }
        BatchGroup grp;
        grp.device_index = static_cast<uint8_t>(dev_idx);
        grp.slave_id = static_cast<uint8_t>(slave_id);
        grp.type = cur_type;
        grp.start_address = cur_addr;
        grp.total_count = 0;
        grp.use_batch = (cur_type == RegisterType::Coil) ? dev.write_multiple_coils
                                                         : dev.write_multiple_registers;
        while (i < items.size() && items[i].second->type == cur_type &&
               items[i].second->address == cur_addr) {
          const auto *reg = items[i].second;
          grp.items.push_back({reg->register_count, reg->data_type, reg,
                               static_cast<uint16_t>(items[i].first)});
          grp.total_count += static_cast<uint16_t>(reg->register_count);
          cur_addr += reg->register_count;
          ++i;
        }
        if (grp.use_batch && grp.total_count > 0) {
          if (cur_type == RegisterType::HoldingRegister) {
            grp.buffer.resize(static_cast<size_t>(grp.total_count) * sizeof(uint16_t));
          } else if (cur_type == RegisterType::Coil) {
            grp.buffer.resize(grp.items.size());
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

void ModbusSystemInterface::readStateBatched(modbus_t *ctx, std::vector<double> &state_vals) {
  if (state_vals.size() != state_handles_.size())
    return;
  std::fill(state_vals.begin(), state_vals.end(), 0.0);
  for (auto &grp : read_batch_groups_) {
    setContextResponseTimeout(ctx, static_cast<size_t>(grp.device_index));
    if (modbus_set_slave(ctx, static_cast<int>(grp.slave_id)) < 0)
      continue;
    if (grp.use_batch && grp.total_count > 0 && !grp.buffer.empty()) {
      if (grp.type == RegisterType::Coil) {
        if (modbus_read_bits(ctx, grp.start_address, grp.total_count, grp.buffer.data()) ==
            static_cast<int>(grp.total_count)) {
          size_t off = 0;
          for (const auto &it : grp.items) {
            state_vals[it.index] = grp.buffer[off] ? 1.0 : 0.0;
            off += static_cast<size_t>(it.register_count);
          }
        }
      } else if (grp.type == RegisterType::DiscreteInput) {
        if (modbus_read_input_bits(ctx, grp.start_address, grp.total_count, grp.buffer.data()) ==
            static_cast<int>(grp.total_count)) {
          size_t off = 0;
          for (const auto &it : grp.items) {
            state_vals[it.index] = grp.buffer[off] ? 1.0 : 0.0;
            off += static_cast<size_t>(it.register_count);
          }
        }
      } else {
        uint16_t *regs = reinterpret_cast<uint16_t *>(grp.buffer.data());
        int ret = (grp.type == RegisterType::InputRegister)
                      ? modbus_read_input_registers(ctx, grp.start_address, grp.total_count, regs)
                      : modbus_read_registers(ctx, grp.start_address, grp.total_count, regs);
        if (ret == static_cast<int>(grp.total_count)) {
          size_t off = 0;
          for (const auto &it : grp.items) {
            state_vals[it.index] = decodeRegistersFromBuffer(regs, off, it.register_count, it.data_type);
            off += static_cast<size_t>(it.register_count);
          }
        }
      }
    } else {
      for (const auto &it : grp.items) {
        state_vals[it.index] = readRegisterValue(ctx, *it.reg);
      }
    }
  }
}

void ModbusSystemInterface::writeCommandBatched(modbus_t *ctx,
                                                const std::vector<double> &command_vals) {
  if (command_vals.size() != command_handles_.size())
    return;
  for (auto &grp : write_batch_groups_) {
    setContextResponseTimeout(ctx, static_cast<size_t>(grp.device_index));
    if (modbus_set_slave(ctx, static_cast<int>(grp.slave_id)) < 0)
      continue;
    if (grp.use_batch && grp.total_count > 0 && grp.type == RegisterType::HoldingRegister &&
        !grp.buffer.empty()) {
      uint16_t *regs = reinterpret_cast<uint16_t *>(grp.buffer.data());
      size_t off = 0;
      for (const auto &it : grp.items) {
        const double val = command_vals[it.index];
        const ModbusRegisterConfig *reg = it.reg;
        if (reg->register_count == 1) {
          regs[off] = (reg->data_type == RegisterDataType::Int16)
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
          regs[off] = static_cast<uint16_t>(u32 >> 16);
          regs[off + 1] = static_cast<uint16_t>(u32 & 0xFFFF);
        } else if (reg->register_count == 4) {
          uint64_t u64;
          if (reg->data_type == RegisterDataType::Float64) {
            memcpy(&u64, &val, 8);
          } else if (reg->data_type == RegisterDataType::Int64) {
            u64 = static_cast<uint64_t>(static_cast<int64_t>(val));
          } else {
            u64 = static_cast<uint64_t>(val);
          }
          regs[off] = static_cast<uint16_t>(u64 >> 48);
          regs[off + 1] = static_cast<uint16_t>((u64 >> 32) & 0xFFFF);
          regs[off + 2] = static_cast<uint16_t>((u64 >> 16) & 0xFFFF);
          regs[off + 3] = static_cast<uint16_t>(u64 & 0xFFFF);
        }
        off += static_cast<size_t>(reg->register_count);
      }
      modbus_write_registers(ctx, grp.start_address, grp.total_count, regs);
    } else if (grp.use_batch && grp.type == RegisterType::Coil && !grp.buffer.empty()) {
      for (size_t g = 0; g < grp.buffer.size(); ++g) {
        grp.buffer[g] = (std::fabs(command_vals[grp.items[g].index]) > 0.5) ? 1 : 0;
      }
      modbus_write_bits(ctx, grp.start_address, static_cast<int>(grp.buffer.size()), grp.buffer.data());
    } else {
      for (const auto &it : grp.items) {
        writeRegisterValue(ctx, *it.reg, command_vals[it.index]);
      }
    }
  }
}

bool ModbusSystemInterface::loadDevicesFromComponents(
    const std::vector<hardware_interface::ComponentInfo> &components,
    const std::string &component_type) {
  const auto logger = rclcpp::get_logger("ModbusSystemInterface");
  ModbusDeviceConfigLoader config_loader(logger);
  for (const auto &comp : components) {
    auto it_cfg = comp.parameters.find("device_config");
    if (it_cfg == comp.parameters.end()) {
      it_cfg = comp.parameters.find("slave_config");
    }
    if (it_cfg == comp.parameters.end() || it_cfg->second.empty()) {
      RCLCPP_ERROR(logger,
                   "%s '%s': missing param 'device_config' or 'slave_config' (path to device YAML)",
                   component_type.c_str(), comp.name.c_str());
      return false;
    }
    int slave_id = 1;
    auto it_slave = comp.parameters.find("slave_id");
    if (it_slave != comp.parameters.end()) {
      try {
        slave_id = std::stoi(it_slave->second);
      } catch (...) {
      }
    }
    ModbusDeviceConfig dev;
    dev.name = comp.name;
    dev.slave_id = slave_id;
    if (!config_loader.load(it_cfg->second, dev)) {
      RCLCPP_ERROR(logger, "Failed to load device config from '%s'", it_cfg->second.c_str());
      return false;
    }
    bus_config_.devices.push_back(dev);
  }
  return true;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
  const auto &info = params.hardware_info;
  hardware_name_ = info.name;
  bus_config_.devices.clear();

  if (!loadBusFromParams(info)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!loadDevicesFromComponents(info.joints, "Joint")) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!loadDevicesFromComponents(info.gpios, "GPIO")) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!loadDevicesFromComponents(info.sensors, "Sensor")) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  state_handles_.clear();
  command_handles_.clear();
  poll_running_.store(false);

  for (size_t di = 0; di < bus_config_.devices.size(); ++di) {
    const auto &dev = bus_config_.devices[di];
    std::string dev_prefix = bus_config_.bus_name + "_" + dev.name;
    for (size_t ri = 0; ri < dev.registers.size(); ++ri) {
      const auto &reg = dev.registers[ri];
      std::string if_name = dev_prefix + "_" + reg.interface_name;
      std::string full_name = hardware_name_ + "/" + if_name;
      if (reg.is_command) {
        command_handles_.push_back(
            {full_name, {static_cast<uint8_t>(dev.slave_id), static_cast<uint8_t>(di), static_cast<uint16_t>(ri)}});
      } else {
        state_handles_.push_back(
            {full_name, {static_cast<uint8_t>(dev.slave_id), static_cast<uint8_t>(di), static_cast<uint16_t>(ri)}});
      }
    }
  }

  state_buffer_.initRT(std::vector<double>(state_handles_.size(), 0.0));
  command_buffer_.initRT(std::vector<double>(command_handles_.size(), 0.0));

  buildBatchGroups();

  return hardware_interface::CallbackReturn::SUCCESS;
}

void ModbusSystemInterface::pollThreadLoop() {
  const auto logger = rclcpp::get_logger("ModbusSystemInterface");
  applyRealtimeThreadParams(logger, thread_priority_, cpu_affinity_cores_);

  const bool use_poll_delay = (poll_rate_hz_ > 0.0);
  const auto period = use_poll_delay ? std::chrono::duration<double>(1.0 / poll_rate_hz_)
                                     : std::chrono::duration<double>(0);
  modbus_t *ctx = getContext();
  if (!ctx)
    return;

  if (!init_registers_done_.exchange(true)) {
    for (size_t di = 0; di < bus_config_.devices.size(); ++di) {
      const auto &dev = bus_config_.devices[di];
      if (dev.init_registers.empty())
        continue;
      setContextResponseTimeout(ctx, di);
      if (modbus_set_slave(ctx, dev.slave_id) < 0) {
        RCLCPP_WARN(logger, "Init registers: failed to set slave_id %d", dev.slave_id);
        continue;
      }
      for (const auto &init : dev.init_registers) {
        ModbusRegisterConfig reg;
        reg.type = init.type;
        reg.address = static_cast<uint16_t>(init.address);
        reg.data_type = init.data_type;
        reg.register_count = static_cast<uint16_t>(init.register_count);
        if (modbus_hw_interface::writeRegisterValue(ctx, reg, init.value)) {
          RCLCPP_DEBUG(logger, "Init write dev '%s' addr %d = %g", dev.name.c_str(), init.address,
                       init.value);
        } else {
          RCLCPP_WARN(logger, "Init write failed dev '%s' type %d addr %d", dev.name.c_str(),
                      static_cast<int>(init.type), init.address);
        }
      }
    }
  }

  while (poll_running_.load(std::memory_order_relaxed)) {
    const auto iteration_start = std::chrono::steady_clock::now();

    readStateBatched(ctx, state_poll_buffer_);
    state_buffer_.writeFromNonRT(state_poll_buffer_);

    const std::vector<double> *ptr = command_buffer_.readFromNonRT();
    if (ptr && ptr->size() == command_handles_.size()) {
      writeCommandBatched(ctx, *ptr);
    }

    if (use_poll_delay) {
      const auto elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - iteration_start);
      const auto remaining = period - elapsed;
      if (remaining.count() > 0) {
        std::this_thread::sleep_for(remaining);
      } else {
        RCLCPP_WARN(logger, "Poll delay: elapsed time %g > period %g", elapsed.count(),
                    period.count());
      }
    }
  }
}

void ModbusSystemInterface::startPollThread() {
  if (poll_thread_.joinable())
    return;
  poll_running_.store(true);
  poll_thread_ = std::thread(&ModbusSystemInterface::pollThreadLoop, this);
}

void ModbusSystemInterface::stopPollThread() {
  poll_running_.store(false);
  if (poll_thread_.joinable()) {
    poll_thread_.join();
    poll_thread_ = std::thread();
  }
}

std::vector<hardware_interface::InterfaceDescription>
ModbusSystemInterface::export_unlisted_state_interface_descriptions() {
  std::vector<hardware_interface::InterfaceDescription> out;
  for (const auto &[full_name, h] : state_handles_) {
    const auto &reg = bus_config_.devices[h.device_index].registers[h.reg_index];
    const auto &dev = bus_config_.devices[h.device_index];
    std::string if_name = bus_config_.bus_name + "_" + dev.name + "_" + reg.interface_name;
    hardware_interface::InterfaceInfo inf;
    inf.name = if_name;
    inf.data_type = "double";
    out.push_back(hardware_interface::InterfaceDescription(hardware_name_, inf));
  }
  return out;
}

std::vector<hardware_interface::InterfaceDescription>
ModbusSystemInterface::export_unlisted_command_interface_descriptions() {
  std::vector<hardware_interface::InterfaceDescription> out;
  for (const auto &[full_name, h] : command_handles_) {
    const auto &reg = bus_config_.devices[h.device_index].registers[h.reg_index];
    const auto &dev = bus_config_.devices[h.device_index];
    std::string if_name = bus_config_.bus_name + "_" + dev.name + "_" + reg.interface_name;
    hardware_interface::InterfaceInfo inf;
    inf.name = if_name;
    inf.data_type = "double";
    out.push_back(hardware_interface::InterfaceDescription(hardware_name_, inf));
  }
  return out;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!getContext()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  startPollThread();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  stopPollThread();
  closeContext();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  init_registers_done_.store(false);
  if (!poll_thread_.joinable())
    startPollThread();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  stopPollThread();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  stopPollThread();
  closeContext();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_error(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  stopPollThread();
  closeContext();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ModbusSystemInterface::read(const rclcpp::Time & /*time*/,
                                                            const rclcpp::Duration & /*period*/) {
  std::vector<double> *ptr = state_buffer_.readFromRT();
  if (!ptr || ptr->size() != state_handles_.size())
    return hardware_interface::return_type::OK;
  try {
    for (size_t i = 0; i < state_handles_.size(); ++i) {
      set_state(state_handles_[i].first, (*ptr)[i]);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"), "set_state: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ModbusSystemInterface::write(const rclcpp::Time & /*time*/,
                                                             const rclcpp::Duration & /*period*/) {
  std::vector<double> cmd_vals;
  cmd_vals.reserve(command_handles_.size());
  try {
    for (const auto &[full_name, h] : command_handles_) {
      cmd_vals.push_back(get_command<double>(full_name));
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"), "get_command: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  command_buffer_.writeFromNonRT(cmd_vals);
  return hardware_interface::return_type::OK;
}

}  // namespace modbus_hw_interface

PLUGINLIB_EXPORT_CLASS(modbus_hw_interface::ModbusSystemInterface,
                       hardware_interface::SystemInterface)
