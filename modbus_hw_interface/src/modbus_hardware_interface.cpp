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

#include "modbus_master/connection_params.hpp"
#include "modbus_slave_plugins/modbus_slave_interface.hpp"
#include "modbus_slave_plugins/modbus_utils.hpp"
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
    bus_config_.tcp_params.ip_address = get("ip_address", "127.0.0.1");
    try {
      bus_config_.tcp_params.port = static_cast<uint16_t>(std::stoi(get("port", "502")));
    } catch (...) {
      bus_config_.tcp_params.port = 502;
    }
  } else if (type_str == "rtu") {
    bus_config_.is_tcp = false;
    bus_config_.rtu_params.serial_port = get("serial_port", "/dev/ttyUSB0");
    try {
      bus_config_.rtu_params.baud_rate = static_cast<uint32_t>(std::stoi(get("baud_rate", "9600")));
    } catch (...) {
      bus_config_.rtu_params.baud_rate = 9600;
    }
    std::string par = get("parity", "N");
    bus_config_.rtu_params.parity = par.empty() ? 'N' : par[0];
    try {
      bus_config_.rtu_params.data_bits = static_cast<uint8_t>(std::stoi(get("data_bits", "8")));
      bus_config_.rtu_params.stop_bits = static_cast<uint8_t>(std::stoi(get("stop_bits", "1")));
    } catch (...) {
      bus_config_.rtu_params.data_bits = 8;
      bus_config_.rtu_params.stop_bits = 1;
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

bool ModbusSystemInterface::ensureConnected() {
  if (master_ && master_->isConnected())
    return true;
  if (!master_)
    master_ = std::make_unique<modbus_master::ModbusMaster>();
  if (bus_config_.is_tcp) {
    if (!master_->connect(bus_config_.tcp_params)) {
      RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"),
                   "Failed to connect Modbus bus '%s'", bus_config_.bus_name.c_str());
      return false;
    }
  } else {
    if (!master_->connect(bus_config_.rtu_params)) {
      RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"),
                   "Failed to connect Modbus bus '%s'", bus_config_.bus_name.c_str());
      return false;
    }
  }
  return true;
}

void ModbusSystemInterface::closeContext() {
  if (master_) {
    master_->disconnect();
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
        modbus_master::BatchGroup grp;
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
        modbus_master::BatchGroup grp;
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

bool ModbusSystemInterface::loadDevicesFromComponents(
    const std::vector<hardware_interface::ComponentInfo> &components,
    const std::string &component_type) {
  const auto logger = rclcpp::get_logger("ModbusSystemInterface");
  for (const auto &comp : components) {
    auto it_plugin = comp.parameters.find("plugin");
    if (it_plugin == comp.parameters.end() || it_plugin->second.empty()) {
      RCLCPP_ERROR(logger,
                   "%s '%s': missing param 'plugin' (e.g. modbus_slave_plugins/GenericModbusSlave). "
                   "Config parsing is done only inside the plugin.",
                   component_type.c_str(), comp.name.c_str());
      return false;
    }

    if (!modbus_slave_loader_) {
      try {
        modbus_slave_loader_ =
            std::make_unique<pluginlib::ClassLoader<ModbusSlaveInterface>>(
                "modbus_slave_plugins", "modbus_hw_interface::ModbusSlaveInterface");
      } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_ERROR(logger, "Failed to create Modbus slave plugin loader: %s", ex.what());
        return false;
      }
    }

    ModbusDeviceConfig dev;
    dev.name = comp.name;
    try {
      auto slave = modbus_slave_loader_->createSharedInstance(it_plugin->second);
      if (!slave->setupSlave(comp.name, comp.parameters, dev)) {
        RCLCPP_ERROR(logger, "Plugin '%s' setupSlave failed for %s '%s'",
                    it_plugin->second.c_str(), component_type.c_str(), comp.name.c_str());
        return false;
      }
      modbus_slaves_.push_back(slave);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(logger, "Failed to load Modbus slave plugin '%s' for %s '%s': %s",
                  it_plugin->second.c_str(), component_type.c_str(), comp.name.c_str(),
                  ex.what());
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
  modbus_slaves_.clear();

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
  buildRtBuffers();

  return hardware_interface::CallbackReturn::SUCCESS;
}

void ModbusSystemInterface::buildRtBuffers() {
  const size_t num_devices = bus_config_.devices.size();
  state_names_per_device_.resize(num_devices);
  state_vals_per_device_.resize(num_devices);
  command_names_per_device_.resize(num_devices);
  command_out_per_device_.resize(num_devices);
  command_global_index_.resize(num_devices);

  state_to_device_local_.resize(state_handles_.size());
  std::vector<size_t> device_state_count(num_devices, 0);
  for (size_t i = 0; i < state_handles_.size(); ++i) {
    const size_t d = state_handles_[i].second.device_index;
    const size_t j = device_state_count[d]++;
    state_to_device_local_[i] = {d, j};
    state_names_per_device_[d].push_back(state_handles_[i].first);
    state_vals_per_device_[d].push_back(0.0);
  }

  for (size_t d = 0; d < num_devices; ++d) {
    for (size_t i = 0; i < command_handles_.size(); ++i) {
      if (command_handles_[i].second.device_index == d) {
        command_names_per_device_[d].push_back(command_handles_[i].first);
        command_out_per_device_[d].push_back(0.0);
        command_global_index_[d].push_back(i);
      }
    }
  }

  cmd_vals_.resize(command_handles_.size(), 0.0);
}

void ModbusSystemInterface::pollThreadLoop() {
  const auto logger = rclcpp::get_logger("ModbusSystemInterface");
  applyRealtimeThreadParams(logger, thread_priority_, cpu_affinity_cores_);

  const bool use_poll_delay = (poll_rate_hz_ > 0.0);
  const auto period = use_poll_delay ? std::chrono::duration<double>(1.0 / poll_rate_hz_)
                                     : std::chrono::duration<double>(0);
  if (!master_ || !master_->isConnected())
    return;

  if (!init_registers_done_.exchange(true)) {
    master_->writeInitRegisters(bus_config_.devices);
  }

  while (poll_running_.load(std::memory_order_relaxed)) {
    const auto iteration_start = std::chrono::steady_clock::now();

    master_->readStateBatched(read_batch_groups_, bus_config_.devices, state_poll_buffer_);
    state_buffer_.writeFromNonRT(state_poll_buffer_);

    const std::vector<double> *ptr = command_buffer_.readFromNonRT();
    if (ptr && ptr->size() == command_handles_.size()) {
      master_->writeCommandBatched(write_batch_groups_, bus_config_.devices, *ptr);
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
  if (!ensureConnected())
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
  if (!ensureConnected()) {
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
  const std::vector<double> * ptr = state_buffer_.readFromRT();
  if (!ptr || ptr->size() != state_handles_.size() ||
      state_vals_per_device_.size() != modbus_slaves_.size()) {
    return hardware_interface::return_type::OK;
  }
  try {
    for (size_t i = 0; i < state_handles_.size(); ++i) {
      const auto [d, j] = state_to_device_local_[i];
      state_vals_per_device_[d][j] = (*ptr)[i];
    }
    auto set_state_cb = [this](const std::string & name, double value) { set_state(name, value); };
    for (size_t d = 0; d < modbus_slaves_.size(); ++d) {
      auto & vals = state_vals_per_device_[d];
      if (!vals.empty()) {
        modbus_slaves_[d]->updateState(d, state_names_per_device_[d], vals.data(), vals.size(),
                                      set_state_cb);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"), "read (updateState): %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ModbusSystemInterface::write(const rclcpp::Time & /*time*/,
                                                             const rclcpp::Duration & /*period*/) {
  if (cmd_vals_.size() != command_handles_.size())
    return hardware_interface::return_type::OK;
  try {
    auto get_command_cb = [this](const std::string & name) {
      return get_command<double>(name);
    };
    for (size_t d = 0; d < modbus_slaves_.size(); ++d) {
      if (d >= command_global_index_.size() || command_names_per_device_[d].empty())
        continue;
      modbus_slaves_[d]->getCommand(d, command_names_per_device_[d], get_command_cb,
                                   command_out_per_device_[d]);
      const auto & out = command_out_per_device_[d];
      const auto & indices = command_global_index_[d];
      for (size_t j = 0; j < out.size() && j < indices.size(); ++j)
        cmd_vals_[indices[j]] = out[j];
    }
    command_buffer_.writeFromNonRT(cmd_vals_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"), "write (getCommand): %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace modbus_hw_interface

PLUGINLIB_EXPORT_CLASS(modbus_hw_interface::ModbusSystemInterface,
                       hardware_interface::SystemInterface)
