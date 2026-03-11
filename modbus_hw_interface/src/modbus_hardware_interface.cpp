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
#include <stdexcept>
#include <utility>

#include "modbus_slave_plugins/modbus_slave_interface.hpp"
#include "modbus_slave_plugins/modbus_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace modbus_hw_interface {

bool ModbusSystemInterface::loadBusFromParams(const hardware_interface::HardwareInfo &info) {
  const auto &p = info.hardware_parameters;
  auto get = [&p](const std::string &key, const std::string &def) {
    auto it = p.find(key);
    return (it != p.end()) ? it->second : def;
  };
  bus_config_.bus_name = get("bus_name", "modbus_bus");
  return true;
}

bool ModbusSystemInterface::ensureConnected() {
  if (master_ && master_->isConnected())
    return true;
  if (!master_) {
    RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"), "Master not initialized");
    return false;
  }
  if (!master_->connect()) {
    RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"),
                 "Failed to connect Modbus bus '%s'", bus_config_.bus_name.c_str());
    return false;
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

  master_ = std::make_unique<modbus_master::ModbusMaster>();
  if (!master_->initFromParams(info.hardware_parameters)) {
    RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"),
                 "Modbus master initFromParams failed (invalid connection_type or params)");
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

  command_buffer_.initRT(std::vector<double>(command_handles_.size(), 0.0));

  buildBatchGroups();
  assignDeviceInterfaces();

  cmd_vals_.resize(command_handles_.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

void ModbusSystemInterface::assignDeviceInterfaces() {
  const uint8_t num_devices = static_cast<uint8_t>(modbus_slaves_.size());
  for (uint8_t d = 0; d < num_devices; ++d) {
    std::vector<modbus_hw_interface::InterfaceNameIndex> state_if;
    for (size_t i = 0; i < state_handles_.size(); ++i) {
      if (state_handles_[i].second.device_index == d)
        state_if.emplace_back(state_handles_[i].first, i);
    }
    std::vector<modbus_hw_interface::InterfaceNameIndex> command_if;
    for (size_t i = 0; i < command_handles_.size(); ++i) {
      if (command_handles_[i].second.device_index == d)
        command_if.emplace_back(command_handles_[i].first, i);
    }
    modbus_slaves_[d]->setInterfaces(d, std::move(state_if),
                                     std::move(command_if));
  }
}

void ModbusSystemInterface::startMasterPollLoop() {
  if (!master_)
    return;
  master_->startPollLoop(
      state_handles_.size(),
      command_handles_.size(),
      read_batch_groups_,
      write_batch_groups_,
      bus_config_.devices,
      [this]() {
        const std::vector<double> *p = command_buffer_.readFromNonRT();
        if (p && p->size() == command_handles_.size())
          return *p;
        return std::vector<double>();
      });
}

std::vector<hardware_interface::InterfaceDescription>
ModbusSystemInterface::export_unlisted_state_interface_descriptions() {
  std::vector<hardware_interface::InterfaceDescription> out;
  const std::string prefix = hardware_name_ + "/";
  for (const auto & slave : modbus_slaves_) {
    for (const auto & full_name : slave->stateNames()) {
      hardware_interface::InterfaceInfo inf;
      inf.name = prefix.empty() ? full_name : full_name.substr(prefix.size());
      inf.data_type = "double";
      out.push_back(hardware_interface::InterfaceDescription(hardware_name_, inf));
    }
  }
  return out;
}

std::vector<hardware_interface::InterfaceDescription>
ModbusSystemInterface::export_unlisted_command_interface_descriptions() {
  std::vector<hardware_interface::InterfaceDescription> out;
  const std::string prefix = hardware_name_ + "/";
  for (const auto & slave : modbus_slaves_) {
    for (const auto & full_name : slave->commandNames()) {
      hardware_interface::InterfaceInfo inf;
      inf.name = prefix.empty() ? full_name : full_name.substr(prefix.size());
      inf.data_type = "double";
      out.push_back(hardware_interface::InterfaceDescription(hardware_name_, inf));
    }
  }
  return out;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!ensureConnected()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  startMasterPollLoop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (master_)
    master_->stopPollLoop();
  closeContext();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (master_)
    master_->resetInitRegisters();
  if (master_ && !master_->isPollLoopRunning())
    startMasterPollLoop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (master_)
    master_->stopPollLoop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (master_)
    master_->stopPollLoop();
  closeContext();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModbusSystemInterface::on_error(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (master_)
    master_->stopPollLoop();
  closeContext();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ModbusSystemInterface::read(const rclcpp::Time & /*time*/,
                                                            const rclcpp::Duration & /*period*/) {
  const std::vector<double> * ptr = master_->readStateSnapshotForRT();
  if (!ptr || ptr->size() != state_handles_.size())
    return hardware_interface::return_type::OK;
  try {
    auto set_state_cb = [this](const std::string & name, double value) { set_state(name, value); };
    for (const auto & slave : modbus_slaves_)
      slave->readState(*ptr, set_state_cb);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"), "read: %s", e.what());
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
    for (const auto & slave : modbus_slaves_)
      slave->writeCommand(get_command_cb, cmd_vals_);
    command_buffer_.writeFromNonRT(cmd_vals_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"), "write: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace modbus_hw_interface

PLUGINLIB_EXPORT_CLASS(modbus_hw_interface::ModbusSystemInterface,
                       hardware_interface::SystemInterface)
