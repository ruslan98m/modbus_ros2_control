// Copyright 2025 modbus_ros2_control contributors.
// SPDX-License-Identifier: Apache-2.0

#include "modbus_ros2_control/modbus_hardware_interface.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include <sys/resource.h>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_helpers.hpp"

namespace modbus_ros2_control
{

namespace
{
struct RegisterDataTypeHash
{
  std::size_t operator()(RegisterDataType t) const noexcept
  {
    return static_cast<std::size_t>(t);
  }
};

// Data type: full and short names (i8/u8 map to 16-bit; Modbus registers are 16-bit)
static const std::unordered_map<std::string, RegisterDataType> STR_TO_DATA_TYPE = {
  {"bool", RegisterDataType::Bool},
  {"b", RegisterDataType::Bool},
  {"int16", RegisterDataType::Int16},
  {"i16", RegisterDataType::Int16},
  {"i8", RegisterDataType::Int16},
  {"uint16", RegisterDataType::Uint16},
  {"u16", RegisterDataType::Uint16},
  {"u8", RegisterDataType::Uint16},
  {"int32", RegisterDataType::Int32},
  {"i32", RegisterDataType::Int32},
  {"uint32", RegisterDataType::Uint32},
  {"u32", RegisterDataType::Uint32},
  {"float32", RegisterDataType::Float32},
  {"f32", RegisterDataType::Float32},
  {"int64", RegisterDataType::Int64},
  {"i64", RegisterDataType::Int64},
  {"uint64", RegisterDataType::Uint64},
  {"u64", RegisterDataType::Uint64},
  {"float64", RegisterDataType::Float64},
  {"f64", RegisterDataType::Float64},
};

// Register type: full and short names
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

// Canonical string per data type (for interface export)
static const std::unordered_map<RegisterDataType, std::string, RegisterDataTypeHash> DATA_TYPE_TO_STR = {
  {RegisterDataType::Bool, "bool"},
  {RegisterDataType::Int16, "int16"},
  {RegisterDataType::Uint16, "uint16"},
  {RegisterDataType::Int32, "int32"},
  {RegisterDataType::Uint32, "uint32"},
  {RegisterDataType::Float32, "float32"},
  {RegisterDataType::Int64, "int64"},
  {RegisterDataType::Uint64, "uint64"},
  {RegisterDataType::Float64, "float64"},
};

static const std::unordered_map<RegisterDataType, int, RegisterDataTypeHash> DATA_TYPE_TO_REG_COUNT = {
  {RegisterDataType::Bool, 1},
  {RegisterDataType::Int16, 1},
  {RegisterDataType::Uint16, 1},
  {RegisterDataType::Int32, 2},
  {RegisterDataType::Uint32, 2},
  {RegisterDataType::Float32, 2},
  {RegisterDataType::Int64, 4},
  {RegisterDataType::Uint64, 4},
  {RegisterDataType::Float64, 4},
};

inline std::string toLower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

inline RegisterDataType dataTypeFromString(const std::string & s)
{
  auto it = STR_TO_DATA_TYPE.find(toLower(s));
  return (it != STR_TO_DATA_TYPE.end()) ? it->second : RegisterDataType::Uint16;
}

inline RegisterType registerTypeFromString(const std::string & s)
{
  auto it = STR_TO_REG_TYPE.find(toLower(s));
  return (it != STR_TO_REG_TYPE.end()) ? it->second : RegisterType::HoldingRegister;
}

inline std::string dataTypeToInterfaceString(RegisterDataType t)
{
  auto it = DATA_TYPE_TO_STR.find(t);
  return (it != DATA_TYPE_TO_STR.end()) ? it->second : "double";
}

inline int registerCountForDataType(RegisterDataType t)
{
  auto it = DATA_TYPE_TO_REG_COUNT.find(t);
  return (it != DATA_TYPE_TO_REG_COUNT.end()) ? it->second : 1;
}
}  // namespace

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
  lock_memory_ = (get("lock_memory", "false") == "true" || get("lock_memory", "0") == "1");
  preallocate_stack_ = (get("preallocate_stack", "false") == "true" || get("preallocate_stack", "0") == "1");
  return true;
}

bool ModbusSystemInterface::loadDeviceConfigFromFile(const std::string & path, ModbusDeviceConfig & out)
{
  try {
    YAML::Node root = YAML::LoadFile(path);
    out.registers.clear();
    out.init_registers.clear();

    YAML::Node init_list = root["init_registers"];
    if (init_list) {
      for (const auto & r : init_list) {
        ModbusInitRegisterConfig init;
        init.type = registerTypeFromString(r["type"].as<std::string>("holding_register"));
        init.address = r["address"].as<int>(0);
        init.data_type = dataTypeFromString(r["data_type"].as<std::string>("uint16"));
        init.register_count = registerCountForDataType(init.data_type);
        init.value = r["value"].as<double>(0.0);
        out.init_registers.push_back(init);
      }
    }

    YAML::Node regs = root["registers"];
    if (!regs) {
      RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"),
        "Device config '%s': missing 'registers'", path.c_str());
      return false;
    }
    for (const auto & r : regs) {
      ModbusRegisterConfig reg;
      reg.name = r["name"].as<std::string>("");
      reg.type = registerTypeFromString(r["type"].as<std::string>("holding_register"));
      reg.address = r["address"].as<int>(0);
      reg.is_command = r["interface"].as<std::string>("state") == "command";
      reg.data_type = dataTypeFromString(r["data_type"].as<std::string>("uint16"));
      reg.register_count = registerCountForDataType(reg.data_type);
      out.registers.push_back(reg);
    }
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ModbusSystemInterface"),
      "Failed to load device config '%s': %s", path.c_str(), e.what());
    return false;
  }
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

double ModbusSystemInterface::readRegisterValue(modbus_t * ctx, const ModbusRegisterConfig & reg)
{
  switch (reg.type) {
    case RegisterType::Coil: {
      uint8_t tab[1];
      if (modbus_read_bits(ctx, reg.address, 1, tab) == 1) {
        return tab[0] ? 1.0 : 0.0;
      }
      break;
    }
    case RegisterType::DiscreteInput: {
      uint8_t tab[1];
      if (modbus_read_input_bits(ctx, reg.address, 1, tab) == 1) {
        return tab[0] ? 1.0 : 0.0;
      }
      break;
    }
    case RegisterType::InputRegister:
    case RegisterType::HoldingRegister: {
      uint16_t tab[4];
      int nb = reg.register_count;
      int ret = (reg.type == RegisterType::InputRegister)
        ? modbus_read_input_registers(ctx, reg.address, nb, tab)
        : modbus_read_registers(ctx, reg.address, nb, tab);
      if (ret != nb) break;
      if (nb == 1) {
        if (reg.data_type == RegisterDataType::Int16) {
          return static_cast<double>(static_cast<int16_t>(tab[0]));
        }
        return static_cast<double>(tab[0]);
      }
      if (nb == 2) {
        uint32_t u32 = (static_cast<uint32_t>(tab[0]) << 16) | tab[1];
        int32_t i32 = static_cast<int32_t>(u32);
        if (reg.data_type == RegisterDataType::Float32) {
          float f;
          memcpy(&f, &u32, 4);
          return static_cast<double>(f);
        }
        if (reg.data_type == RegisterDataType::Int32) return static_cast<double>(i32);
        return static_cast<double>(u32);
      }
      if (nb == 4) {
        uint64_t u64 = (static_cast<uint64_t>(tab[0]) << 48) | (static_cast<uint64_t>(tab[1]) << 32) |
          (static_cast<uint64_t>(tab[2]) << 16) | tab[3];
        int64_t i64 = static_cast<int64_t>(u64);
        if (reg.data_type == RegisterDataType::Float64) {
          double d;
          memcpy(&d, &u64, 8);
          return d;
        }
        if (reg.data_type == RegisterDataType::Int64) return static_cast<double>(i64);
        return static_cast<double>(u64);
      }
      break;
    }
    default:
      break;
  }
  return 0.0;
}

bool ModbusSystemInterface::writeRegisterValue(
  modbus_t * ctx, const ModbusRegisterConfig & reg, double value)
{
  switch (reg.type) {
    case RegisterType::Coil: {
      int v = (std::fabs(value) > 0.5) ? 1 : 0;
      return modbus_write_bit(ctx, reg.address, v) == 1;
    }
    case RegisterType::HoldingRegister: {
      if (reg.register_count == 1) {
        uint16_t v = static_cast<uint16_t>(value);
        if (reg.data_type == RegisterDataType::Int16) {
          v = static_cast<uint16_t>(static_cast<int16_t>(value));
        }
        return modbus_write_register(ctx, reg.address, v) == 1;
      }
      if (reg.register_count == 2) {
        uint32_t u32;
        if (reg.data_type == RegisterDataType::Float32) {
          float f = static_cast<float>(value);
          memcpy(&u32, &f, 4);
        } else {
          u32 = static_cast<uint32_t>(value);
        }
        uint16_t tab[2] = {static_cast<uint16_t>(u32 >> 16), static_cast<uint16_t>(u32 & 0xFFFF)};
        return modbus_write_registers(ctx, reg.address, 2, tab) == 2;
      }
      if (reg.register_count == 4) {
        uint64_t u64;
        if (reg.data_type == RegisterDataType::Float64) {
          double d = value;
          memcpy(&u64, &d, 8);
        } else if (reg.data_type == RegisterDataType::Int64) {
          u64 = static_cast<uint64_t>(static_cast<int64_t>(value));
        } else {
          u64 = static_cast<uint64_t>(value);
        }
        uint16_t tab[4] = {
          static_cast<uint16_t>(u64 >> 48),
          static_cast<uint16_t>((u64 >> 32) & 0xFFFF),
          static_cast<uint16_t>((u64 >> 16) & 0xFFFF),
          static_cast<uint16_t>(u64 & 0xFFFF),
        };
        return modbus_write_registers(ctx, reg.address, 4, tab) == 4;
      }
    }
    default:
      return false;
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

  return hardware_interface::CallbackReturn::SUCCESS;
}

void ModbusSystemInterface::pollThreadLoop()
{
  const auto logger = rclcpp::get_logger("ModbusSystemInterface");
  try {
    const bool has_realtime = realtime_tools::has_realtime_kernel();

    if (lock_memory_) {
      const auto lock_result = realtime_tools::lock_memory();
      if (!lock_result.first) {
        RCLCPP_WARN(logger, "Unable to lock poll thread memory: '%s'", lock_result.second.c_str());
      } else {
        RCLCPP_INFO(logger, "Poll thread: successfully locked memory");
      }
    }

    if (has_realtime) {
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

    if (preallocate_stack_) {
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
        if (writeRegisterValue(ctx, reg, init.value)) {
          RCLCPP_DEBUG(logger, "Init write dev '%s' addr %d = %g", dev.name.c_str(), init.address, init.value);
        } else {
          RCLCPP_WARN(logger, "Init write failed dev '%s' type %d addr %d", dev.name.c_str(),
            static_cast<int>(init.type), init.address);
        }
      }
    }
  }

  while (poll_running_.load(std::memory_order_relaxed)) {
    std::vector<double> state_vals(state_handles_.size(), 0.0);
    for (size_t i = 0; i < state_handles_.size(); ++i) {
      const auto & h = state_handles_[i].second;
      if (modbus_set_slave(ctx, h.slave_id) >= 0) {
        const auto & reg = bus_config_.devices[h.device_index].registers[h.reg_index];
        state_vals[i] = readRegisterValue(ctx, reg);
      }
    }
    state_buffer_.writeFromNonRT(state_vals);

    std::vector<double> cmd_vals;
    {
      const std::vector<double> * ptr = command_buffer_.readFromNonRT();
      if (ptr && ptr->size() == command_handles_.size()) cmd_vals = *ptr;
    }
    if (!cmd_vals.empty()) {
      for (size_t i = 0; i < command_handles_.size(); ++i) {
        const auto & h = command_handles_[i].second;
        if (modbus_set_slave(ctx, h.slave_id) >= 0) {
          const auto & reg = bus_config_.devices[h.device_index].registers[h.reg_index];
          writeRegisterValue(ctx, reg, cmd_vals[i]);
        }
      }
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
    inf.data_type = dataTypeToInterfaceString(reg.data_type);
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
    inf.data_type = dataTypeToInterfaceString(reg.data_type);
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

}  // namespace modbus_ros2_control

PLUGINLIB_EXPORT_CLASS(
  modbus_ros2_control::ModbusSystemInterface,
  hardware_interface::SystemInterface)
