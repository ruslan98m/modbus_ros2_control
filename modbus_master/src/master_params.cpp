// Copyright 2025 modbus_master contributors.
// SPDX-License-Identifier: Apache-2.0

#include "modbus_master/master_params.hpp"

#include <sstream>

namespace modbus_master {

bool parseMasterParams(const std::unordered_map<std::string, std::string>& params,
                       MasterParams& out) {
  auto get = [&params](const std::string& key, const std::string& def) {
    auto it = params.find(key);
    return (it != params.end()) ? it->second : def;
  };

  const std::string type_str = get("connection_type", "tcp");
  if (type_str == "tcp") {
    out.is_tcp = true;
    out.tcp_params.ip_address = get("ip_address", "127.0.0.1");
    try {
      out.tcp_params.port = static_cast<uint16_t>(std::stoi(get("port", "502")));
    } catch (...) {
      out.tcp_params.port = 502;
    }
  } else if (type_str == "rtu") {
    out.is_tcp = false;
    out.rtu_params.serial_port = get("serial_port", "/dev/ttyUSB0");
    try {
      out.rtu_params.baud_rate =
          static_cast<uint32_t>(std::stoi(get("baud_rate", "9600")));
    } catch (...) {
      out.rtu_params.baud_rate = 9600;
    }
    const std::string par = get("parity", "N");
    out.rtu_params.parity = par.empty() ? 'N' : par[0];
    try {
      out.rtu_params.data_bits =
          static_cast<uint8_t>(std::stoi(get("data_bits", "8")));
      out.rtu_params.stop_bits =
          static_cast<uint8_t>(std::stoi(get("stop_bits", "1")));
    } catch (...) {
      out.rtu_params.data_bits = 8;
      out.rtu_params.stop_bits = 1;
    }
  } else {
    return false;
  }

  try {
    out.poll_rate_hz = std::stod(get("poll_rate_hz", "0"));
  } catch (...) {
    out.poll_rate_hz = 0.0;
  }
  try {
    out.thread_priority = std::stoi(get("thread_priority", "50"));
  } catch (...) {
    out.thread_priority = 50;
  }
  out.cpu_affinity_cores.clear();
  const std::string affinity_str = get("cpu_affinity", "");
  if (!affinity_str.empty()) {
    std::istringstream ss(affinity_str);
    std::string part;
    while (std::getline(ss, part, ',')) {
      try {
        size_t pos = 0;
        int c = std::stoi(part, &pos);
        if (c >= 0) {
          out.cpu_affinity_cores.push_back(c);
        }
      } catch (...) {
      }
    }
  }
  return true;
}

}  // namespace modbus_master
