// Copyright 2025 modbus_master contributors.
// SPDX-License-Identifier: Apache-2.0

#ifndef MODBUS_MASTER__MASTER_PARAMS_HPP_
#define MODBUS_MASTER__MASTER_PARAMS_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "modbus_master/connection_params.hpp"

namespace modbus_master {

/** Parsed master params: connection + poll thread settings. */
struct MasterParams {
  bool is_tcp{true};
  TcpConnectionParams tcp_params;
  RtuConnectionParams rtu_params;
  double poll_rate_hz{0.0};  // 0 = no delay
  int thread_priority{50};
  std::vector<int> cpu_affinity_cores;
};

/**
 * Parse hardware_parameters map (e.g. from URDF) into MasterParams.
 * Keys: connection_type, ip_address, port, serial_port, baud_rate, parity,
 * data_bits, stop_bits, poll_rate_hz, thread_priority, cpu_affinity.
 * @return true on success, false on invalid connection_type.
 */
bool parseMasterParams(const std::unordered_map<std::string, std::string>& params,
                       MasterParams& out);

}  // namespace modbus_master

#endif  // MODBUS_MASTER__MASTER_PARAMS_HPP_
