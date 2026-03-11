// Copyright 2025 modbus_master contributors.
// SPDX-License-Identifier: Apache-2.0

#ifndef MODBUS_MASTER__CONNECTION_PARAMS_HPP_
#define MODBUS_MASTER__CONNECTION_PARAMS_HPP_

#include <cstdint>
#include <string>

namespace modbus_master {

/** Connection parameters for Modbus TCP. */
struct TcpConnectionParams {
  std::string ip_address{"127.0.0.1"};
  uint16_t port{502};
};

/** Connection parameters for Modbus RTU (serial). */
struct RtuConnectionParams {
  std::string serial_port{"/dev/ttyUSB0"};
  uint32_t baud_rate{9600};
  char parity{'N'};
  uint8_t data_bits{8};
  uint8_t stop_bits{1};
};

}  // namespace modbus_master

#endif  // MODBUS_MASTER__CONNECTION_PARAMS_HPP_
