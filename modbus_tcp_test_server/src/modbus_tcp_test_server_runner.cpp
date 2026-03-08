// Copyright 2025 modbus_ros2_control contributors.
// SPDX-License-Identifier: Apache-2.0

#include "modbus_tcp_test_server/modbus_tcp_test_server_runner.hpp"

#include <unistd.h>

namespace modbus_tcp_test_server {

bool ModbusTcpTestServerRunner::open(int port, int slave_id) {
  if (ctx_ != nullptr) {
    close();
  }

  ctx_ = modbus_new_tcp(nullptr, port);
  if (!ctx_) {
    return false;
  }

  modbus_set_slave(ctx_, slave_id);
  port_ = port;
  slave_id_ = slave_id;

  mb_mapping_ =
      modbus_mapping_new(NB_COILS, NB_DISCRETE_INPUTS, NB_HOLDING_REGISTERS, NB_INPUT_REGISTERS);
  if (!mb_mapping_) {
    modbus_free(ctx_);
    ctx_ = nullptr;
    return false;
  }

  // Coils: value = address % 2 (even -> 0, odd -> 1)
  for (int i = 0; i < NB_COILS; ++i) {
    mb_mapping_->tab_bits[i] = static_cast<uint8_t>(i % 2);
  }
  for (int i = 0; i < NB_DISCRETE_INPUTS; ++i) {
    mb_mapping_->tab_input_bits[i] = static_cast<uint8_t>(i % 2);
  }
  // Registers: value = address
  for (int i = 0; i < NB_HOLDING_REGISTERS; ++i) {
    mb_mapping_->tab_registers[i] = static_cast<uint16_t>(i);
  }
  for (int i = 0; i < NB_INPUT_REGISTERS; ++i) {
    mb_mapping_->tab_input_registers[i] = static_cast<uint16_t>(i);
  }

  server_socket_ = modbus_tcp_listen(ctx_, 1);
  if (server_socket_ < 0) {
    modbus_mapping_free(mb_mapping_);
    mb_mapping_ = nullptr;
    modbus_free(ctx_);
    ctx_ = nullptr;
    return false;
  }

  return true;
}

void ModbusTcpTestServerRunner::run(std::atomic<bool>& running) {
  if (ctx_ == nullptr || mb_mapping_ == nullptr) {
    return;
  }

  while (running.load()) {
    int client_socket = modbus_tcp_accept(ctx_, &server_socket_);
    if (client_socket < 0) {
      break;
    }

    while (running.load()) {
      uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
      int rc = modbus_receive(ctx_, query);

      if (rc > 0) {
        modbus_reply(ctx_, query, rc, mb_mapping_);
      } else if (rc == -1) {
        break;
      }
    }

    modbus_close(ctx_);
    modbus_free(ctx_);
    ctx_ = nullptr;
    if (!running.load()) {
      break;
    }
    ctx_ = modbus_new_tcp(nullptr, port_);
    if (!ctx_) {
      break;
    }
    modbus_set_slave(ctx_, slave_id_);
  }
}

void ModbusTcpTestServerRunner::close_listen_socket() {
  if (server_socket_ >= 0) {
    ::close(server_socket_);
    server_socket_ = -1;
  }
}

void ModbusTcpTestServerRunner::close() {
  close_listen_socket();
  if (mb_mapping_ != nullptr) {
    modbus_mapping_free(mb_mapping_);
    mb_mapping_ = nullptr;
  }
  if (ctx_ != nullptr) {
    modbus_free(ctx_);
    ctx_ = nullptr;
  }
}

void run_modbus_tcp_test_server(int port, int slave_id, std::atomic<bool>& running) {
  ModbusTcpTestServerRunner runner;
  if (!runner.open(port, slave_id)) {
    return;
  }
  runner.run(running);
  runner.close();
}

}  // namespace modbus_tcp_test_server
