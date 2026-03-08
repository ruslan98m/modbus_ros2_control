// Copyright 2025 modbus_ros2_control contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_tcp_test_server_runner.hpp
 * @brief Modbus TCP test server runner: owns libmodbus context and mapping for unit testing.
 */

#ifndef MODBUS_TCP_TEST_SERVER__MODBUS_TCP_TEST_SERVER_RUNNER_HPP_
#define MODBUS_TCP_TEST_SERVER__MODBUS_TCP_TEST_SERVER_RUNNER_HPP_

#include <modbus/modbus.h>

#include <atomic>
#include <cstdint>

namespace modbus_tcp_test_server
{

/**
 * @class ModbusTcpTestServerRunner
 * @brief Runs a Modbus TCP test server: owns the libmodbus context and register mapping.
 *
 * Typical use: call open() to bind and listen, then run() in a separate thread;
 * set the running flag to false and call close() to stop and release resources.
 * @see https://libmodbus.org/
 */
class ModbusTcpTestServerRunner
{
public:
  ModbusTcpTestServerRunner() = default;

  /** @brief Copy disabled (class owns modbus context and file descriptor). */
  ModbusTcpTestServerRunner(const ModbusTcpTestServerRunner &) = delete;
  ModbusTcpTestServerRunner & operator=(const ModbusTcpTestServerRunner &) = delete;

  /** @brief Destructor calls close() to free context and mapping. */
  ~ModbusTcpTestServerRunner() { close(); }

  /**
   * @brief Create Modbus TCP context, set slave ID and listen on the given port.
   * @param port TCP port to bind (e.g. 5502).
   * @param slave_id Modbus slave ID (e.g. 1).
   * @return true if context and mapping were created and listen succeeded, false on error.
   */
  bool open(int port, int slave_id);

  /**
   * @brief Run the accept/receive/reply loop in the current thread.
   * Call only after a successful open(). Exits when @p running becomes false.
   * @param running Set to false to stop the loop (e.g. from another thread).
   */
  void run(std::atomic<bool> & running);

  /**
   * @brief Close only the listening socket (e.g. to unblock accept() in run()).
   * Call before join() when stopping the server thread; then call close() to free resources.
   */
  void close_listen_socket();

  /**
   * @brief Close the server socket and free context and mapping.
   * Safe to call multiple times (idempotent).
   */
  void close();

  /**
   * @brief Check if the server is currently open.
   * @return true if open() succeeded and close() has not been called.
   */
  bool is_open() const { return ctx_ != nullptr; }

  /**
   * @brief Get the raw libmodbus context (e.g. for custom operations).
   * @return Pointer to the context, or nullptr if not open.
   */
  modbus_t * context() { return ctx_; }
  /** @brief Const overload of context(). */
  const modbus_t * context() const { return ctx_; }

  /**
   * @brief Get the modbus mapping (coils, discrete inputs, input/holding registers).
   * @return Pointer to the mapping, or nullptr if not open.
   */
  modbus_mapping_t * mapping() { return mb_mapping_; }
  /** @brief Const overload of mapping(). */
  const modbus_mapping_t * mapping() const { return mb_mapping_; }

private:
  modbus_t * ctx_{nullptr};           /**< libmodbus TCP context (recreated per client) */
  modbus_mapping_t * mb_mapping_{nullptr};  /**< Coils and registers mapping */
  int server_socket_{-1};             /**< Listening socket (or -1) */
  int port_{-1};                      /**< Port from open() for context recreation */
  int slave_id_{1};                   /**< Slave ID from open() for context recreation */

  /** Number of coils (0x) in the mapping */
  static constexpr int NB_COILS = 256;
  /** Number of discrete inputs (1x) in the mapping */
  static constexpr int NB_DISCRETE_INPUTS = 256;
  /** Number of holding registers (4x) in the mapping */
  static constexpr int NB_HOLDING_REGISTERS = 256;
  /** Number of input registers (3x) in the mapping */
  static constexpr int NB_INPUT_REGISTERS = 256;
};

/**
 * @brief Convenience function: run a Modbus TCP test server in the current thread.
 * Creates a temporary ModbusTcpTestServerRunner, calls open(), run(), then close().
 * @param port TCP port to listen on (e.g. 5502).
 * @param slave_id Modbus slave ID (e.g. 1).
 * @param running Set to false to stop the server loop.
 */
void run_modbus_tcp_test_server(int port, int slave_id, std::atomic<bool> & running);

}  // namespace modbus_tcp_test_server

#endif  // MODBUS_TCP_TEST_SERVER__MODBUS_TCP_TEST_SERVER_RUNNER_HPP_
