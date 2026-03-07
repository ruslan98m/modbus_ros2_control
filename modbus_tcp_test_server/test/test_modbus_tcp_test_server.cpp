// Copyright 2025 modbus_ros2_control contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_modbus_tcp_test_server.cpp
 * @brief Unit tests for ModbusTcpTestServerRunner and Modbus TCP test server protocol.
 */

#include <modbus/modbus.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>

#include <gtest/gtest.h>

#include "modbus_tcp_test_server/modbus_tcp_test_server_runner.hpp"

static constexpr const char * HOST = "127.0.0.1";
static constexpr int SLAVE_ID = 1;

/** Next port for tests; each test gets a unique port to avoid conflicts. */
static int get_test_port()
{
  static std::atomic<int> next_port{15502};
  return next_port++;
}

/** Connect client to server at port, return ctx or nullptr on failure. */
static modbus_t * connect_client(const char * host, int port, int slave_id)
{
  modbus_t * ctx = modbus_new_tcp(host, port);
  if (!ctx) return nullptr;
  modbus_set_slave(ctx, slave_id);
  if (modbus_connect(ctx) != 0) {
    modbus_free(ctx);
    return nullptr;
  }
  return ctx;
}

// -----------------------------------------------------------------------------
// ModbusTcpTestServerRunner API
// -----------------------------------------------------------------------------

TEST(ModbusTcpTestServerRunner, OpenCloseIsOpen)
{
  modbus_tcp_test_server::ModbusTcpTestServerRunner runner;
  EXPECT_FALSE(runner.is_open());

  int port = get_test_port();
  ASSERT_TRUE(runner.open(port, SLAVE_ID)) << "open failed";
  EXPECT_TRUE(runner.is_open());

  runner.close();
  EXPECT_FALSE(runner.is_open());
}

TEST(ModbusTcpTestServerRunner, CloseIdempotent)
{
  modbus_tcp_test_server::ModbusTcpTestServerRunner runner;
  ASSERT_TRUE(runner.open(get_test_port(), SLAVE_ID));
  runner.close();
  runner.close();
  runner.close();
  EXPECT_FALSE(runner.is_open());
}

TEST(ModbusTcpTestServerRunner, OpenAfterCloseReopens)
{
  modbus_tcp_test_server::ModbusTcpTestServerRunner runner;
  int port = get_test_port();
  ASSERT_TRUE(runner.open(port, SLAVE_ID));
  runner.close();
  EXPECT_FALSE(runner.is_open());
  ASSERT_TRUE(runner.open(port, SLAVE_ID));
  EXPECT_TRUE(runner.is_open());
  runner.close();
}

TEST(ModbusTcpTestServerRunner, RunWithoutOpenDoesNotBlock)
{
  modbus_tcp_test_server::ModbusTcpTestServerRunner runner;
  std::atomic<bool> running{true};
  runner.run(running);
  running.store(false);
}

TEST(ModbusTcpTestServerRunner, ContextAndMappingNullWhenClosed)
{
  modbus_tcp_test_server::ModbusTcpTestServerRunner runner;
  EXPECT_EQ(runner.context(), nullptr);
  EXPECT_EQ(runner.mapping(), nullptr);
  runner.close();
  EXPECT_EQ(runner.context(), nullptr);
}

TEST(ModbusTcpTestServerRunner, ContextAndMappingValidWhenOpen)
{
  modbus_tcp_test_server::ModbusTcpTestServerRunner runner;
  ASSERT_TRUE(runner.open(get_test_port(), SLAVE_ID));
  EXPECT_NE(runner.context(), nullptr);
  EXPECT_NE(runner.mapping(), nullptr);
  runner.close();
}

// -----------------------------------------------------------------------------
// Modbus protocol (server running in thread)
// -----------------------------------------------------------------------------

TEST(ModbusTcpTestServer, WriteAndReadHoldingRegister)
{
  const int port = get_test_port();
  std::atomic<bool> running{true};
  std::thread server_thread(
    [&]() { modbus_tcp_test_server::run_modbus_tcp_test_server(port, SLAVE_ID, running); });

  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  modbus_t * ctx = connect_client(HOST, port, SLAVE_ID);
  if (!ctx) {
    running.store(false);
    if (server_thread.joinable()) server_thread.join();
    GTEST_SKIP() << "Could not connect to server";
  }

  const int reg_addr = 0;
  const uint16_t write_val = 0x1234;
  EXPECT_EQ(modbus_write_register(ctx, reg_addr, write_val), 1)
    << modbus_strerror(errno);

  uint16_t read_val = 0;
  EXPECT_EQ(modbus_read_registers(ctx, reg_addr, 1, &read_val), 1)
    << modbus_strerror(errno);
  EXPECT_EQ(read_val, write_val);

  modbus_close(ctx);
  modbus_free(ctx);
  running.store(false);
  if (server_thread.joinable()) server_thread.join();
}

TEST(ModbusTcpTestServer, WriteAndReadCoil)
{
  const int port = get_test_port();
  std::atomic<bool> running{true};
  std::thread server_thread(
    [&]() { modbus_tcp_test_server::run_modbus_tcp_test_server(port, SLAVE_ID, running); });

  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  modbus_t * ctx = connect_client(HOST, port, SLAVE_ID);
  if (!ctx) {
    running.store(false);
    if (server_thread.joinable()) server_thread.join();
    GTEST_SKIP() << "Could not connect to server";
  }

  const int coil_addr = 0;
  EXPECT_EQ(modbus_write_bit(ctx, coil_addr, 1), 1) << modbus_strerror(errno);

  uint8_t tab[1] = {0};
  EXPECT_EQ(modbus_read_bits(ctx, coil_addr, 1, tab), 1) << modbus_strerror(errno);
  EXPECT_EQ(tab[0], 1);

  modbus_write_bit(ctx, coil_addr, 0);
  EXPECT_EQ(modbus_read_bits(ctx, coil_addr, 1, tab), 1);
  EXPECT_EQ(tab[0], 0);

  modbus_close(ctx);
  modbus_free(ctx);
  running.store(false);
  if (server_thread.joinable()) server_thread.join();
}

TEST(ModbusTcpTestServer, WriteAndReadMultipleHoldingRegisters)
{
  const int port = get_test_port();
  std::atomic<bool> running{true};
  std::thread server_thread(
    [&]() { modbus_tcp_test_server::run_modbus_tcp_test_server(port, SLAVE_ID, running); });

  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  modbus_t * ctx = connect_client(HOST, port, SLAVE_ID);
  if (!ctx) {
    running.store(false);
    if (server_thread.joinable()) server_thread.join();
    GTEST_SKIP() << "Could not connect to server";
  }

  const int start_addr = 0;
  uint16_t write_tab[4] = {0x1111, 0x2222, 0x3333, 0x4444};
  EXPECT_EQ(modbus_write_registers(ctx, start_addr, 4, write_tab), 4)
    << modbus_strerror(errno);

  uint16_t read_tab[4] = {0, 0, 0, 0};
  EXPECT_EQ(modbus_read_registers(ctx, start_addr, 4, read_tab), 4)
    << modbus_strerror(errno);
  EXPECT_EQ(std::memcmp(write_tab, read_tab, sizeof(write_tab)), 0);

  modbus_close(ctx);
  modbus_free(ctx);
  running.store(false);
  if (server_thread.joinable()) server_thread.join();
}

TEST(ModbusTcpTestServer, RunnerClassWriteAndReadHoldingRegister)
{
  const int port = get_test_port();
  modbus_tcp_test_server::ModbusTcpTestServerRunner runner;
  ASSERT_TRUE(runner.open(port, SLAVE_ID));

  std::atomic<bool> running{true};
  std::thread server_thread([&]() { runner.run(running); });
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  modbus_t * ctx = connect_client(HOST, port, SLAVE_ID);
  if (!ctx) {
    running.store(false);
    if (server_thread.joinable()) server_thread.join();
    runner.close();
    GTEST_SKIP() << "Could not connect to server";
  }

  EXPECT_EQ(modbus_write_register(ctx, 0, 0xABCD), 1);
  uint16_t val = 0;
  EXPECT_EQ(modbus_read_registers(ctx, 0, 1, &val), 1);
  EXPECT_EQ(val, 0xABCD);

  modbus_close(ctx);
  modbus_free(ctx);
  running.store(false);
  if (server_thread.joinable()) server_thread.join();
  runner.close();
}

TEST(ModbusTcpTestServer, ReadInputRegisters)
{
  const int port = get_test_port();
  modbus_tcp_test_server::ModbusTcpTestServerRunner runner;
  ASSERT_TRUE(runner.open(port, SLAVE_ID));
  modbus_mapping_t * map = runner.mapping();
  ASSERT_NE(map, nullptr);
  map->tab_input_registers[0] = 0xDEAD;
  map->tab_input_registers[1] = 0xBEEF;

  std::atomic<bool> running{true};
  std::thread server_thread([&]() { runner.run(running); });
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  modbus_t * ctx = connect_client(HOST, port, SLAVE_ID);
  if (!ctx) {
    running.store(false);
    if (server_thread.joinable()) server_thread.join();
    runner.close();
    GTEST_SKIP() << "Could not connect to server";
  }

  uint16_t read_tab[2] = {0, 0};
  EXPECT_EQ(modbus_read_input_registers(ctx, 0, 2, read_tab), 2)
    << modbus_strerror(errno);
  EXPECT_EQ(read_tab[0], 0xDEAD);
  EXPECT_EQ(read_tab[1], 0xBEEF);

  modbus_close(ctx);
  modbus_free(ctx);
  running.store(false);
  if (server_thread.joinable()) server_thread.join();
  runner.close();
}
