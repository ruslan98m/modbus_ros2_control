// Copyright 2025 modbus_ros2_control contributors.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file modbus_tcp_test_server_node.cpp
 * @brief Lifecycle node: Modbus TCP test server for unit testing.
 */

#include <modbus_tcp_test_server/modbus_tcp_test_server_runner.hpp>

#include <memory>
#include <thread>

#include "rclcpp/executors.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace modbus_tcp_test_server {

constexpr int DEFAULT_PORT = 5502;
constexpr int DEFAULT_SLAVE_ID = 1;

class ModbusTcpTestServerNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit ModbusTcpTestServerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : LifecycleNode("modbus_tcp_test_server", options) {
    declare_parameter<int>("port", DEFAULT_PORT);
    declare_parameter<int>("slave_id", DEFAULT_SLAVE_ID);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &) override {
    port_ = get_parameter("port").as_int();
    slave_id_ = get_parameter("slave_id").as_int();
    RCLCPP_INFO(get_logger(), "Configured: port=%d, slave_id=%d", port_, slave_id_);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &) override {
    if (!runner_.open(port_, slave_id_)) {
      RCLCPP_ERROR(get_logger(), "Failed to open Modbus TCP test server on port %d", port_);
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    running_.store(true);
    server_thread_ = std::thread([this]() { runner_.run(running_); });
    RCLCPP_INFO(get_logger(), "Modbus TCP test server active on port %d, slave_id %d", port_,
                slave_id_);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &) override {
    running_.store(false);
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    runner_.close();
    RCLCPP_INFO(get_logger(), "Modbus TCP test server deactivated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State &) override {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State &) override {
    running_.store(false);
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    runner_.close();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  ~ModbusTcpTestServerNode() {
    // On SIGINT launch does not call on_deactivate/on_shutdown; ensure thread is stopped
    // before runner_ is destroyed (otherwise joinable thread -> std::terminate).
    running_.store(false);
    runner_.close_listen_socket();
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    runner_.close();
  }

 private:
  int port_{DEFAULT_PORT};
  int slave_id_{DEFAULT_SLAVE_ID};
  std::atomic<bool> running_{false};
  std::thread server_thread_;
  ModbusTcpTestServerRunner runner_;
};

}  // namespace modbus_tcp_test_server

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<modbus_tcp_test_server::ModbusTcpTestServerNode>();
  executor.add_node(node->get_node_base_interface());

  node->configure();
  node->activate();

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
