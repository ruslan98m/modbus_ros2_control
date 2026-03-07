cmake_minimum_required(VERSION 3.8)
project(modbus_ros2_control)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(modbus_hw_interface REQUIRED)
  find_package(modbus_tcp_test_server REQUIRED)

  # Launch test: modbus TCP test server + modbus hw interface (controller_manager)
  add_launch_test(test/test_modbus_hw_with_server_launch_test.py TIMEOUT 30)
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
