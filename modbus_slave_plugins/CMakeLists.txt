cmake_minimum_required(VERSION 3.8)
project(modbus_slave_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(yaml-cpp REQUIRED)

# Shared types, utils, config loader and base interface impl
add_library(modbus_slave_common SHARED
  src/modbus_config_loader.cpp
  src/modbus_slave_interface.cpp
  src/modbus_utils.cpp
)
target_include_directories(modbus_slave_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(modbus_slave_common
  rclcpp
  realtime_tools
)
target_link_libraries(modbus_slave_common yaml-cpp)

# Plugin library
add_library(modbus_slave_plugins SHARED
  src/generic_modbus_slave.cpp
)
target_include_directories(modbus_slave_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(modbus_slave_plugins pluginlib rclcpp)
target_link_libraries(modbus_slave_plugins modbus_slave_common)

pluginlib_export_plugin_description_file(modbus_slave_plugins modbus_slave_plugins.xml)

install(TARGETS modbus_slave_common modbus_slave_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(modbus_slave_common modbus_slave_plugins)
ament_export_dependencies(pluginlib rclcpp realtime_tools)

ament_package()
