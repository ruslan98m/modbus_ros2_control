cmake_minimum_required(VERSION 3.8)
project(modbus_hw_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(modbus_master REQUIRED)
find_package(modbus_slave_interface REQUIRED)
find_package(modbus_slave_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(modbus_hardware_interface SHARED
  src/modbus_hardware_interface.cpp
  src/master_params_parser.cpp
)
target_include_directories(modbus_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(modbus_hardware_interface
  hardware_interface
  modbus_master
  modbus_slave_interface
  modbus_slave_plugins
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(modbus_hardware_interface ${modbus_master_LIBRARIES} ${modbus_slave_interface_LIBRARIES} ${modbus_slave_plugins_LIBRARIES})

pluginlib_export_plugin_description_file(hardware_interface modbus_hw_interface_plugins.xml)

install(TARGETS modbus_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(modbus_hardware_interface)
ament_export_dependencies(hardware_interface modbus_master modbus_slave_interface modbus_slave_plugins pluginlib rclcpp rclcpp_lifecycle)

ament_package()
