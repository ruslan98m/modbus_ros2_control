cmake_minimum_required(VERSION 3.8)
project(modbus_hw_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

find_path(LIBMODBUS_INCLUDE_DIR modbus/modbus.h)
find_library(LIBMODBUS_LIBRARY modbus)
if(NOT LIBMODBUS_INCLUDE_DIR OR NOT LIBMODBUS_LIBRARY)
  message(FATAL_ERROR "libmodbus not found. Install libmodbus-dev (e.g. sudo apt install libmodbus-dev)")
endif()

find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${LIBMODBUS_INCLUDE_DIR}
)

add_library(modbus_hardware_interface SHARED
  src/modbus_config_loader.cpp
  src/modbus_hardware_interface.cpp
  src/modbus_io.cpp
  src/modbus_utils.cpp
)
target_include_directories(modbus_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(modbus_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
target_link_libraries(modbus_hardware_interface ${LIBMODBUS_LIBRARY} yaml-cpp)

pluginlib_export_plugin_description_file(hardware_interface modbus_hw_interface_plugins.xml)

install(TARGETS modbus_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_include_directories(include)
ament_export_libraries(modbus_hardware_interface)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools)

ament_package()
