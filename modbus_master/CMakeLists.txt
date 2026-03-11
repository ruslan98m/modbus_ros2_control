cmake_minimum_required(VERSION 3.8)
project(modbus_master)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(modbus_slave_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)

find_path(LIBMODBUS_INCLUDE_DIR modbus/modbus.h)
find_library(LIBMODBUS_LIBRARY modbus)
if(NOT LIBMODBUS_INCLUDE_DIR OR NOT LIBMODBUS_LIBRARY)
  message(FATAL_ERROR "libmodbus not found. Install libmodbus-dev (e.g. sudo apt install libmodbus-dev)")
endif()

add_library(modbus_master SHARED
  src/modbus_master.cpp
  src/modbus_master_helpers.cpp
)
target_include_directories(modbus_master PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBMODBUS_INCLUDE_DIR}
)
ament_target_dependencies(modbus_master modbus_slave_interface rclcpp realtime_tools)
target_link_libraries(modbus_master ${LIBMODBUS_LIBRARY} ${modbus_slave_interface_LIBRARIES})

install(TARGETS modbus_master
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(modbus_master)
ament_export_dependencies(modbus_slave_interface rclcpp realtime_tools)

ament_package()
