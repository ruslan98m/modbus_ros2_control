cmake_minimum_required(VERSION 3.8)
project(modbus_tcp_test_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(LIBMODBUS QUIET libmodbus)
endif()
if(NOT LIBMODBUS_FOUND)
  find_path(LIBMODBUS_INCLUDE_DIR modbus/modbus.h)
  find_library(LIBMODBUS_LIBRARY modbus)
  if(LIBMODBUS_INCLUDE_DIR AND LIBMODBUS_LIBRARY)
    set(LIBMODBUS_FOUND TRUE)
    set(LIBMODBUS_INCLUDE_DIRS ${LIBMODBUS_INCLUDE_DIR})
    set(LIBMODBUS_LIBRARIES ${LIBMODBUS_LIBRARY})
  endif()
endif()
if(NOT LIBMODBUS_FOUND)
  message(FATAL_ERROR "libmodbus not found. Install libmodbus-dev")
endif()

include_directories(include ${LIBMODBUS_INCLUDE_DIRS})

add_library(modbus_tcp_test_server_runner src/modbus_tcp_test_server_runner.cpp)
target_link_libraries(modbus_tcp_test_server_runner ${LIBMODBUS_LIBRARIES})

add_executable(modbus_tcp_test_server_node src/modbus_tcp_test_server_node.cpp)
ament_target_dependencies(modbus_tcp_test_server_node rclcpp rclcpp_lifecycle)
target_link_libraries(modbus_tcp_test_server_node modbus_tcp_test_server_runner)

install(TARGETS modbus_tcp_test_server_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS modbus_tcp_test_server_runner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_modbus_tcp_test_server test/test_modbus_tcp_test_server.cpp)
  if(TARGET test_modbus_tcp_test_server)
    target_include_directories(test_modbus_tcp_test_server PRIVATE include ${LIBMODBUS_INCLUDE_DIRS})
    target_link_libraries(test_modbus_tcp_test_server modbus_tcp_test_server_runner ${LIBMODBUS_LIBRARIES})
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(modbus_tcp_test_server_runner)
ament_package()
