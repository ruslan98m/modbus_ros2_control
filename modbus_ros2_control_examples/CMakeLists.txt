cmake_minimum_required(VERSION 3.8)
project(modbus_ros2_control_examples)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
