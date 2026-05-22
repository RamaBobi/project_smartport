cmake_minimum_required(VERSION 3.8)
project(smartport_control_lower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(smartport_interfaces REQUIRED)

add_executable(ackermann_bridge src/ackermann_bridge.cpp)
ament_target_dependencies(ackermann_bridge rclcpp smartport_interfaces)

install(TARGETS
  ackermann_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
