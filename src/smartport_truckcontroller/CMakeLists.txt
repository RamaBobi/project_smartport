cmake_minimum_required(VERSION 3.8)
project(smartport_truckcontroller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ─────────────────────────────────────────────────────────────
find_package(ament_cmake       REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(std_msgs          REQUIRED)
find_package(sensor_msgs       REQUIRED)
find_package(nav_msgs          REQUIRED)
find_package(smartport_interfaces REQUIRED)

# ── Shared include path (contains ackermann.hpp) ─────────────────────────────
set(INCLUDE_DIR include)

# ─────────────────────────────────────────────────────────────────────────────
# 1. ackermann_controller_node
#    Custom Ackermann steering @ 1000 Hz
#    Sub: /truck/cmd_delta  /truck/vehicle_delta
#    Pub: /truck/fl_steer/cmd_pos  /truck/fr_steer/cmd_pos
# ─────────────────────────────────────────────────────────────────────────────
add_executable(ackermann_controller_node
  src/ackermann_controller_node.cpp)
target_include_directories(ackermann_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${INCLUDE_DIR}>)
target_compile_features(ackermann_controller_node PUBLIC cxx_std_17)
ament_target_dependencies(ackermann_controller_node
  rclcpp std_msgs)

# ─────────────────────────────────────────────────────────────────────────────
# 2. wheel_state_node
#    Reads /joint_states → publishes /truck/fl_steer_angle, /truck/fr_steer_angle,
#    /truck/vehicle_delta
# ─────────────────────────────────────────────────────────────────────────────
add_executable(wheel_state_node
  src/wheel_state_node.cpp)
target_include_directories(wheel_state_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${INCLUDE_DIR}>)
target_compile_features(wheel_state_node PUBLIC cxx_std_17)
ament_target_dependencies(wheel_state_node
  rclcpp std_msgs sensor_msgs)

# ─────────────────────────────────────────────────────────────────────────────
# 3. vehicle_state_node
#    Aggregates odometry + IMU + joint states + control inputs
#    Pub: /truck/vehicle_state (smartport_interfaces/VehicleState)
# ─────────────────────────────────────────────────────────────────────────────
add_executable(vehicle_state_node
  src/vehicle_state_node.cpp)
target_include_directories(vehicle_state_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${INCLUDE_DIR}>)
target_compile_features(vehicle_state_node PUBLIC cxx_std_17)
ament_target_dependencies(vehicle_state_node
  rclcpp std_msgs sensor_msgs nav_msgs smartport_interfaces)

# ── Install targets ───────────────────────────────────────────────────────────
install(TARGETS
  ackermann_controller_node
  wheel_state_node
  vehicle_state_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/keyboard_steer.py
  scripts/teleop_truck.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
