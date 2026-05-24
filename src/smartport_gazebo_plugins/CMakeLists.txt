cmake_minimum_required(VERSION 3.16)
project(smartport_gazebo_plugins)

find_package(ament_cmake REQUIRED)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-math7 REQUIRED)

add_library(pacejka_plugin SHARED src/PacejkaPlugin.cc)
target_link_libraries(pacejka_plugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
)

install(TARGETS pacejka_plugin
  DESTINATION lib
)

ament_package()
