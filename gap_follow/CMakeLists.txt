cmake_minimum_required(VERSION 3.10)
project(gap_follow)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  reactive_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
