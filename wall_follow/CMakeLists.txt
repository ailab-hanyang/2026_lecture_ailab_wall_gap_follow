cmake_minimum_required(VERSION 3.10)
project(wall_follow)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  wall_follow_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
