cmake_minimum_required(VERSION 3.8)
project(nammy_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/waypoint_follower.py
    scripts/waypoint_collector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
