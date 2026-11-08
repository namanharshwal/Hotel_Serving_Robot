cmake_minimum_required(VERSION 3.8)
project(nammy_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/waypoint_collector.py
    scripts/waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
