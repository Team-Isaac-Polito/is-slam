cmake_minimum_required(VERSION 3.8)
project(hector_map_tools)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.h"
)

ament_export_dependencies(nav_msgs Eigen3)
ament_export_include_directories(include)

ament_package()