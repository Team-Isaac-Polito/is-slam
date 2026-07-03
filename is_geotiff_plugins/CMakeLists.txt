cmake_minimum_required(VERSION 3.8)
project(is_geotiff_plugins)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(is_geotiff REQUIRED)
find_package(is_nav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

###########
## Build ##
###########

add_library(is_geotiff_plugins src/trajectory_geotiff_plugin.cpp)
ament_target_dependencies(is_geotiff_plugins
  geometry_msgs
  is_geotiff
  is_nav_msgs
  nav_msgs
  pluginlib
  rclcpp
)

#############
## Install ##
#############

install(TARGETS is_geotiff_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  is_geotiff_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(is_geotiff is_nav_msgs nav_msgs pluginlib)

ament_package()
