cmake_minimum_required(VERSION 3.8)
project(hector_geotiff)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(hector_map_tools REQUIRED)
find_package(hector_nav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

# 1. Library geotiff_writer
add_library(geotiff_writer src/geotiff_writer/geotiff_writer.cpp)
target_include_directories(geotiff_writer PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${EIGEN3_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)
target_link_libraries(geotiff_writer Qt5::Widgets stdc++fs)
ament_target_dependencies(geotiff_writer
  hector_map_tools
  hector_nav_msgs
  nav_msgs
  pluginlib
  rclcpp
  std_msgs
)


# 2. Executable geotiff_saver
add_executable(geotiff_saver src/geotiff_saver.cpp)
target_link_libraries(geotiff_saver geotiff_writer)

add_executable(geotiff_node src/geotiff_node.cpp)
target_link_libraries(geotiff_node geotiff_writer)

#############
## Install ##
#############

# 1. Library geotiff_writer
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS geotiff_writer
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 2. Executable geotiff_saver and geotiff_node
install(TARGETS geotiff_saver geotiff_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY fonts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
