cmake_minimum_required(VERSION 3.0.2)
project(is_geotiff_plugins)

find_package(catkin REQUIRED COMPONENTS is_geotiff is_nav_msgs)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS is_geotiff is_nav_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(is_geotiff_plugins src/trajectory_geotiff_plugin.cpp)
add_dependencies(is_geotiff_plugins ${catkin_EXPORTED_TARGETS})
target_link_libraries(is_geotiff_plugins
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS is_geotiff_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  is_geotiff_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
