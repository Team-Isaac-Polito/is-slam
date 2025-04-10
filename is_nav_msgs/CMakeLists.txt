cmake_minimum_required(VERSION 3.8)
project(is_nav_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "srv/GetDistanceToObstacle.srv"
  "srv/GetRecoveryInfo.srv"
  "srv/GetRobotTrajectory.srv"
  "srv/GetSearchPosition.srv"
  "srv/GetNormal.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs builtin_interfaces
) 

ament_package()
