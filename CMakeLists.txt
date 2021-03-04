cmake_minimum_required(VERSION 3.10)
project(forklift_webots_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Webots
link_directories($ENV{WEBOTS_HOME}/lib/controller)
set(
  WEBOTS_LIB
  ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
  ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
)


# ROS2 Packages
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(forklift_robot_description REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(webots_ros2_msgs REQUIRED)

include_directories(
  include
  $ENV{WEBOTS_HOME}/include/controller/c
  $ENV{WEBOTS_HOME}/include/controller/cpp
)

set(${PROJECT_NAME}_dependencies
  action_msgs  
  forklift_robot_description
  nav2_bt_navigator
  nav2_map_server
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  tf2_ros
  vision_msgs
  webots_ros2_msgs
)

#############
##  Build  ##
#############


#############
## Install ##
#############

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME} 
)
ament_package()