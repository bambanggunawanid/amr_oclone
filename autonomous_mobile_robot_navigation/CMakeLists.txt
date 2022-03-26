cmake_minimum_required(VERSION 3.0.2)
project(autonomous_mobile_robot_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  amcl
  actionlib
  actionlib_msgs
  geometry_msgs
  nav_msgs
  control_msgs
  controller_manager
  dynamic_reconfigure
  control_toolbox
  rosparam_shortcuts
  hardware_interface
  gmapping
  move_base
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
  tf2
  Eigen3
  moveit_ros_planning_interface
)

catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
)


add_library(i2c_ros src/i2c_ros.cpp)

add_executable(mobile_robot_hardware_interface src/robot_hardware_interface_node.cpp)
target_link_libraries(mobile_robot_hardware_interface i2c_ros ${catkin_LIBRARIES})