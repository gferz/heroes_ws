cmake_minimum_required(VERSION 3.0.2)
project(omni4_odometry)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  tf2
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES omni4_odometry
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  nodes/odom_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)