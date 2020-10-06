cmake_minimum_required(VERSION 2.8.3)
project(xyi2_envi)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs

  tf
  sensor_msgs
  interactive_markers

  cv_bridge
  image_transport

  pcl_conversions
  pcl_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xyi2_envi
#  CATKIN_DEPENDS cv_bridge image_transport interactive_markers pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
