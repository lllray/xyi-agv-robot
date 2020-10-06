cmake_minimum_required(VERSION 2.8.3)
project(xyi2_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcloud_matching
#  CATKIN_DEPENDS cv_bridge image_transport interactive_markers pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(	xyi2_test

                src/main.cpp
)

target_link_libraries(xyi2_test ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_dependencies(xyi2_test ${catkin_EXPORTED_TARGETS})
