cmake_minimum_required(VERSION 2.8.3)
project(xyi2_brain)

# support g2o lib.
# SET(CMAKE_CXX_FLAGS "-std=c++0x")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf

  image_transport
  cv_bridge

  pcl_conversions
  pcl_ros

  interactive_markers
  sensor_msgs
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

add_executable(	xyi2_brain_main

                src/main.cpp

                src/thread_manage/thread_manage.cpp

                src/class_tool/param_robot.cpp
                src/class_tool/slamdata.cpp
                src/class_tool/tfm.cpp
                src/class_tool/tfm_slam.cpp
                src/class_tool/hfunc.cpp

                src/map/mfunc_mapexisting.cpp
                src/map/struct_map.cpp

                src/sensors/model_laser.cpp

                src/matchings/struct_localmapping_chart.cpp
                src/matchings/struct_lsr_filter.cpp
                src/matchings/func_lsr_mapping.cpp
                src/matchings/search_window_strategy.cpp
                src/matchings/search_window_portfunc.cpp
                src/matchings/al_match.cpp

                src/navigation/lmap/localmap.cpp
                src/navigation/gpath_plan/asearch.cpp
                src/navigation/lpath_plan/dwindow_chart.cpp
                src/navigation/lpath_plan/dwindow.cpp
                src/navigation/rcontrol/r_controller.cpp
                src/navigation/navi_func.cpp
                src/navigation/navi_manage.cpp

                src/graph_base/unit_kdtree.cpp
                src/graph_base/unit_table.cpp
                src/graph_base/unit_edge.cpp
                src/graph_base/unit_node.cpp
                src/graph_base/slam_procedure.cpp
                src/graph_base/struct_graph.cpp
                src/graph_base/graph_func.cpp
                src/graph_base//graph_io/graph_io.cpp

                src/com_bus/scom/scom_bsp.cpp
                src/com_bus/scom/scom_function.cpp
                src/com_bus/scom/scom_manage.cpp
)

target_link_libraries(xyi2_brain_main ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_dependencies(xyi2_brain_main ${catkin_EXPORTED_TARGETS})

