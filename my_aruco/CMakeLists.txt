cmake_minimum_required(VERSION 2.8.3)
project(my_aruco)
SET(CMAKE_CXX_FLAGS "-std=c++11")
SET(OpenCV_DIR "/opt/ros/kinetic/share/OpenCV-3.3.1-dev")

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg  image_transport image_geometry cv_bridge tf2 tf)
find_package(Boost REQUIRED
  COMPONENTS  thread)


include_directories(${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}  ${catkin_INCLUDE_DIRS}  )
 
include_directories(include) 

catkin_package()

add_executable(my_aruco src/camera_position.cpp src/ippe.cpp  src/match.cpp src/my_aruco.cpp)
target_link_libraries(my_aruco ${OpenCV_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES} )

  



