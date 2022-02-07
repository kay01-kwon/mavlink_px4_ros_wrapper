cmake_minimum_required(VERSION 2.8.3)
project(px4_gps_rtk)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES px4_gps_rtk
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(px4_test src/px4_test.cpp include/px4_gps_rtk/autopilot.cpp include/px4_gps_rtk/serial_port.cpp)
target_link_libraries(px4_test ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})


add_executable(thread_test src/thread_test.cpp)
target_link_libraries(thread_test ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})