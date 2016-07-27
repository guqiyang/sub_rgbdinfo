cmake_minimum_required(VERSION 2.8.3)
project(sub_rgbdinfo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs roscpp rospy
  genmsg cv_bridge
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aicroboxi_example
  CATKIN_DEPENDS geometry_msgs roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(obstacle nodes/obstacle.cpp)
target_link_libraries(obstacle ${catkin_LIBRARIES})
