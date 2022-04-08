cmake_minimum_required(VERSION 2.8.3)
project(zeus_pcl)
include("../config.cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_geometry
  genmsg
  nav_msgs
  zeus_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  roslint
)

set(ROSLINT_CPP_OPTS "--filter=-legal, -build/c++11, -whitespace/indent, -whitespace/braces, -runtime/references, -readability/casting")
roslint_cpp()

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS
    system
    filesystem
)

find_package(PkgConfig)

catkin_package(
INCLUDE_DIRS include
LIBRARIES zeus_pcl
LIBRARIES zeus_vis
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(zeus_pcl src/utils/zeus_pcl.cpp)
target_link_libraries(zeus_pcl
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_library(zeus_vis src/utils/zeus_vis.cpp)
target_link_libraries(zeus_vis
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_dependencies(zeus_pcl zeus_msgs_generate_messages_cpp)
add_dependencies(zeus_vis zeus_msgs_generate_messages_cpp)
