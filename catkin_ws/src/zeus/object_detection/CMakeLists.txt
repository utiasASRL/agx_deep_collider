cmake_minimum_required(VERSION 2.8.3)
project(object_detection)
include("../config.cmake")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_geometry
  image_transport
  genmsg
  nav_msgs
  zeus_msgs
  visualization_msgs
  kalman
  zeus_pcl
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
CATKIN_DEPENDS kalman
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Cluster Publisher
add_library(cluster_publisher_node src/cluster_publisher/cluster_publisher_node.cpp)
target_link_libraries(cluster_publisher_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_executable(cluster_publisher src/cluster_publisher/cluster_publisher.cpp)
target_link_libraries(cluster_publisher
  cluster_publisher_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(cluster_publisher zeus_msgs_generate_messages_cpp)
# Secondary Clustering
add_library(secondary_clustering_node src/secondary_clustering/secondary_clustering_node.cpp)
target_link_libraries(secondary_clustering_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_executable(secondary_clustering src/secondary_clustering/secondary_clustering.cpp)
target_link_libraries(secondary_clustering
  secondary_clustering_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(secondary_clustering zeus_msgs_generate_messages_cpp)
# Object Tracker
add_library(object_tracker_node src/object_tracker/object_tracker_node.cpp)
target_link_libraries(object_tracker_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_executable(object_tracker src/object_tracker/object_tracker.cpp)
target_link_libraries(object_tracker
    object_tracker_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_dependencies(object_tracker zeus_msgs_generate_messages_cpp)
# Object Visualizer
add_library(object_visualizer_node src/object_visualizer/object_visualizer_node.cpp)
target_link_libraries(object_visualizer_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_executable(object_visualizer src/object_visualizer/object_visualizer.cpp)
target_link_libraries(object_visualizer
  object_visualizer_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(object_visualizer zeus_msgs_generate_messages_cpp)
# Occupancy Grid
add_library(occupancy_grid_node src/occupancy_grid/occupancy_grid_node.cpp)
target_link_libraries(occupancy_grid_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_executable(occupancy_grid src/occupancy_grid/occupancy_grid.cpp)
target_link_libraries(occupancy_grid
  occupancy_grid_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(occupancy_grid zeus_msgs_generate_messages_cpp)
