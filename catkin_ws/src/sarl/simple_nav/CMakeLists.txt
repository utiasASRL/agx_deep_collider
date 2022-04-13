cmake_minimum_required(VERSION 2.8.3)
project(crowd_planner)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  zeus_msgs
  geometry_msgs
  nav_msgs
)

#find_package(OpenCV REQUIRED)
