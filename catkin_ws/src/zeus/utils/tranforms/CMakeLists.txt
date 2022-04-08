cmake_minimum_required(VERSION 2.8.3)
project(transforms)
set(CMAKE_CXX_FLAGS "-std=c++1y ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPENDS} message_runtime
    LIBRARIES ${PROJECT_NAME}_lib
)
