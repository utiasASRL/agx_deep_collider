###########################################################################
# IMPORTANT: This file is only used by CMake. Catkin builds will ignore it.
#            Do not put anything in here that's project related.
###########################################################################

project(zeus_top)
cmake_minimum_required(VERSION 3.1)
set(IS_CMAKE_BUILD TRUE)
include(config.cmake)
include_directories(../../devel/include)

option(ZEUS_BUILD_TESTS "Determines whether to build tests." ON)
if(ZEUS_BUILD_TESTS)
    enable_testing()
endif()

set(USE_ZEUS_MESSAGES ON CACHE BOOL "Determines whether to use zeus_msgs for DLA nodes" FORCE)

include_directories(${CMAKE_BINARY_DIR}/devel/include)
add_subdirectory(zeus_pcl)
add_subdirectory(kalman)
add_subdirectory(object_detection)
