#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#


cmake_minimum_required(VERSION 2.6.3)

project(kinova_driver)
add_definitions(-std=c++11)

#================= Use the following for release version:
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -Wall")#Generates additional warning messages when compiling

#================= Use the following for Debuging:
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} -Wall")#Generates additional warning messages when compiling
# # set( CMAKE_VERBOSE_MAKEFILE on )
# set(CMAKE_BUILD_TYPE Debug)


# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' or 'source devel/setup.bash' in case of working with a ROS CATKIN package and then run cmake/catkin_make again.")
endif()
# find_package(catkin REQUIRED COMPONENTS rtt_ros roscpp)
find_package(catkin QUIET OPTIONAL_COMPONENTS rtt_ros roscpp)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# Rest of the configuration is in src/
add_subdirectory( src )

orocos_generate_package()
