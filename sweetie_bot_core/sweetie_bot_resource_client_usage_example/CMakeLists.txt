#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(sweetie_bot_resource_client_usage_example)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

find_package(catkin REQUIRED COMPONENTS
  sweetie_bot_resource_control_msgs
  sweetie_bot_resource_control
  actionlib
  rtt_actionlib
)

find_package(Boost REQUIRED COMPONENTS graph filesystem system thread regex)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Rest of the configuration is in src/
add_subdirectory( src )

#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package(DEPENDS Boost)
