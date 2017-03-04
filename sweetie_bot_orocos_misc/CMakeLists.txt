#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(sweetie_bot_orocos_misc)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

#Catkin support
find_package(catkin REQUIRED)
catkin_package(CATKIN_DEPENDS actionlib rtt_actionlib INCLUDE_DIRS include)
include_directories(include ${catkin_INCLUDE_DIRS})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

#Install headers: OROCOS automatically place them in `orocos/package_name/` directory.
orocos_install_headers(include/sweetie_bot_orocos_misc/simple_action_server.hpp include/sweetie_bot_orocos_misc/get_subservice_by_type.hpp include/sweetie_bot_orocos_misc/joint_state_check.hpp)

#Libraries: only .so modules from root CMakeLists are added to *.pc file.
orocos_library(sweetie_bot_orocos_misc src/joint_state_check.cpp) # ...you may add multiple source files

# Rest of the configuration is in src/
add_subdirectory( src )
#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
