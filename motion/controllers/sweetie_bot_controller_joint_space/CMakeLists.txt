#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(sweetie_bot_controller_joint_space)

### c++11 support ###
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

### SweetieBot Logger configuration ###
set(LOGGER_DEFAULT "" CACHE STRING "Default logger for all packages (LoggerOCL, LoggerLog4Cpp, LoggerRosout, LoggerRTT).")
set(LOGGER_CONTROLLER_JOINT_SPACE "" CACHE STRING "Logger for ${PROJECT_NAME} (LoggerOCL, LoggerLog4Cpp, LoggerRosout, LoggerRTT).")

if (LOGGER_CONTROLLER_JOINT_SPACE)
	add_definitions(-DSWEETIEBOT_LOGGER=${LOGGER_CONTROLLER_JOINT_SPACE})
elseif(LOGGER_DEFAULT)
	add_definitions(-DSWEETIEBOT_LOGGER=${LOGGER_DEFAULT})
else()
	add_definitions(-DSWEETIEBOT_LOGGER=LoggerRosout)
endif()

### General dependencies ###
find_package(Eigen3 REQUIRED)
include_directories("${EIGEN3_INCLUDE_DIR}")

find_path(alglib_INCLUDE_DIR interpolation.h PATH_SUFFIXES libalglib alglib)
find_library(alglib_LIBRARY NAMES alglib)
if (NOT (alglib_INCLUDE_DIR AND alglib_LIBRARY))
	message (FATAL_ERROR "\nCould not find alglib library.")
endif()
include_directories(${alglib_INCLUDE_DIR})

### General Dependencies ###
find_package(Boost REQUIRED COMPONENTS thread) # Fix for xenial linking problems

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS rtt_ros)
include_directories(${catkin_INCLUDE_DIRS})

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file
include_directories(${USE_OROCOS_INCLUDE_DIRS})

### Orocos Targets ###

include_directories(include)

# Include src/ directory.
add_subdirectory( src )

### Orocos Package Exports and Install Targets ###

#Install headers: OROCOS automatically place them in `orocos/package_name/` directory.
orocos_install_headers(include/sweetie_bot_controller_joint_space/filter_joint_state.hpp)

# Export package information (replaces catkin_package() macro)
orocos_generate_package(INCLUDE_DIRS include)

if (DEFINED _pkgconfig_error)
if (${_pkgconfig_error})
	message(FATAL_ERROR ${_pkgconfig_error})
endif()
else()
	message(FATAL_ERROR ${_pkgconfig_error})
endif()
