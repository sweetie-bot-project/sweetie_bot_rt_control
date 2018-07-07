#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(sweetie_bot_controller_cartesian)

### c++11 support ###
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

### SweetieBot Logger configuration ###
set(LOGGER_DEFAULT "" CACHE STRING "Default logger for all packages (LoggerOCL, LoggerLog4Cpp, LoggerRosout, LoggerRTT).")
set(LOGGER_CONTROLLER_JOINT_SPACE "" CACHE STRING "Logger for ${PROJECT_NAME} (LoggerOCL, LoggerLog4Cpp, LoggerRosout, LoggerRTT).")

if (LOGGER_CONTROLLER_JOINT_SPACE)
	add_definitions(-DSWEETIEBOT_LOGGER=${LOGGER_CONTROLLER_CARTESIAN})
elseif(LOGGER_DEFAULT)
	add_definitions(-DSWEETIEBOT_LOGGER=${LOGGER_DEFAULT})
else()
	add_definitions(-DSWEETIEBOT_LOGGER=LoggerRTT)
endif()

### General dependencies ###
#find_package(Boost REQUIRED COMPONENTS graph filesystem system thread regex)
find_package(Boost REQUIRED COMPONENTS thread) # Fix for xenial linking problems with Boost

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS rtt_ros kdl_conversions)
include_directories(include ${catkin_INCLUDE_DIRS})

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file
orocos_use_package(orocos_kdl REQUIRED)
include_directories(${USE_OROCOS_INCLUDE_DIRS})

### Orocos Targets ###

# Include src/ directory.
add_subdirectory( src )

### Orocos Package Exports and Install Targets ###

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(INCLUDE_DIRS include)

if (DEFINED _pkgconfig_error)
if (${_pkgconfig_error})
	message(FATAL_ERROR ${_pkgconfig_error})
endif()
else()
	message(FATAL_ERROR ${_pkgconfig_error})
endif()
