#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(sweetie_bot_resource_client_usage_example)

### c++11 support ###
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

### SweetieBot Logger configuration ###
set(LOGGER_DEFAULT "" CACHE STRING "Default logger for all packages (LoggerOCL, LoggerLog4Cpp, LoggerRosout, LoggerRTT).")
set(LOGGER_RESOURCE_CLIENT_USAGE_EXAMPLE "" CACHE STRING "Logger for ${PACKAGE_NAME} (LoggerOCL, LoggerLog4Cpp, LoggerRosout, LoggerRTT).")

if (LOGGER_RESOURCE_CLIENT_USAGE_EXAMPLE)
	add_definitions(-DSWEETIEBOT_LOGGER=${LOGGER_RESOURCE_CLIENT_USAGE_EXAMPLE})
elseif(LOGGER_DEFAULT)
	add_definitions(-DSWEETIEBOT_LOGGER=${LOGGER_DEFAULT})
else()
	add_definitions(-DSWEETIEBOT_LOGGER=LoggerRosout)
endif()

### General dependencies ###
#find_package(Boost REQUIRED COMPONENTS graph filesystem system thread regex)
find_package(Boost REQUIRED COMPONENTS thread)

### ROS Dependencies ###

# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS rtt_ros)
include_directories(${catkin_INCLUDE_DIRS})

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file
include_directories(${USE_OROCOS_INCLUDE_DIRS})

### Orocos Targets ###

# Include src/ directory.
add_subdirectory( src )

### Orocos Package Exports and Install Targets ###

# Scripts installation
install(DIRECTORY scripts
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# Launch installation
install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# python node installation
catkin_install_python(PROGRAMS test_client_actionlib.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Export package information (replaces catkin_package() macro)
orocos_generate_package(DEPENDS rtt_ros Boost)

if (DEFINED _pkgconfig_error)
if (${_pkgconfig_error})
	message(FATAL_ERROR ${_pkgconfig_error})
endif()
else()
	message(FATAL_ERROR ${_pkgconfig_error})
endif()
