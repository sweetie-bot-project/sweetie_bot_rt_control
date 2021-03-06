#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(sweetie_bot_robot_model)

#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

### c++11 support ###
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# Logger selection
set(LOGGER_DEFAULT "" CACHE STRING "Default logger for all packages (LoggerOCL, LoggerLog4Cpp, LoggerRosout, LoggerRTT).")
set(LOGGER_ROBOT_MODEL "" CACHE STRING "Logger for current package (LoggerOCL, LoggerLog4Cpp, LoggerRosout, LoggerRTT).")

if (LOGGER_ROBOT_MODEL)
        add_definitions(-DSWEETIEBOT_LOGGER=${LOGGER_ROBOT_MODEL})
elseif(LOGGER_DEFAULT)
        add_definitions(-DSWEETIEBOT_LOGGER=${LOGGER_DEFAULT})
else()
        add_definitions(-DSWEETIEBOT_LOGGER=LoggerRosout)
endif()

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
	rtt_ros
	kdl_parser
)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file
include_directories(SYSTEM ${USE_OROCOS_INCLUDE_DIRS})

### Orocos Targets ###

include_directories(include)

# Include src/ directory.
add_subdirectory( src )

### Orocos Package Exports and Install Targets ###

# Generate install targets for header files
orocos_install_headers(DIRECTORY include/${PROJECT_NAME}/)

# Export package information (replaces catkin_package() macro)
orocos_generate_package(
	INCLUDE_DIRS include
	DEPENDS kdl_parser orocos-kdl
	DEPENDS_TARGETS rtt_ros kdl_typekit sweetie_bot_logger
)

if (DEFINED _pkgconfig_error)
if (${_pkgconfig_error})
	message(FATAL_ERROR ${_pkgconfig_error})
endif()
else()
	message(FATAL_ERROR ${_pkgconfig_error})
endif()
