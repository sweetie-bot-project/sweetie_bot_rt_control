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

### c++11 support ###
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS rtt_ros kdl_msgs sensor_msgs geometry_msgs sweetie_bot_kinematics_msgs)
include_directories(${catkin_INCLUDE_DIRS})

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file
orocos_use_package(orocos_kdl REQUIRED)
include_directories(${USE_OROCOS_INCLUDE_DIRS})

include_directories(
	include
	include/orocos # necessary to reference headers in this package as <sweetie_bot_orocos_misc/header.hpp>
)


### Orocos Targets ###

# Libraries: only .so modules from root CMakeLists are added to *.pc file.
orocos_library(sweetie_bot_orocos_misc src/message_checks.cpp src/math.cpp)
add_dependencies(sweetie_bot_orocos_misc ${catkin_EXPORTED_TARGETS})
# Include src/ directory.
add_subdirectory( src )

### Orocos Package Exports and Install Targets ###

# Generate install targets for header files
orocos_install_headers(DIRECTORY include/orocos/${PROJECT_NAME}/) # will install headers to include/orocos/sweetie_bot_orocos_misc

# Export package information (replaces catkin_package() macro)
orocos_generate_package(
	INCLUDE_DIRS include/orocos  # include path for OROCOS packages is <sweetie_bot_orocos_misc/header.h>
	DEPENDS kdl_msgs sensor_msgs geometry_msgs sweetie_bot_kinematics_msgs orocos_kdl
	DEPENDS_TARGETS rtt_ros rtt_actionlib
)
# force buid system to export include path for pure ROS packages
catkin_package(INCLUDE_DIRS include) # include path for ROS packages is <orocos/sweetie_bot_orocos_misc/header.h>

if (DEFINED _pkgconfig_error)
if (${_pkgconfig_error})
	message(FATAL_ERROR ${_pkgconfig_error})
endif()
else()
	message(FATAL_ERROR ${_pkgconfig_error})
endif()
