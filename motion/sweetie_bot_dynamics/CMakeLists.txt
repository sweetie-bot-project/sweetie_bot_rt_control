#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(sweetie_bot_dynamics)

### c++11 support ###
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Logger selection
set(LOGGER_DEFAULT "" CACHE STRING "Default logger for all packages (LoggerOCL, LoggerLog4Cpp, LoggerRosout, LoggerRTT).")
set(LOGGER_DYNAMICS "" CACHE STRING "Logger for current package (LoggerOCL, LoggerLog4Cpp, LoggerRosout, LoggerRTT).")

if (LOGGER_KINEMATICS)
        add_definitions(-DSWEETIEBOT_LOGGER=${LOGGER_DYNAMICS})
elseif(LOGGER_DEFAULT)
        add_definitions(-DSWEETIEBOT_LOGGER=${LOGGER_DEFAULT})
else()
        add_definitions(-DSWEETIEBOT_LOGGER=LoggerRosout)
endif()

### General Dependencies ###
find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_check_modules(RBDL REQUIRED rbdl>=2.6)
find_library(RBDL_URDFREADER_LIBRARY NAMES rbdl_urdfreader PATHS ${RBDL_LIBRARY_DIRS})
if (RBDL_URDFREADER_LIBRARY-NOTFOUND)
	message(FATAL_ERROR "Unable to find rbdl_urdfreader library.")
endif()
include_directories(SYSTEM ${RBDL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS rtt_ros roscpp tf eigen_conversions kdl_conversions tf_conversions sensor_msgs sweetie_bot_kinematics_msgs)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file
orocos_use_package(orocos_kdl REQUIRED)
include_directories(SYSTEM ${USE_OROCOS_INCLUDE_DIRS})

### Orocos Targets ###

# Include src/ directory.
add_subdirectory( src )

### Orocos Package Exports and Install Targets ###

## Scripts installation
#install(DIRECTORY scripts
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# Launch installation
#install(DIRECTORY launch
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Export package information (replaces catkin_package() macro)
orocos_generate_package(DEPENDS rtt_ros)

if (DEFINED _pkgconfig_error)
if (${_pkgconfig_error})
	message(FATAL_ERROR ${_pkgconfig_error})
endif()
else()
	message(FATAL_ERROR ${_pkgconfig_error})
endif()
