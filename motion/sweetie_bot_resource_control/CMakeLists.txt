#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(sweetie_bot_resource_control)

### c++11 support ###
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

### SweetieBot Logger configuration ###
set(LOGGER_DEFAULT "" CACHE STRING "Default logger for all packages (LoggerOCL, LoggerLog4Cpp, LoggerRosout, LoggerRTT).")
set(LOGGER_RESOURCE_CONTROL "" CACHE STRING "Logger for herkulex package (LoggerOCL, LoggerLog4Cpp, LoggerRosout, LoggerRTT).")

if (LOGGER_RESOURCE_CONTROL)
	add_definitions(-DSWEETIEBOT_LOGGER=${LOGGER_RESOURCE_CONTROL})
elseif(LOGGER_DEFAULT)
	add_definitions(-DSWEETIEBOT_LOGGER=${LOGGER_DEFAULT})
else()
	add_definitions(-DSWEETIEBOT_LOGGER=LoggerRTT)
endif()

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS rtt_ros)
include_directories(include ${catkin_INCLUDE_DIRS})

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file
include_directories(${USE_OROCOS_INCLUDE_DIRS})

# find_package(Boost REQUIRED COMPONENTS thread)

### Orocos Targets ###

# Include src/ directory.
add_subdirectory( src )

# Libraries: only .so modules from root CMakeLists are added to *.pc file.
orocos_library(actionlib_controller_base src/actionlib_controller_base.cpp)
set_target_properties(actionlib_controller_base PROPERTIES COMPILE_FLAGS "-DRTT_STATIC")
target_link_libraries(actionlib_controller_base ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Package Exports and Install Targets ###

# Generate install targets for header files
orocos_install_headers(DIRECTORY include/${PROJECT_NAME}/)

# Export package information (replaces catkin_package() macro)
orocos_generate_package(
	INCLUDE_DIRS include
	DEPENDS_TARGETS rtt_ros sweetie_bot_logger sweetie_bot_orocos_misc rtt_sweetie_bot_resource_control_msgs rtt_std_srvs
)

if (DEFINED _pkgconfig_error)
if (${_pkgconfig_error})
	message(FATAL_ERROR ${_pkgconfig_error})
endif()
else()
	message(FATAL_ERROR ${_pkgconfig_error})
endif()
