#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(sweetie_bot_common)

### c++11 support ###
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++11)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS rtt_ros)
include_directories(include ${catkin_INCLUDE_DIRS})

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file
include_directories(${USE_OROCOS_INCLUDE_DIRS})

### Orocos Targets ###

### Orocos Package Exports and Install Targets ###

# Generate install targets for header files
orocos_install_headers(DIRECTORY include/${PROJECT_NAME}/)

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(INCLUDE_DIRS include DEPENDS rtt_ros)

# Scripts installation
install(DIRECTORY scripts
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
