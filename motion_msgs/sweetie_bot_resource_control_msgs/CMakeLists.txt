cmake_minimum_required(VERSION 2.8.3)
project(sweetie_bot_resource_control_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
  genmsg
  actionlib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ResourceRequest.msg
  ResourceAssignment.msg
  ResourceRequesterState.msg
  ControllerState.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
add_action_files(
  FILES
  MoveManually.action
  SetOperational.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

## catkin specific configuration ##
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sweetie_bot_resource_control_msgs
 CATKIN_DEPENDS message_runtime std_msgs
#  DEPENDS system_lib
)

