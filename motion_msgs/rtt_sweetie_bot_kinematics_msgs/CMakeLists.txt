cmake_minimum_required(VERSION 2.8.3)
project(rtt_sweetie_bot_kinematics_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

### c++11 support ###
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

ros_generate_rtt_typekit(sweetie_bot_kinematics_msgs)
ros_generate_rtt_service_proxies(sweetie_bot_kinematics_msgs)

orocos_generate_package(
  DEPENDS sweetie_bot_kinematics_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs rtt_geometry_msgs rtt_kdl_msgs
)
