cmake_minimum_required(VERSION 2.8.3)
project(rtt_sweetie_bot_resource_control_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

ros_generate_rtt_typekit(sweetie_bot_resource_control_msgs)
ros_generate_rtt_service_proxies(sweetie_bot_resource_control_msgs)

orocos_generate_package(
  DEPENDS sweetie_bot_resource_control_msgs
  DEPENDS_TARGETS rtt_roscomm  rtt_std_msgs

)
