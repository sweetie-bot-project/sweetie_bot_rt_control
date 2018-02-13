cmake_minimum_required(VERSION 2.8.3)
project(rtt_sweetie_bot_servo_model_msg)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

ros_generate_rtt_typekit(sweetie_bot_servo_model_msg)
ros_generate_rtt_service_proxies(sweetie_bot_servo_model_msg)

orocos_generate_package(
  DEPENDS sweetie_bot_servo_model_msg
  DEPENDS_TARGETS rtt_roscomm
)
