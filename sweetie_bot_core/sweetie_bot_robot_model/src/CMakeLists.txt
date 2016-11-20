
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

#
# Building a Service:
#
# Creates a plugin library librobot_model-service-<target>.so
# and installs in the directory lib/orocos/sweetie_bot_robot_model/plugins/
#
orocos_service(robot_model-service robot_model-service.cpp) # ...only one service per library !
target_link_libraries(robot_model-service ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES})
orocos_install_headers( include/sweetie_bot_robot_model/robot_model-requester.hpp )

#
# You may add multiple orocos_service statements.


