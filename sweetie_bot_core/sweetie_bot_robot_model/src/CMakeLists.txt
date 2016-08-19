
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
# Creates a plugin library libsweetie_bot_robot_model-service-<target>.so
# and installs in the directory lib/orocos/sweetie_bot_robot_model/plugins/
#
orocos_service(sweetie_bot_robot_model-service sweetie_bot_robot_model-service.cpp) # ...only one service per library !
target_link_libraries(sweetie_bot_robot_model-service ${orocos_kdl_LIBRARIES} ${kdl_parser_LIBRARIES})
orocos_install_headers( inclide/sweetie_bot_robot_model/sweetie_bot_robot_model-requester.hpp )

#
# You may add multiple orocos_service statements.


