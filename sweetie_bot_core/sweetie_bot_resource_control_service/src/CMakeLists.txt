
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
# Creates a plugin library libsweetie_bot_resource_control_service-service-<target>.so
# and installs in the directory lib/orocos/sweetie_bot_resource_control_service/plugins/
#
orocos_service(sweetie_bot_resource_control_service sweetie_bot_resource_control_service.cpp) # ...only one service per library !

orocos_install_headers(include/sweetie_bot_resource_control_service/sweetie_bot_resource_control_service.hpp)

#
# You may add multiple orocos_service statements.


