
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libsweetie_bot_resource_control-<target>.so
# and installs in the directory lib/orocos/sweetie_bot_resource_control/
#
orocos_component(resource_arbiter resource_arbiter-component.cpp) # ...you may add multiple source files

orocos_service(resource_client_service resource_client-service.cpp) # only one service per library!
orocos_service(resource_client_dummy_service resource_client_dummy-service.cpp) # only one service per library!
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/sweetie_bot_resource_control/ directory
#
# orocos_install_headers(sweetie_bot_resource_control-component.hpp) # ...you may add multiple header files
# orocos_install_headers(include/sweetie_bot_resource_control/sweetie_bot_resource_control_service.hpp)
#
# You may add multiple orocos_install_headers statements.
