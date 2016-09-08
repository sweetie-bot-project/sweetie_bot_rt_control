
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libsweetie_bot_gait_walking_3_plus_1-<target>.so
# and installs in the directory lib/orocos/sweetie_bot_gait_walking_3_plus_1/
#
orocos_component(sweetie_bot_gait_walking_3_plus_1 sweetie_bot_gait_walking_3_plus_1-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/sweetie_bot_gait_walking_3_plus_1/ directory
#
orocos_install_headers(sweetie_bot_gait_walking_3_plus_1-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

