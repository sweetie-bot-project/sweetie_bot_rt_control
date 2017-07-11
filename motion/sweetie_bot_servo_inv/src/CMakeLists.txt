
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libsweetie_bot_servo_inv-<target>.so
# and installs in the directory lib/orocos/sweetie_bot_servo_inv/
#
orocos_component(servo_inv_lead servo_inv_lead.hpp servo_inv_lead.cpp) # ...you may add multiple source files
orocos_component(servo_inv_7param servo_inv_7param.hpp servo_inv_7param.cpp) # ...you may add multiple source files
orocos_component(player_joint_state player_joint_state.hpp player_joint_state.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/sweetie_bot_servo_inv/ directory
#
orocos_install_headers() # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

