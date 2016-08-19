
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libsweetie_bot_kinematics-<target>.so
# and installs in the directory lib/orocos/sweetie_bot_kinematics/
#
orocos_component(sweetie_bot_kinematics sweetie_bot_kinematics-component.hpp sweetie_bot_kinematics-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.
#target_link_libraries(sweetie_bot_kinematics ${orocos_kdl_LIBRARIES} ${trac_ik_lib_LIBRARIES})
target_link_libraries(sweetie_bot_kinematics ${catkin_LIBRARIES})

#
# Additional headers:
#
# Installs in the include/orocos/sweetie_bot_kinematics/ directory
#
orocos_install_headers(sweetie_bot_kinematics-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

