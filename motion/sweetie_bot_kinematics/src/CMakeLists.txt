
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libkinematics-<target>.so
# and installs in the directory lib/orocos/sweetie_bot_kinematics/
#
#orocos_component(kinematics kinematics-component.hpp kinematics-component.cpp) # ...you may add multiple source files
#target_link_libraries(kinematics ${catkin_LIBRARIES})

orocos_component(kinematics_fwd kinematics_fwd-component.hpp kinematics_fwd-component.cpp) # ...you may add multiple source files
#
# Additional headers:
#
# Installs in the include/orocos/sweetie_bot_kinematics/ directory
#
#orocos_install_headers(kinematics-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

