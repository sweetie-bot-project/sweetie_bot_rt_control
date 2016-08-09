
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library liburdf_need-<target>.so
# and installs in the directory lib/orocos/urdf_need/
#
orocos_component(urdf_need urdf_need-component.hpp urdf_need-component.cpp) # ...you may add multiple source files
target_link_libraries(urdf_need ${orocos_kdl_LIBRARIES})

#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/urdf_need/ directory
#
orocos_install_headers(urdf_need-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

