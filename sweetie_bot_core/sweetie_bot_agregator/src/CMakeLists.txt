
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libsweetie_bot_agregator-<target>.so
# and installs in the directory lib/orocos/sweetie_bot_agregator/
#
orocos_component(sweetie_bot_agregator sweetie_bot_agregator-component.hpp sweetie_bot_agregator-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.
target_link_libraries(sweetie_bot_agregator ${catkin_LIBRARIES})

#
# Additional headers:
#
# Installs in the include/orocos/sweetie_bot_agregator/ directory
#
orocos_install_headers(sweetie_bot_agregator-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

