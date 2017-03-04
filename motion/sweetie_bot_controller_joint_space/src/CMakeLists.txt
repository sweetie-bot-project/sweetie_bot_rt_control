
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libsweetie_bot_resource_client_usage_example-<target>.so
# and installs in the directory lib/orocos/sweetie_bot_resource_client_usage_example/
#
orocos_component(follow_joint_state follow_joint_state-component.cpp) # ...you may add multiple source files
#target_link_libraries(follow_joint_state ${catkin_LIBRARIES})
orocos_component(animation_joint_trajectory animation_joint_trajectory-component.cpp animation_joint_trajectory-base.cpp joint_trajectory_cache.cpp) # ...you may add multiple source files
target_link_libraries(animation_joint_trajectory ${alglib_LIBRARY})
#target_link_libraries(animation_joint_trajectory ${catkin_LIBRARIES} ${alglib_LIBRARY})
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/sweetie_bot_resource_client_usage_example/ directory
#
# orocos_install_headers(sweetie_bot_resource_client_usage_example-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

