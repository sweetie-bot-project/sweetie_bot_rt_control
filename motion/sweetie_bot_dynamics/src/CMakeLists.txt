
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

orocos_component(dynamics_inv_simple dynamics_inv_simple-component.hpp dynamics_inv_simple-component.cpp) # ...you may add multiple source files
target_link_libraries(dynamics_inv_simple ${catkin_LIBRARIES} ${RBDL_URDFREADER_LIBRARY})
set_target_properties(dynamics_inv_simple PROPERTIES LINK_FLAGS "${RBDL_LDFLAGS}")

add_executable(visualizer visualizer.cpp)
target_link_libraries(visualizer ${catkin_LIBRARIES})
# node installation
install(TARGETS visualizer
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# Additional headers:
#
# Installs in the include/orocos/sweetie_bot_kinematics/ directory
#
#orocos_install_headers(kinematics-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.
