orocos_component(follow_joint_state follow_joint_state-component.cpp) # ...you may add multiple source files

orocos_component(animation_joint_trajectory animation_joint_trajectory-component.cpp animation_joint_trajectory-base.cpp joint_trajectory_cache.cpp) # ...you may add multiple source files
target_link_libraries(animation_joint_trajectory ${alglib_LIBRARY})

orocos_component(torque_main_switch torque_main_switch-component.cpp) # ...you may add multiple source files

orocos_service(transient_joint_state_exp_service transient_joint_state_exp-service.cpp) # only one service per library!

