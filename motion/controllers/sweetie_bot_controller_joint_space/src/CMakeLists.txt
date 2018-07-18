orocos_component(follow_joint_state follow_joint_state-component.cpp) # ...you may add multiple source files
target_link_libraries(follow_joint_state ${catkin_LIBRARIES} ${Boost_LIBRARIES})

orocos_component(execute_joint_trajectory execute_joint_trajectory-component.cpp execute_joint_trajectory-base.cpp joint_trajectory_cache.cpp interpolation_algorithms.cpp) # ...you may add multiple source files
target_link_libraries(execute_joint_trajectory ${alglib_LIBRARY} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

orocos_component(torque_main_switch torque_main_switch-component.cpp) # ...you may add multiple source files
target_link_libraries(torque_main_switch ${catkin_LIBRARIES} ${Boost_LIBRARIES})

orocos_service(filter_joint_state_exp_service filter_joint_state_exp-service.cpp) # only one service per library!

orocos_service(filter_joint_state_exp_individual_service filter_joint_state_exp_individual-service.cpp) # only one service per library!

orocos_service(transient_joint_state_exp_service transient_joint_state_exp-service.cpp) # only one service per library!
