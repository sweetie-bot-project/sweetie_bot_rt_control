orocos_component(follow_stance follow_stance-component.cpp) # ...you may add multiple source files
target_link_libraries(follow_stance ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#target_link_libraries(follow_stance ${catkin_LIBRARIES})

orocos_component(follow_pose follow_pose-component.cpp) # ...you may add multiple source files
target_link_libraries(follow_pose ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#target_link_libraries(follow_pose ${catkin_LIBRARIES})

orocos_component(execute_step_sequence execute_step_sequence-component.cpp cartesian_trajectory_cache.cpp) # ...you may add multiple source files
target_link_libraries(execute_step_sequence ${catkin_LIBRARIES})

orocos_service(filter_rigid_body_state_pd_service filter_rigid_body_state_pd-service.cpp) # only one service per library!
target_link_libraries(filter_rigid_body_state_pd_service ${catkin_LIBRARIES})
