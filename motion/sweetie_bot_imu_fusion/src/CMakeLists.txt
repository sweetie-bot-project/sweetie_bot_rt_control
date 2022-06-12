orocos_component(pose_fusion_rtimulib pose_fusion_rtimulib-component.cpp) 
target_link_libraries(pose_fusion_rtimulib ${catkin_LIBRARIES})

orocos_component(pose_fusion_bno080_rvc pose_fusion_bno080_rvc-component.cpp) 
target_link_libraries(pose_fusion_bno080_rvc ${catkin_LIBRARIES})
