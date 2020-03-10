orocos_component(pose_fusion_rtimulib pose_fusion_rtimulib-component.hpp pose_fusion_rtimulib-component.cpp) 
target_link_libraries(pose_fusion_rtimulib ${rtimulib_LIBRARY} ${catkin_LIBRARIES})
