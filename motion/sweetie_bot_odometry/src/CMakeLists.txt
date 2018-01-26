# Creates a component library libsweetie_bot_odometry-<target>.so
# and installs in the directory lib/orocos/sweetie_bot_odometry/
#
orocos_component(sweetie_bot_odometry odometry-component.hpp odometry-component.cpp) # ...you may add multiple source files
target_link_libraries(sweetie_bot_odometry ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${USE_OROCOS_LIBRARIES})
#
# You may add multiple orocos_component statements.


