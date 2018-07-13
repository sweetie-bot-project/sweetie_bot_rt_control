Sweetie Bot controllers
=============================

Controllers produce reference robot pose which eventually is passed to robot servos.

Depending on the control goal controllers are divided in following groups:
* `Follow*` controllers follow desired pose presented in joint or Cartesian space.
* `Execute*` controllers execute movements.
* `Move*`, `Keep*` controllers move robot to desired pose or keeps current pose.
There are may be another types of controllers. For example, `MainTroqueSwitch` turns off servos.

## Conventions

* Usually controller requires `robot_model` service to function.

* Controllers implement resource client interface by using `ResourceClient` service or by being 
    a descendant of `SimpleControllerBase` class. See @ref `sweetie_bot_resource_control` for details.

* When controller is operational it publishes the reference pose in joints space directly or by invoking 
    inverse kinematics components. When controller is not operational it shoud not publish anything.

* Controllers are activated by actionlib actions. It is preferable way because allows a client to monitor 
   controller state. But they also can support activation by OROCOS operations.

   For example, `SimpleControllerBase` class descendants can be activated by
   * `SetOperational` action,
   * `start()/stop()` OROCOS operation,
   * `std_srv::SetBool` ROS service (`rosSetOperational()` OROCOS operation).
