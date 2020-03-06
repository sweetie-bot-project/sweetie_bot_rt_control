Joint space movement controllers
================================

This package contains controllers and filter plugins which are working on the level of the robot joints. 
All components implements resource management interface as described in documentation for `sweetie_bot_resource_control` package.
New control cycle of all controllers is triggered by sync port.
It is the part of [Sweetie Bot project](sweetiebot.net).  Full documentation 
is available in Russian [here](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-gait).

### Controllers

OROCOS components which control robot movements by publishing reference robot pose.

* `FollowJointTrajectory` monitor `in_joints_ref` port and use filter plugin to merge received pose 
    with current robot pose on `in_joints_sorted`. Then result is published on `out_joints_fixed` port.
	All messages have `sensor_msgs::JointState` type. The list of controlled kinematic chains 
	is specified via pararmeter `controlled_chains`. 

	Component is `SimpleControllerBase` descendant so can be activated by `SetOperational` action, `start/stop` or `rosSetOperational`
    operations. After actiavation it publishes current robot pose on `out_joints_src_reset` port for `activation_delay` cycles.
	During activation resource list may contain the list of joints names. In the latter case corresponding list of chains 
	is derived automatically. Note that mixing chains names and joint names is not supported.

	Possible usage pattern: connect `joint_state_publiser` GUI to `in_joints_ref` and `out_joints_src_reset` ports.

* `ExecuteJointTrajectory` provides `FollowJointTrajectory` actionlib server. Received trajectory is interpolated by
    thrid order spline, only trajectory point positions are used, information about speed abd acceleration is ignored.
	Component uses filter plugin to merge current robot pose (`in_joints_sorted` port) and publish pose on `out_joints_ref_fixed` port.

* `MainTorqueSwitch` switches selected group of servos off when it is active. The names of `HerkulexSched` and `HerkulexArray` components 
    (`sweetie_bot_herkulex_control` package) should be provided via component properties. If component active it publises 
	current robot pose received from sensors (`in_joints_actual` port) on `out_joints_ref` port.

	Component can be activated by `SetOperational` action or `rosSetOperational` and `setOperational` operations.
	The list of resources is list of joints groups to be turned off.

	DIRECT CALL TO `start` OPERATION IS IGNORED, `stop` DEACTIVATES CONTROLLER BUT DOES NOT TURN SERVOS ON.
	`stop()` opeation should be used only during robot deactivation sequence. 
	Use `setOperational()` operation to activate or deactivate component from OROCOS context.

### Filter plugins

OROCOS services which can be loaded into motion controllers. They are used for trajectory preprocessing.

* `filter_joint_state_exp` is exponential filter with critical damping. It can be used to smooth abruptly reference position changes.
    Trancient time can be tuned with `trancient_time` parameter.

* `filter_joint_state_exp_individual` is also exponential filter. It allows per joint transient time settings.

* `transient_joint_state_exp` is exponential filter which takes in account only initial error. Can be useful initial pose of 
    movement trajectory and actual robot pose are not exactly same.

