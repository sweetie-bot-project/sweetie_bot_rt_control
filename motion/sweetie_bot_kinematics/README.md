# Sweetie Bot kinematic solver component

This package provides kinematics solver components.  It is part of [Sweetie Bot project](http://sweetiebot.net). 
See complete specification [here (Rus)](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-kinematics).

* `kinematics_fwd`  provides KDL-based forward kinematics. Component receives full robot pose in joint space via `in_joints_sorted` port 
    and calculates positions and velocities of the end segments of the kinematic chains listed in `kinematic_chains` property. 
	Calculated poses and velocities are published on `out_limbs` port (LimbState). All coordinates are relateve to chains' base link.

	Component is missing following capabilities: 
	* ROS `tf` calculation and publishing. 
	* Jacobian publishing
	
* `kinematics` is an old version of kinematic solving component. Can solve forward and inverse kinematics problem at the same time.
    * Forward kinematics: input port is `in_joints_sorted` (JointState), output port is `out_limbs`. KDL recursive solver is used.
	* Inverse kinematics: input port is `in_limbs` (LimbsState), output port is `out_joints`. Full joint space pose received on 
	    `in_joints_seed_sorted` is used as starting point for inverse kinematics algorithm. This componet uses trac_ik solver.

	Not implemented:
    1. Global solver parameters
    2. Operations

Components depend on `robot_model` service which provides URDF robot model and list of registered kinematic chains.


## Testing

### Direct kinematics

```
roslaunch sweetie_bot_kinematics test_kinematics.launch
rostopic echo /kinematics/output_limb_state
rosrun sweetie_bot_kinematics send_joint_state.sh
```

### Inverse kinematics

```
roslaunch sweetie_bot_kinematics test_kinematics.launch
rostopic echo /kinematics/output_joint_state
rosrun sweetie_bot_kinematics send_cartesian.sh
rosrun sweetie_bot_kinematics send_cartesian2.sh
```

### Visualising inverse kinematics

```
roslaunch sweetie_bot_kinematics test_kinematics_agregator_rviz.launch
rosrun sweetie_bot_kinematics send_cartesian.sh
rosrun sweetie_bot_kinematics send_cartesian2.sh
```
