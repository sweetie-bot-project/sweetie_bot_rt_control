# Sweetie Bot kinematic solver component

This package provides kinematics solver components.  It is part of [Sweetie Bot project](http://sweetiebot.net). 
See complete specification [here (Rus)](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-kinematics).

## Components

* `KinematicsFwd` component provides KDL-based forward kinematics. Component receives full robot pose in joint space via `in_joints_sorted` port 
    and calculates positions and velocities of the end segments of the kinematic chains listed in `kinematic_chains` property. 
	Calculated poses and velocities are published on `out_limbs` port (LimbState). All coordinates are relateve to chains' base link.

	Component is missing following capabilities: 
	* ROS `tf` calculation and publishing. 
	* Jacobian publishing

* `KinematicsFwdForce` component works the same as `KinematicsFwd` but also performs forces conversion.
	
* `KinematicsInv` component performs inverse kinematics transformation. Component processes only kinematic chains with names
    listed in `kinematic_chains` property.
    
    Instantaneous transforamtion is provided with KDL pseudoinverse solver. It can be configured via 
    properties `eps_vel` (it uesd for signulatirity detection), `max_iterations` and `zero_vel_at_singularity` properties.
    
    Solvers for inverse kinematics problems are provided with `solver_ik_factory_*` services loaded in component.
    This mechanism allows to use different solvers for different kinematics chains.

    Component receive request in form of RigidBodyState message on `in_limbs` port and publises result on `out_joints` port.
	Current pose is received on `in_joints_seed_sorted` and can be used as seed for iterative solvers if otherwise is not specified (`use_ik_pose_as_seed` property).
    If IK solution is not found for one of chain then all result message is discarded and filled with default failsafe values from `in_joints_seed_sorted` ports. 
	Additionaly joint shifts between solution and previous pose are checked that it is not greater then product of poperties `period` and `max_joint_velocity`

    Component provides `poseToJointState()` and `poseToJointStatePublish()` operation for syncronious IK requests.

	Joint limints and solver tolerance can be assigned via dynamic properties:

	    <chain>_q_min (double[]) --- lower bounds, array length equal to number of joints.
	    <chain>_q_max (double[]) --- upper bounds.
		<chain>_tolerance (double[]) --- tolerance values in format [x, y, z, rx, ry, rz]. 

	Components depend on `robot_model` service which provides URDF robot model and list of registered kinematic chains.

## Services

* `solver_ik_factory_trac_ik` service provides TRAC_IK based iterative solvers. Only chains form `kinematic_chains` property is processed.
	Solvers parameters: `eps_pos` (singularity detection) and `timeout`.

* `solver_ik_factory_analytical` service provides analytical IK solvers mainly for Proto2. Only chains form `kinematic_chains` property is processed.

### Writing our own solver factory

Solver factory Service should implement `SolverIKFactoryInterface` interface. Solvers must implements `SolverIKInterface`.
See `solver_ik.hpp` and `solver_ik_trac_ik.cpp` as example.

## Testing

### Direct kinematics

```
roslaunch sweetie_bot_kinematics test_kinematics.launch
rostopic echo /kinematics/output_limb_state
rosrun sweetie_bot_kinematics send_joints_zero.sh
rosrun sweetie_bot_kinematics send_joints_brohoof.sh
```

You should see an output message on the `output_limb_state` topic.

### Inverse kinematics

```
roslaunch sweetie_bot_kinematics test_kinematics.launch
rostopic echo /kinematics/output_joint_state
rosrun sweetie_bot_kinematics send_joints_seed.sh
rosrun sweetie_bot_kinematics send_limbs_zero.sh
rosrun sweetie_bot_kinematics send_limbs_brohoof.sh
```

You should see two output message on the `output_joint_state` topic and should not see any error on a component.

