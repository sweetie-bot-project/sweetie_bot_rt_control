# Sweetie Bot kinematic solver component

This package provides kinematics solver components.  It is part of [Sweetie Bot project](http://sweetiebot.net). 
See complete specification [here (Rus)](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-kinematics).

* `KinematicsFwd`  provides KDL-based forward kinematics. Component receives full robot pose in joint space via `in_joints_sorted` port 
    and calculates positions and velocities of the end segments of the kinematic chains listed in `kinematic_chains` property. 
	Calculated poses and velocities are published on `out_limbs` port (LimbState). All coordinates are relateve to chains' base link.

	Component is missing following capabilities: 
	* ROS `tf` calculation and publishing. 
	* Jacobian publishing
	
* `kinematicsInvTracIk` provides inverse kinematics based on TRAC_IK solver (inverse kinematics) and KDL (instantaneous kinematics).
    Component receive request in form of RigidBodyState message on  `in_limbs` port and publises result on `out_joints` port.
    Only declared in `kinematic_chain` property chains are processeed. If IK solution is not found for one of chain then
    all result message is discarded and filled with default failsafe values from `in_joints_seed_sorted` ports.
    Also pose at `in_joints_seed_sorted` port is used as first approximation.

    Component provides `poseToJointState()` operation for syncronious IK requests.

Components depend on `robot_model` service which provides URDF robot model and list of registered kinematic chains.

## Testing

### Direct kinematics

```
roslaunch sweetie_bot_kinematics test_kinematics.launch
rostopic echo /kinematics/output_limb_state
roscd sweetie_bot_kinematics scripts
rosbag play zero_joints.bag
rosbag play brohoof_joints.bag
```

### Inverse kinematics

```
roslaunch sweetie_bot_kinematics test_kinematics.launch
rostopic echo /kinematics/output_joint_state
roscd sweetie_bot_kinematics scripts
rosbag play zero_limbs.bag
rosbag play brohoof_limbs.bag
```
