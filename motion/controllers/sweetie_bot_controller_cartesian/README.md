Cartesian space movement controllers
================================

This package contains controllers which are working with targets defined in Cartesian coordinates.
All components implements resource management interface as described in documentation for `sweetie_bot_resource_control` package.
Both controllers are `SimpleControllerBase` descendants so can be activated by (`SetOperational` action, `std_srv::SetBool` 
ROS operation or by direct `start()/stop()` call.  New control cycle is triggered by sync port.:q

It is the part of [Sweetie Bot project](sweetiebot.net).  Full documentation 
is available in Russian [here](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-gait).

### Controllers

OROCOS component which control robot movements by publishing reference robot pose.

* `FollowStance` controller receives a desired body pose on `in_pose_ref` port and move body to it preserving all active contacts (support legs).
    If desired pose is unreachable the movement is performed only partially. The list of active contacts is received with SetOperational action.
	`support_legs` property contains default suooort leg list. It is used if `start()` or `rosSetOperational()` operations activate component.

    Component implements following algorithm. `filter_rigid_body_state` filter calculates small body displacement to move from current pose 
    to target robot pose. This displacment is passed to inverse kinematics which calculates desired robot pose in joint space in such a way
    that the  ends of legs are not moving in world frame.

    Interaction with IK is performed via `out_limbs_ref` port or `poseToJointStatePublish()` operation. In latter case 
    controller receives feedback if IK failed (pose is unreachble).

    On `out_base_ref` body pose in world frame is publised. It can be used to override odometry results.
    
    If property `pose_feedback` is set to false component ignores `in_base` and `in_limbs` port and track robot position itself
	using only initial pose. Property `activation_delay` delays initial pose acquition, so odometry component can adjust 
	base_link pose according to new contact set before initial pose is acquired.

    If static stability conditions for target pose are violated and property `balance_check` is set target pose is ignored.
    If `keep_balance` is set robot will try to move base to keep balance in current situation. 

* `FollowPose` controller tracks pose received on `in_pose_ref` port (`PoseStamped`) with specific limb. The limb is defined by 
    SetOperational action (resources field maust contains only one entity). Default value specified by `controlled_chains` property.

    Component implements the same algorithm as `FollowStance` controller. (Filter `filter_rigid_body_state` is used to calculate limb displacment).

### Filter plugins

OROCOS services which can be loaded into motion controllers. They calculates pose displacement for one control cycle given current and target Cartesian pose.
Also they can be used to smooth Cartesian space trajectory. Cartesian pose is represented by `KDL::Frame` and `KDL::Twist` (screw twist) pair.

* `filter_rigid_body_state_pd` PD regulator in Cartesian space. It approximates differential equation with Euler method so beware instability on fast settings.

