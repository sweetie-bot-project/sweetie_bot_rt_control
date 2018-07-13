Sweetie Bot dynamic inverse components
=====================================

This package provides inverse dynamics solver components.  It is part of [Sweetie Bot project](http://sweetiebot.net). 
See complete specification [here (Rus)](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-dynamics).

## Components

`DynamicsInvSimple` component implements the simplest whole body robot controller (WBRC). 

It receives reference joint states on `in_joints_sorted` port (`JointState`), robot base position  on `in_base` 
port (`RigidBodyState`) and information about current contacts (`in_supports` port, `SupportState`). 
It differentiates velocity to get desired accelerations and calculate. 
Then `DynamicsInvSimple` uses robot dynamic model to calculate necessary joints efforts (`out_joint_accel_sorted` port, 
`JointStateAccel`), support reaction forces (`out_wrenches_fixed` port, `RigidBodyState`) and sum of reaction forces (`out_base`, `RigidBodyState`).  

Also it publishes robot pose in the world frame (`out_wrenches_fixed` and `out_base` ports) and information about robot balance (the center of mass, zero 
moment point (the center of pressure), support polygon) on `out_balance` port (`SupportState`).

## ROS nodes

`visualizer` display information about robot balance, reaction forces and velocities in rviz using `MarkerArray` mesages.
Also it converts `JointStateAccel` messages to `JointState`.

### ROS interface

#### Publish topics

* `marker_array` (`visualization_msgs::MarkerArray`) --- visualization for  rviz.
* `joints` (`JointState`) --- `JointStateAccel` without acceleration.


#### Subscribe topics

* `joint_state_accel` (`JointStateAccel`) --- joints state with acceleration from dynamics component.
* `wrenches` (`RigidBodyState`) --- kinematics chains end links state and reaction forces (world frame).
* `supports` (`SupportState`) --- active contacts.
* `balance` (`BalanceState`) --- information about robot balance.

#### Actionlib

* Client: `set_operational_action` (`SetOperational`) --- activate or deactivate corresponding controller with resource set selected in context menu.

#### Parameters

* `~robot_model_namespace` (`string`, "") --- namespace with `robot_model` properties loaded to ROS server.
* `~point_size` (`double`, 0.005) --- size of points.
* `~torque_scale` (`double`, 0.01) --- torque scale factor (from Nm to m).
* `~force_scale` (`double`, 0.01) --- force scale factor (from N to m)
* `~display_twist` (`bool`, true) --- torque scale factor.
* `~velocity_angular_scale` (`double`, 1/(2*pi)) --- twist scale factor.
