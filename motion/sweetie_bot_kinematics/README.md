# Sweetie Bot kinematic solver component
This package is part of [Sweetie Bot project](http://sweetiebot.net). See complete specification [here (Rus)](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-kinematics).

This component provides kinematics solvers (forward, reverse, instantaneous).
It provides various interfaces: ports, operations.

## Input ports
### Direct kinematics

`in_joints_sorted` (`JointState`) --- state of the robot in the corner SC (sorted).

### Inverse kinematics

`in_joints_seed_sorted` (`JointState`) --- initial state of the robot in the polar coordinate system (sorted).

`in_limbs` (`CartesianState`) --- state of the robot in a cartesian coordinate system (real or desired by the sensors).

## Output ports

### Direct kinematics

`out_limbs` (`CartesianState`) --- state of the robot in a cartesian coordinate system (real or desired by the sensors).

### Inverse kinematics

`out_joints` (`JointState`) --- state of the robot in the polar coordinate system (real or desired by the sensors).

### Plugins

Requires: `robot_model`

### Not implemented yet

1. Global solver parameters

2. Operations

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
