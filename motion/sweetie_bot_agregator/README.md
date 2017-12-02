# Pose agregator component
This package is part of [Sweetie Bot project](http://sweetiebot.net). 
See complete specification [here (Rus)](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-agregator-gait).

This component constricts full robot pose in joint space from partial pose messages (`in_joints` port).
Full pose is published on each pose update but at least one time per control cycle.
Joints are sorted according its natural order described by `robot_model`.

### Input ports

`in_joints` (`JointState`, `EventPort`) --- partial robot pose in joint space.
`sync` (`int`, `EventPort`) --- timer syncronization port.

### Output ports

`out_joints_sorted` (`JointState`) --- full robot pose in joint space.

### Plugins

Requires: `robot_model` service --- robots kinematics chains description.

## Testing

```
roslaunch sweetie_bot_agregator test_agregator.launch
rostopic echo /agregator/output_joint_state
rosrun sweetie_bot_agregator send_joint_state_partial.sh
rosrun sweetie_bot_agregator send_joint_state_full.sh
```
