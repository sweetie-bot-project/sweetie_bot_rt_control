# Pose agregator component
This package is part of [Sweetie Bot project](http://sweetiebot.net). See complete specification [here (Rus)](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-agregator-gait).

This component buffers the incoming messages with a partial pose in the polar coordinate system and output full sorted pose.

### Input ports

`in_joints` (`JointState`, `EventPort`) --- the state of individual elements of the robot in the polar coordinate system.

### Output ports

`out_joints_sorted` (`JointState`) --- the state of the robot in the polar coordinate system, joints are sorted by chains.

### Plugins

Requires: `robot_model`

## Testing

```
roslaunch sweetie_bot_agregator agregator.launch
rostopic echo /agregator/output_joint_state
rosrun sweetie_bot_agregator send_joint_state_partial.sh
rosrun sweetie_bot_agregator send_joint_state_full.sh
```
