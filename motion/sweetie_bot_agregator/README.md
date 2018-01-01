Pose aggregator component
========================

This package is part of [Sweetie Bot project](http://sweetiebot.net). 
See complete specification [here (Rus)](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-agregator-gait).

This component composes full robot pose in joint space from partial pose messages (`in_joints` port, `sensor_msgs::JointState`).
Result pose is published on each update (`publish_on_event` property is set) or/and if timer synchronization message is received 
on `sync` port.(`publish_on_timer` property is set).

Joints are sorted according its natural order described by `robot_model` service.

## Testing

```
roslaunch sweetie_bot_agregator test_agregator.launch
rostopic echo /agregator/output_joint_state
rosrun sweetie_bot_agregator send_joint_state_partial.sh
rosrun sweetie_bot_agregator send_joint_state_full.sh
```
