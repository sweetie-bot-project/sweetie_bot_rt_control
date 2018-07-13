Pose aggregator component
========================

This package is part of [Sweetie Bot project](http://sweetiebot.net). 
See complete specification [here (Rus)](https://gitlab.com/sweetie-bot/sweetie_doc/wikis/components-aggregator-gait).

### Components

`Aggregator` component composes full robot pose in joint space from partial pose messages (`in_joints` port, `sensor_msgs::JointState`).
Result pose is published on each update if  `publish_on_event` property is set or/and if timer synchronization message is received 
on `sync` port (`publish_on_timer` property).

Joints are sorted according its natural order described by `robot_model` service which should be loaded in the component.

## Testing

```
roslaunch sweetie_bot_aggregator test_aggregator.launch
rostopic echo /aggregator/output_joint_state
rosrun sweetie_bot_aggregator send_joint_state_partial.sh
rosrun sweetie_bot_aggregator send_joint_state_full.sh
```
